"""
Multi-View Triangulator

Estimates 3D drone positions from noisy 2D detections in several calibrated
cameras. The detector receives only pixel coordinates; it must work out which
blob in each camera belongs to the same drone (cross-view association) before it
can triangulate.

Algorithm
1. Hypothesis generation: for every pair of cameras and every pair of
   detections, intersect the two pixel rays. Keep the pair if the closest-
   approach point reprojects within the noise gate in both images.
2. Verification: project each hypothesis into every camera. A camera *supports*
   it if a detection lies within the gate; it *contradicts* it if the point is
   inside that camera's field of view and range but nothing was detected there.
   Ghost points (rays from different drones that happen to cross) are usually
   contradicted by the other views.
3. Greedy assignment: repeatedly accept the hypothesis with the most support
   (ties broken by reprojection error), using each detection at most once.
   Accept if support >= 3 with at most `max_contradictions`, or support == 2
   with no contradictions.
4. Refinement: least-squares on reprojection error over all supporting views;
   covariance from the Jacobian.
"""

from typing import List, Tuple
import numpy as np
from scipy.optimize import least_squares

from camera_geometry import CameraModel, gate_px, closest_points_between_rays, intersect_rays
from detection_types import DetectedTarget, DetectionMethod


class MultiViewTriangulator:
    def __init__(self, simulation_bounds=None, max_contradictions: int = 1,
                 bounds_margin: float = 50.0):
        self.bounds = simulation_bounds
        self.max_contradictions = max_contradictions
        self.bounds_margin = bounds_margin

    # ------------------------------------------------------------------
    def detect(self, observations: List, timestamp: float) -> List[DetectedTarget]:
        obs = [o for o in observations if o.camera is not None]
        if len(obs) < 2:
            return []
        cams = [CameraModel(o.camera) for o in obs]
        gates = np.array([gate_px(o) for o in obs])
        dets = [o.image_coordinates for o in obs]
        rays = [cam.pixel_to_ray(d) if len(d) else np.empty((0, 3)) for cam, d in zip(cams, dets)]

        hyps = self._generate_hypotheses(cams, dets, rays, gates)
        if len(hyps) == 0:
            return []

        support, n_contra, err = self._verify(hyps, cams, dets, gates)
        used = [np.zeros(len(d), dtype=bool) for d in dets]
        alive = np.ones(len(hyps), dtype=bool)
        targets = []

        while alive.any():
            # effective support: ignore detections already assigned to a target
            eff = support.copy()
            for k in range(len(cams)):
                col = eff[:, k]
                m = col >= 0
                col[m] = np.where(used[k][col[m]], -1, col[m])
            n_sup = (eff >= 0).sum(axis=1)
            ok = alive & (((n_sup >= 3) & (n_contra <= self.max_contradictions))
                          | ((n_sup == 2) & (n_contra == 0)))
            if not ok.any():
                break
            score = np.where(ok, n_sup * 1000.0 - err, -np.inf)
            i = int(np.argmax(score))
            alive[i] = False

            views = [(k, int(eff[i, k])) for k in range(len(cams)) if eff[i, k] >= 0]
            target = self._refine(hyps[i], views, cams, dets, obs, gates, timestamp)
            if target is None:
                continue
            for k, j in views:
                used[k][j] = True
            targets.append(target)
        return targets

    # ------------------------------------------------------------------
    def _in_bounds(self, X: np.ndarray) -> np.ndarray:
        if self.bounds is None:
            return np.ones(len(X), dtype=bool)
        b, m = self.bounds, self.bounds_margin
        return ((X[:, 0] >= b.x_min - m) & (X[:, 0] <= b.x_max + m) &
                (X[:, 1] >= b.y_min - m) & (X[:, 1] <= b.y_max + m) &
                (X[:, 2] >= b.z_min - m) & (X[:, 2] <= b.z_max + m))

    def _generate_hypotheses(self, cams, dets, rays, gates) -> np.ndarray:
        out = []
        for a in range(len(cams)):
            for b in range(a + 1, len(cams)):
                if len(dets[a]) == 0 or len(dets[b]) == 0:
                    continue
                d1 = np.broadcast_to(rays[a][:, None, :], (len(dets[a]), len(dets[b]), 3))
                d2 = np.broadcast_to(rays[b][None, :, :], (len(dets[a]), len(dets[b]), 3))
                o1 = np.broadcast_to(cams[a].position, d1.shape)
                o2 = np.broadcast_to(cams[b].position, d2.shape)
                with np.errstate(invalid='ignore'):
                    mid, gap, t1, t2 = closest_points_between_rays(o1, d1, o2, d2)
                mid = mid.reshape(-1, 3)
                valid = (np.isfinite(mid).all(axis=1)
                         & (np.nan_to_num(t1).ravel() > 0) & (np.nan_to_num(t2).ravel() > 0))
                valid &= self._in_bounds(np.nan_to_num(mid))
                if not valid.any():
                    continue
                X = mid[valid]
                ia, ib = np.unravel_index(np.flatnonzero(valid), (len(dets[a]), len(dets[b])))
                ea = np.linalg.norm(cams[a].project(X) - dets[a][ia], axis=1)
                eb = np.linalg.norm(cams[b].project(X) - dets[b][ib], axis=1)
                keep = (ea <= gates[a]) & (eb <= gates[b])
                if keep.any():
                    out.append(X[keep])
        return np.vstack(out) if out else np.empty((0, 3))

    def _verify(self, X, cams, dets, gates) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        n, K = len(X), len(cams)
        support = np.full((n, K), -1, dtype=int)
        contra = np.zeros(n, dtype=int)
        err_sum = np.zeros(n)
        for k, cam in enumerate(cams):
            iv = cam.can_see(X)
            if len(dets[k]) == 0:
                contra += iv
                continue
            pix = cam.project(X)
            dist = np.linalg.norm(pix[:, None, :] - dets[k][None, :, :], axis=2)
            dist = np.nan_to_num(dist, nan=np.inf)
            j = np.argmin(dist, axis=1)
            dmin = dist[np.arange(n), j]
            hit = dmin <= gates[k]
            support[hit, k] = j[hit]
            err_sum[hit] += dmin[hit]
            contra += iv & ~hit
        n_sup = np.maximum((support >= 0).sum(axis=1), 1)
        return support, contra, err_sum / n_sup

    def _refine(self, X0, views, cams, dets, obs, gates, timestamp):
        origins = np.array([cams[k].position for k, _ in views])
        dirs = np.vstack([cams[k].pixel_to_ray(dets[k][j]) for k, j in views])
        x_lin = intersect_rays(origins, dirs)
        if not np.all(np.isfinite(x_lin)):
            x_lin = X0
        sig = np.array([obs[k].pixel_noise_std or obs[k].camera.spec.noise_std for k, _ in views])

        def resid(x):
            r = [(cams[k].project(x[None])[0] - dets[k][j]) / s for (k, j), s in zip(views, sig)]
            return np.nan_to_num(np.concatenate(r), nan=1e3)

        sol = least_squares(resid, x_lin, method='lm' if 2 * len(views) >= 3 else 'trf')
        x = sol.x
        # reject if refinement left any supporting view outside its gate
        for (k, j) in views:
            if not np.linalg.norm(cams[k].project(x[None])[0] - dets[k][j]) <= gates[k]:
                return None
        try:
            cov = np.linalg.inv(sol.jac.T @ sol.jac)
        except np.linalg.LinAlgError:
            cov = np.eye(3) * 1e4
        conf = float(np.mean([obs[k].confidence_scores[j] for k, j in views]))
        return DetectedTarget(
            position=x, velocity=np.zeros(3), confidence=conf, volume_estimate=0.0,
            contributing_sensors=[cams[k].camera_id for k, _ in views],
            detection_method=DetectionMethod.TRIANGULATION,
            covariance_matrix=cov, timestamp=timestamp)
