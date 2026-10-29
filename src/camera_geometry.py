"""
Camera geometry shared by all detection methods.

Detectors only ever see what a real passive camera reports: noisy pixel
coordinates plus the camera's own calibration (position, heading, field of view,
resolution, range limits, pixel-noise level). This module turns that into 3D
geometry: projecting world points into an image, turning a pixel into a ray,
intersecting rays, and finding the voxels a detection ray passes through.

The projection model mirrors VirtualSensorArray._world_to_image_coordinates
(angular model, yaw-only rotation), so detectors and simulator agree.
"""

from typing import List, Tuple
import numpy as np


class CameraModel:
    """Calibrated camera built from a sensor_simulation.CameraPosition."""

    def __init__(self, camera):
        self.camera_id = camera.camera_id
        self.position = np.asarray(camera.position, dtype=float)
        yaw = np.radians(camera.orientation[0])
        self.cos_yaw, self.sin_yaw = np.cos(yaw), np.sin(yaw)
        self.hfov = np.radians(camera.spec.field_of_view[0])
        self.vfov = np.radians(camera.spec.field_of_view[1])
        self.width, self.height = camera.spec.resolution
        self.min_range = camera.spec.min_range
        self.max_range = camera.spec.max_range
        self.base_noise_std = camera.spec.noise_std
        # pixels per radian (horizontal / vertical)
        self.px_per_rad_h = self.width / self.hfov
        self.px_per_rad_v = self.height / self.vfov

    # --- world <-> camera -------------------------------------------------
    def _to_cam(self, X: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        rel = np.atleast_2d(X) - self.position
        cx = rel[:, 0] * self.cos_yaw + rel[:, 1] * self.sin_yaw
        cy = -rel[:, 0] * self.sin_yaw + rel[:, 1] * self.cos_yaw
        cz = rel[:, 2]
        return cx, cy, cz

    def project(self, X: np.ndarray) -> np.ndarray:
        """World points (N,3) -> pixel coordinates (N,2). Points behind the camera get NaN."""
        cx, cy, cz = self._to_cam(X)
        h = np.arctan2(cy, cx)
        v = np.arctan2(cz, cx)
        px = (h / (self.hfov / 2) + 1) * self.width / 2
        py = (1 - v / (self.vfov / 2)) * self.height / 2
        out = np.column_stack([px, py])
        out[cx <= 0] = np.nan
        return out

    def can_see(self, X: np.ndarray) -> np.ndarray:
        """Which world points lie inside this camera's field of view and range limits."""
        cx, cy, cz = self._to_cam(X)
        rng = np.sqrt(cx ** 2 + cy ** 2 + cz ** 2)
        front = cx > 0
        h = np.abs(np.arctan2(cy, cx))
        v = np.abs(np.arctan2(cz, cx))
        return (front & (h <= self.hfov / 2) & (v <= self.vfov / 2)
                & (rng >= self.min_range) & (rng <= self.max_range))

    def ranges(self, X: np.ndarray) -> np.ndarray:
        return np.linalg.norm(np.atleast_2d(X) - self.position, axis=1)

    def pixel_to_ray(self, pix: np.ndarray) -> np.ndarray:
        """Pixel coordinates (N,2) -> unit ray directions in world frame (N,3)."""
        pix = np.atleast_2d(pix)
        h = (pix[:, 0] * 2 / self.width - 1) * self.hfov / 2
        v = (1 - pix[:, 1] * 2 / self.height) * self.vfov / 2
        d0 = np.ones(len(pix))
        d1 = np.tan(h)
        d2 = np.tan(v)
        dx = d0 * self.cos_yaw - d1 * self.sin_yaw
        dy = d0 * self.sin_yaw + d1 * self.cos_yaw
        d = np.column_stack([dx, dy, d2])
        return d / np.linalg.norm(d, axis=1, keepdims=True)

    def angular_radius_px(self, radius_m: float, X: np.ndarray) -> np.ndarray:
        """Approximate image radius (px) of a sphere of radius_m at points X."""
        return radius_m / np.maximum(self.ranges(X), 1e-6) * max(self.px_per_rad_h, self.px_per_rad_v)


def gate_px(obs, n_sigma: float = 3.5, floor: float = 4.0) -> float:
    """Pixel association gate from the camera's calibrated noise level."""
    sigma = obs.pixel_noise_std if obs.pixel_noise_std is not None else obs.camera.spec.noise_std
    return max(floor, n_sigma * sigma + 1.0)  # +1 px for blob-centroid quantisation


def closest_points_between_rays(o1, d1, o2, d2):
    """Vectorised closest approach of ray pairs. Returns (midpoints, gap, t1, t2)."""
    w0 = o1 - o2
    a = np.sum(d1 * d1, axis=-1)
    b = np.sum(d1 * d2, axis=-1)
    c = np.sum(d2 * d2, axis=-1)
    d = np.sum(d1 * w0, axis=-1)
    e = np.sum(d2 * w0, axis=-1)
    denom = a * c - b * b
    denom = np.where(np.abs(denom) < 1e-12, np.nan, denom)
    t1 = (b * e - c * d) / denom
    t2 = (a * e - b * d) / denom
    p1 = o1 + t1[..., None] * d1
    p2 = o2 + t2[..., None] * d2
    return (p1 + p2) / 2, np.linalg.norm(p1 - p2, axis=-1), t1, t2


def intersect_rays(origins: np.ndarray, dirs: np.ndarray) -> np.ndarray:
    """Least-squares point closest to a set of rays (linear triangulation)."""
    A = np.zeros((3, 3))
    b = np.zeros(3)
    for o, d in zip(origins, dirs):
        P = np.eye(3) - np.outer(d, d)
        A += P
        b += P @ o
    return np.linalg.lstsq(A, b, rcond=None)[0]


def cone_voxels(cam: CameraModel, pixel: np.ndarray, origin: np.ndarray, res: float,
                dims: Tuple[int, int, int], tol_px: float) -> np.ndarray:
    """
    Flat indices of voxels whose centres may project within tol_px of `pixel`:
    walk the detection ray through the grid and include neighbours out to the
    cone radius at each depth. This is only a cheap candidate (region-of-
    interest) step; the exact test is done by projecting voxel centres.
    """
    d = cam.pixel_to_ray(pixel)[0]
    dims = np.asarray(dims)
    lo = origin
    hi = origin + dims * res
    # parametric entry/exit of the ray through the grid box
    with np.errstate(divide='ignore', invalid='ignore'):
        t_a = (lo - cam.position) / d
        t_b = (hi - cam.position) / d
    t_near = np.nanmax(np.minimum(t_a, t_b))
    t_far = np.nanmin(np.maximum(t_a, t_b))
    t_near = max(t_near, cam.min_range)
    t_far = min(t_far, cam.max_range)
    if not np.isfinite(t_near) or not np.isfinite(t_far) or t_far <= t_near:
        return np.empty(0, dtype=np.int64)

    ts = np.arange(t_near, t_far + res / 2, res / 2)
    pts = cam.position + ts[:, None] * d
    radius_m = tol_px / cam.px_per_rad_v * ts + res * 0.87
    k = np.ceil(radius_m / res).astype(int)
    base = np.floor((pts - origin) / res).astype(int)

    out = []
    for kk in np.unique(k):
        sel = base[k == kk]
        r = np.arange(-kk, kk + 1)
        off = np.stack(np.meshgrid(r, r, r, indexing='ij'), -1).reshape(-1, 3)
        idx = (sel[:, None, :] + off[None, :, :]).reshape(-1, 3)
        ok = np.all((idx >= 0) & (idx < dims), axis=1)
        idx = idx[ok]
        out.append(np.ravel_multi_index(idx.T, dims))
    if not out:
        return np.empty(0, dtype=np.int64)
    return np.unique(np.concatenate(out))


def candidate_voxels(observations, cams: List[CameraModel], origin, res, dims,
                     min_cameras: int = 2) -> np.ndarray:
    """Voxels lying in the detection cones of at least `min_cameras` distinct cameras."""
    n = int(np.prod(dims))
    counts = np.zeros(n, dtype=np.uint8)
    for obs, cam in zip(observations, cams):
        if len(obs.image_coordinates) == 0:
            continue
        tol = gate_px(obs)
        per_cam = [cone_voxels(cam, p, origin, res, dims, tol) for p in obs.image_coordinates]
        per_cam = np.unique(np.concatenate(per_cam)) if per_cam else np.empty(0, dtype=np.int64)
        counts[per_cam] += 1
    return np.flatnonzero(counts >= min_cameras)


def view_evidence(observations, cams: List[CameraModel], X: np.ndarray, res: float):
    """
    For voxel centres X (M,3), per camera: is the voxel in view, and does a
    detection fall inside its projected footprint? Returns a dict of
    (n_cams, M) arrays: in_view, hit, conf (matched detection confidence),
    det (matched detection index or -1), resid (pixel distance to it).
    """
    m, K = len(X), len(cams)
    ev = {'in_view': np.zeros((K, m), dtype=bool), 'hit': np.zeros((K, m), dtype=bool),
          'conf': np.zeros((K, m)), 'det': np.full((K, m), -1, dtype=int),
          'resid': np.zeros((K, m))}
    for k, (obs, cam) in enumerate(zip(observations, cams)):
        iv = cam.can_see(X)
        ev['in_view'][k] = iv
        if len(obs.image_coordinates) == 0 or not iv.any():
            continue
        Xi = X[iv]
        pix = cam.project(Xi)
        dist = np.linalg.norm(pix[:, None, :] - obs.image_coordinates[None, :, :], axis=2)
        j = np.argmin(dist, axis=1)
        dmin = dist[np.arange(len(j)), j]
        h = dmin <= gate_px(obs) + cam.angular_radius_px(res * 0.87, Xi)
        idx = np.flatnonzero(iv)
        ev['hit'][k, idx] = h
        ev['conf'][k, idx] = np.where(h, obs.confidence_scores[j], 0.0)
        ev['det'][k, idx] = np.where(h, j, -1)
        ev['resid'][k, idx] = np.where(h, dmin, 0.0)
    return ev


def exclusive_peaks(X: np.ndarray, det: np.ndarray, score: np.ndarray, res: float,
                    min_views: int = 2):
    """
    Greedy target extraction from scored voxel hypotheses. Highest-scoring
    voxel first; a voxel becomes a target only if at least `min_views` of its
    supporting detections are not already explained by an earlier target. Its
    detections are then marked used, so each image blob explains one drone.
    This suppresses visual-hull ghosts (voxels where rays from *different*
    drones cross), which otherwise merge neighbouring drones into one blob.

    Returns a list of (position, member_voxel_indices, views) where views is a
    list of (camera_index, detection_index).
    """
    K, M = det.shape
    order = np.argsort(-score)
    used = [set() for _ in range(K)]
    peaks = []
    for i in order:
        views = [(k, int(det[k, i])) for k in range(K) if det[k, i] >= 0]
        fresh = [(k, j) for k, j in views if j not in used[k]]
        if len(fresh) < min_views or len(fresh) * 2 <= len(views):
            continue
        # member voxels: nearby voxels supported by the same detections
        near = np.flatnonzero(np.linalg.norm(X - X[i], axis=1) <= 1.5 * res)
        same = np.zeros(len(near), dtype=int)
        for k, j in fresh:
            same += det[k, near] == j
        members = near[same >= min(len(fresh), max(min_views, len(fresh) - 1))]
        w = np.maximum(score[members] - score[members].min() + 1e-3, 1e-3)
        pos = np.average(X[members], axis=0, weights=w)
        for k, j in fresh:
            used[k].add(j)
        peaks.append((pos, members, fresh))
    return peaks
