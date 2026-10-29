"""
Space Carving (visual hull of point detections)

A voxel survives carving only if every camera that should be able to see it
has a detection inside the voxel's projected footprint, and at least
`min_views` cameras do. Inputs are pixel detections plus camera calibration
only. Because this is a hard consistency test, a single missed detection
(e.g. poor visibility at long range) carves the drone away; that fragility is a
real property of the method.

Targets are extracted from the surviving voxels with exclusive peak picking
(camera_geometry.exclusive_peaks): best-supported voxel first, each image blob
explaining at most one target. Plain connected-component labelling would merge
neighbouring drones through visual-hull ghost voxels.

For speed only voxels inside the detection cones of >= `min_views` cameras are
tested (region-of-interest step); no other voxel could survive carving.
"""

from typing import List
import numpy as np

from bayesian_occupancy_updater import VoxelGrid
from camera_geometry import CameraModel, candidate_voxels, view_evidence, exclusive_peaks
from detection_types import DetectedTarget, DetectionMethod


class SpaceCarvingAlgorithm:
    def __init__(self, grid: VoxelGrid, min_views: int = 2):
        self.grid = grid
        self.min_views = min_views
        self.carved_volume = None

    def _evaluate(self, observations):
        obs = [o for o in observations if o.camera is not None]
        dims = tuple(self.grid.dimensions)
        if len(obs) < self.min_views:
            return obs, None, np.empty(0, dtype=np.int64), None, None
        cams = [CameraModel(o.camera) for o in obs]
        res = self.grid.resolution
        cand = candidate_voxels(obs, cams, self.grid.origin, res, dims, self.min_views)
        if len(cand) == 0:
            return obs, cams, cand, None, None
        X = self.grid.centers(cand)
        ev = view_evidence(obs, cams, X, res)
        keep = np.all(ev['hit'] | ~ev['in_view'], axis=0) & (ev['hit'].sum(axis=0) >= self.min_views)
        return obs, cams, cand, ev, keep

    def carve_space(self, observations: List, timestamp: float) -> np.ndarray:
        """Boolean volume of voxels consistent with every view."""
        vol = np.zeros(tuple(self.grid.dimensions), dtype=bool)
        _, _, cand, ev, keep = self._evaluate(observations)
        if keep is not None:
            vol.ravel()[cand[keep]] = True
        self.carved_volume = vol
        return vol

    def detect(self, observations: List, timestamp: float) -> List[DetectedTarget]:
        obs, cams, cand, ev, keep = self._evaluate(observations)
        if keep is None or not keep.any():
            return []
        sel = np.flatnonzero(keep)
        X = self.grid.centers(cand[sel])
        n_hit = ev['hit'][:, sel].sum(axis=0)
        resid = ev['resid'][:, sel].sum(axis=0) / np.maximum(n_hit, 1)
        score = n_hit * 1000.0 - resid
        peaks = exclusive_peaks(X, ev['det'][:, sel], score, self.grid.resolution, self.min_views)
        return peaks_to_targets(peaks, X, obs, cams, self.grid.resolution, timestamp,
                                DetectionMethod.SPACE_CARVING)


def peaks_to_targets(peaks, X, obs, cams, res, timestamp, method) -> List[DetectedTarget]:
    out = []
    for pos, members, views in peaks:
        spread = (np.var(X[members], axis=0) if len(members) > 1 else np.zeros(3)) + res ** 2 / 12
        conf = float(np.mean([obs[k].confidence_scores[j] for k, j in views]))
        out.append(DetectedTarget(
            position=pos, velocity=np.zeros(3), confidence=conf,
            volume_estimate=float(len(members) * res ** 3),
            contributing_sensors=[cams[k].camera_id for k, _ in views],
            detection_method=method, covariance_matrix=np.diag(spread), timestamp=timestamp))
    return out
