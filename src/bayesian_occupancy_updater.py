"""
Bayesian Occupancy Grid

Per-voxel log-odds occupancy updated from pixel detections only.

For each camera and each voxel in that camera's field of view:
  - a detection inside the voxel's projected footprint is evidence the voxel is
    occupied:            L += l_hit * confidence_scale
  - no detection there is evidence it is free:   L += l_miss
Voxels outside a camera's view get no update from that camera. Before each
update the grid relaxes toward the prior (L *= decay), so evidence from earlier
frames persists briefly while drones move on.

Unlike space carving this is a soft test: one missed detection lowers the score
instead of deleting the voxel, so a drone seen by 3 cameras and missed by 1 can
still be reported.

A passive camera gives a direction, not a distance, so the evidence for a single
detection is spread along its whole ray; drones appear where rays from several
cameras agree.
"""

from dataclasses import dataclass, field
from typing import List, Tuple
import numpy as np

from camera_geometry import CameraModel, candidate_voxels, view_evidence, exclusive_peaks
from detection_types import DetectedTarget, DetectionMethod


@dataclass
class VoxelGrid:
    """3D voxel grid. log_odds is stored relative to the prior (0 = prior)."""
    origin: np.ndarray
    resolution: float
    dimensions: Tuple[int, int, int]
    log_odds: np.ndarray = None

    def __post_init__(self):
        self.origin = np.asarray(self.origin, dtype=float)
        if self.log_odds is None:
            self.log_odds = np.zeros(self.dimensions, dtype=np.float32)

    def centers(self, flat_idx: np.ndarray) -> np.ndarray:
        ijk = np.column_stack(np.unravel_index(flat_idx, self.dimensions))
        return self.origin + (ijk + 0.5) * self.resolution


class BayesianOccupancyUpdater:
    def __init__(self, grid: VoxelGrid, l_hit: float = 1.6, l_miss: float = -2.0,
                 decay: float = 0.3, occupied_threshold: float = 2.5,
                 min_cameras: int = 2, clamp: float = 10.0):
        self.grid = grid
        self.l_hit = l_hit
        self.l_miss = l_miss
        self.decay = decay
        self.occupied_threshold = occupied_threshold
        self.min_cameras = min_cameras
        self.clamp = clamp

    def update_grid(self, observations: List, timestamp: float):
        """Apply one frame of evidence. Returns (obs, cams, voxel indices, evidence) for extraction."""
        L = self.grid.log_odds
        L *= self.decay
        obs = [o for o in observations if o.camera is not None]
        if not obs:
            return obs, None, np.empty(0, dtype=np.int64), None
        cams = [CameraModel(o.camera) for o in obs]
        res = self.grid.resolution
        dims = tuple(self.grid.dimensions)
        cand = candidate_voxels(obs, cams, self.grid.origin, res, dims, self.min_cameras)
        # voxels still carrying positive evidence from earlier frames also need
        # this frame's (likely negative) evidence
        prev = np.flatnonzero(L.ravel() > 0.5)
        idx = np.union1d(cand, prev)
        if len(idx) == 0:
            return obs, cams, idx, None
        X = self.grid.centers(idx)
        ev = view_evidence(obs, cams, X, res)
        hit, in_view, conf = ev['hit'], ev['in_view'], ev['conf']
        scale = 0.5 + conf  # confidence in [0.1, 0.95] -> scale in [0.6, 1.45]
        delta = (np.where(hit, self.l_hit * scale, 0.0).sum(axis=0)
                 + np.where(in_view & ~hit, self.l_miss, 0.0).sum(axis=0))
        flat = L.ravel()
        flat[idx] = np.clip(flat[idx] + delta, -self.clamp, self.clamp)
        return obs, cams, idx, ev

    def occupied_mask(self) -> np.ndarray:
        return self.grid.log_odds > self.occupied_threshold

    def probability(self) -> np.ndarray:
        """Posterior occupancy relative to the prior (log_odds 0 -> 0.5)."""
        return 1.0 / (1.0 + np.exp(-self.grid.log_odds))

    def detect(self, observations: List, timestamp: float) -> List[DetectedTarget]:
        from space_carving import peaks_to_targets
        obs, cams, idx, ev = self.update_grid(observations, timestamp)
        if ev is None:
            return []
        Lsel = self.grid.log_odds.ravel()[idx]
        sel = np.flatnonzero(Lsel > self.occupied_threshold)
        if len(sel) == 0:
            return []
        X = self.grid.centers(idx[sel])
        peaks = exclusive_peaks(X, ev['det'][:, sel], Lsel[sel].astype(float),
                                self.grid.resolution, self.min_cameras)
        return peaks_to_targets(peaks, X, obs, cams, self.grid.resolution, timestamp,
                                DetectionMethod.OCCUPANCY_GRID)
