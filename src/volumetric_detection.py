"""
Volumetric Motion Detection Pipeline for Multi-Drone Swarm Engagement

Three detectors plus a fusion mode, all working only from what passive cameras
report (noisy pixel detections + camera calibration):

- TRIANGULATION   cross-view association + least-squares triangulation
                  (multi_view_triangulator.py)
- SPACE_CARVING   hard visual-hull consistency over a voxel grid (space_carving.py)
- OCCUPANCY_GRID  soft Bayesian log-odds evidence over a voxel grid
                  (bayesian_occupancy_updater.py)
- HYBRID          runs all three and fuses their targets

Ground truth (true drone positions) is never available here; use
evaluation.py to score outputs against it.
"""

from concurrent.futures import ThreadPoolExecutor
from multiprocessing import cpu_count
from collections import deque
from typing import List, Dict, Optional
import time
import numpy as np
from scipy.optimize import linear_sum_assignment

from detection_types import DetectionMethod, DetectedTarget  # re-exported for callers
from bayesian_occupancy_updater import BayesianOccupancyUpdater, VoxelGrid
from space_carving import SpaceCarvingAlgorithm
from multi_view_triangulator import MultiViewTriangulator
from temporal_target_tracker import TemporalTargetTracker
from performance_monitor import PerformanceMonitor

__all__ = ['DetectionMethod', 'DetectedTarget', 'VolumetricDetectionPipeline']


class VolumetricDetectionPipeline:
    """
    Args:
        voxel_resolution: voxel edge length (m) for carving and occupancy grid.
        fusion_weights: hybrid confidence weights per method. These are
            unfitted defaults; tune them on held-out scenarios (e.g. maximise
            F1 from evaluation.py) before reporting them as optimised.
    """

    def __init__(self, simulation_bounds, voxel_resolution: float = 5.0,
                 temporal_decay_rate: float = 0.3, detection_threshold: float = 0.5,
                 fusion_weights: Optional[Dict[str, float]] = None,
                 min_carving_views: int = 2):
        self.bounds = simulation_bounds
        self.voxel_resolution = voxel_resolution
        self.detection_threshold = detection_threshold
        self.fusion_weights = fusion_weights or {'triangulation': 0.5,
                                                 'space_carving': 0.25,
                                                 'occupancy_grid': 0.25}
        self.grid = self._initialize_voxel_grid()
        self.triangulator = MultiViewTriangulator(simulation_bounds)
        self.space_carver = SpaceCarvingAlgorithm(self.grid, min_views=min_carving_views)
        self.occupancy_updater = BayesianOccupancyUpdater(self.grid, decay=temporal_decay_rate)
        self.target_tracker = TemporalTargetTracker()
        self.performance_monitor = PerformanceMonitor()
        self.detection_history = deque(maxlen=100)

    # ------------------------------------------------------------------ setup
    def _initialize_voxel_grid(self) -> VoxelGrid:
        b, r = self.bounds, self.voxel_resolution
        dims = (int(np.ceil((b.x_max - b.x_min) / r)),
                int(np.ceil((b.y_max - b.y_min) / r)),
                int(np.ceil((b.z_max - b.z_min) / r)))
        return VoxelGrid(origin=np.array([b.x_min, b.y_min, b.z_min]), resolution=r, dimensions=dims)

    def reset(self):
        """Clear temporal state (occupancy evidence, tracks). Call between independent runs."""
        self.grid.log_odds[...] = 0.0
        self.target_tracker = TemporalTargetTracker()
        self.detection_history.clear()

    # --------------------------------------------------------------- dispatch
    def detect(self, observations: List, timestamp: float, method: DetectionMethod,
               max_workers: int = 3) -> List[DetectedTarget]:
        if any(getattr(o, 'camera', None) is None for o in observations):
            raise ValueError("Observations lack camera calibration (obs.camera); "
                             "generate them with VirtualSensorArray.observe_targets*.")
        if method == DetectionMethod.TRIANGULATION:
            return self.triangulator.detect(observations, timestamp)
        if method == DetectionMethod.SPACE_CARVING:
            return self.space_carver.detect(observations, timestamp)
        if method == DetectionMethod.OCCUPANCY_GRID:
            return self.occupancy_updater.detect(observations, timestamp)
        if method == DetectionMethod.HYBRID:
            return self._hybrid(observations, timestamp, max_workers)
        raise ValueError(f"Unknown detection method: {method}")

    def process_sensor_observations_parallel(self, observations, timestamp, method, max_workers=None):
        """Single-frame detection (hybrid runs its three detectors concurrently)."""
        return self.detect(observations, timestamp, method,
                           max_workers=max_workers or min(cpu_count(), 3))

    def process_sensor_observations(self, observations: List, timestamp: float,
                                    method: DetectionMethod = DetectionMethod.HYBRID) -> List[DetectedTarget]:
        """Detection followed by temporal tracking, with performance logging."""
        start = time.time()
        targets = self.detect(observations, timestamp, method)
        tracked = self.target_tracker.update_tracks(targets, timestamp)
        elapsed = time.time() - start
        self.performance_monitor.record_frame(elapsed, len(targets), len(observations), self.grid)
        self.detection_history.append({'timestamp': timestamp, 'targets': tracked,
                                       'method': method, 'processing_time': elapsed})
        return tracked

    # Backwards-compatible names used by older scripts
    def _parallel_space_carving(self, observations, timestamp, max_workers=None):
        return self.detect(observations, timestamp, DetectionMethod.SPACE_CARVING)

    def _parallel_triangulation(self, observations, timestamp, max_workers=None):
        return self.detect(observations, timestamp, DetectionMethod.TRIANGULATION)

    def _get_all_voxel_coordinates(self):
        return self.grid.centers(np.arange(int(np.prod(self.grid.dimensions))))

    # ----------------------------------------------------------------- hybrid
    def _hybrid(self, observations, timestamp, max_workers=3):
        with ThreadPoolExecutor(max_workers=max(1, max_workers)) as ex:
            f_tri = ex.submit(self.triangulator.detect, observations, timestamp)
            f_sc = ex.submit(self.space_carver.detect, observations, timestamp)
            f_occ = ex.submit(self.occupancy_updater.detect, observations, timestamp)
            tri, sc, occ = f_tri.result(), f_sc.result(), f_occ.result()
        return self._fuse(tri, sc, occ, timestamp)

    def _fuse(self, tri, sc, occ, timestamp) -> List[DetectedTarget]:
        """
        Associate targets across methods (Hungarian matching within a distance
        gate) and combine confidences: C = sum_m w_m * C_m (0 if method missed it).
        Position comes from triangulation when available (most precise),
        otherwise the mean of the voxel-method centroids. A target found only
        by voxel methods is kept only if carving and occupancy both report it.
        """
        gate = max(3 * self.voxel_resolution, 15.0)
        entries = [{'triangulation': t} for t in tri]

        def pos(e):
            ts = list(e.values())
            return e['triangulation'].position if 'triangulation' in e else np.mean([t.position for t in ts], axis=0)

        for name, items in (('space_carving', sc), ('occupancy_grid', occ)):
            if not items:
                continue
            if entries:
                P = np.array([pos(e) for e in entries])
                Q = np.array([t.position for t in items])
                D = np.linalg.norm(P[:, None] - Q[None], axis=2)
                r, c = linear_sum_assignment(D)
                matched = set()
                for i, j in zip(r, c):
                    if D[i, j] <= gate:
                        entries[i][name] = items[j]
                        matched.add(j)
                entries += [{name: items[j]} for j in range(len(items)) if j not in matched]
            else:
                entries = [{name: t} for t in items]

        out = []
        for e in entries:
            if 'triangulation' not in e and not ('space_carving' in e and 'occupancy_grid' in e):
                continue
            conf = sum(self.fusion_weights[m] * t.confidence for m, t in e.items())
            primary = e.get('triangulation') or e.get('space_carving')
            sensors = sorted({s for t in e.values() for s in t.contributing_sensors})
            out.append(DetectedTarget(
                position=pos(e), velocity=np.zeros(3), confidence=float(conf),
                volume_estimate=max(t.volume_estimate for t in e.values()),
                contributing_sensors=sensors, detection_method=DetectionMethod.HYBRID,
                covariance_matrix=primary.covariance_matrix, timestamp=timestamp))
        return out
