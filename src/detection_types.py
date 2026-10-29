"""Shared result types for all detection methods (one definition, imported everywhere)."""

from dataclasses import dataclass
from enum import Enum
from typing import List, Optional
import numpy as np


class DetectionMethod(Enum):
    OCCUPANCY_GRID = "occupancy_grid"
    SPACE_CARVING = "space_carving"
    TRIANGULATION = "triangulation"
    HYBRID = "hybrid"


@dataclass
class DetectedTarget:
    """3D target detection result"""
    position: np.ndarray            # [x, y, z] world coordinates (estimated)
    velocity: np.ndarray            # [vx, vy, vz] (zeros: single-frame detectors)
    confidence: float               # Detection confidence [0-1]
    volume_estimate: float          # Occupied volume (m^3) for voxel methods, else 0
    contributing_sensors: List[str]  # Camera IDs whose detections support this target
    detection_method: DetectionMethod
    covariance_matrix: np.ndarray   # Position uncertainty (3x3)
    timestamp: float
    target_id: Optional[int] = None
