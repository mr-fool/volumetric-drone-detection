"""
Bayesian Occupancy Grid Update Module

Implements Bayesian inference algorithms for 3D occupancy grid updates
in volumetric motion detection systems.

Author: Research implementation for academic publication
"""

import numpy as np
from typing import List, Tuple, Set
from dataclasses import dataclass


@dataclass
class VoxelGrid:
    """3D voxel grid for volumetric representation"""
    origin: np.ndarray  # [x, y, z] origin coordinates
    resolution: float   # meters per voxel
    dimensions: Tuple[int, int, int]  # [nx, ny, nz] voxel count
    occupancy_probs: np.ndarray = None  # Bayesian occupancy probabilities
    confidence_scores: np.ndarray = None  # Detection confidence per voxel
    last_update_time: np.ndarray = None  # Temporal decay tracking
    
    def __post_init__(self):
        if self.occupancy_probs is None:
            self.occupancy_probs = np.full(self.dimensions, 0.5, dtype=np.float32)
        if self.confidence_scores is None:
            self.confidence_scores = np.zeros(self.dimensions, dtype=np.float32)
        if self.last_update_time is None:
            self.last_update_time = np.zeros(self.dimensions, dtype=np.float32)


class BayesianOccupancyUpdater:
    """Fixed Bayesian occupancy grid update algorithms"""
    
    def __init__(self, grid: VoxelGrid):
        self.grid = grid
        
    def update_grid(self, observations: List, timestamp: float):
        """Update occupancy probabilities using Bayesian inference"""
        updated_voxels = set()
        
        for obs in observations:
            if len(obs.detected_objects) == 0:
                continue
                
            sensor_pos = self._get_sensor_position(obs.camera_id, obs)
            self._update_from_sensor_observation(obs, sensor_pos, timestamp, updated_voxels)
        
        self._apply_targeted_temporal_decay(updated_voxels, timestamp)
    
    def _get_sensor_position(self, camera_id: str, observation) -> np.ndarray:
        """Extract actual sensor position from camera ID"""
        try:
            cam_num = int(camera_id.split('_')[-1])
            center_x = (self.grid.origin[0] + self.grid.dimensions[0] * self.grid.resolution) / 2
            center_y = (self.grid.origin[1] + self.grid.dimensions[1] * self.grid.resolution) / 2
            radius = min(self.grid.dimensions[0], self.grid.dimensions[1]) * self.grid.resolution * 0.6
            angle = 2 * np.pi * cam_num / 8
            cam_x = center_x + radius * np.cos(angle)
            cam_y = center_y + radius * np.sin(angle)
            cam_z = 100.0
            return np.array([cam_x, cam_y, cam_z])
        except (ValueError, IndexError):
            return np.array([500, 500, 100])

    def _update_from_sensor_observation(self, observation, sensor_pos: np.ndarray, 
                                       timestamp: float, updated_voxels: Set):
        """Enhanced sensor observation processing"""
        for i, obj in enumerate(observation.detected_objects):
            if i >= len(observation.range_estimates):
                continue
                
            target_pos = obj['world_position']
            confidence = min(0.95, max(0.2, 
                observation.confidence_scores[i] if i < len(observation.confidence_scores) else 0.7))
            
            self._update_ray_voxels(sensor_pos, target_pos, confidence, timestamp, updated_voxels)

    def _update_ray_voxels(self, sensor_pos: np.ndarray, target_pos: np.ndarray, 
                          confidence: float, timestamp: float, updated_voxels: Set):
        """Improved ray casting with dense sampling"""
        ray_vec = target_pos - sensor_pos
        ray_length = np.linalg.norm(ray_vec)
        if ray_length < 1e-6:
            return
            
        ray_dir = ray_vec / ray_length
        step_size = min(self.grid.resolution * 0.25, 1.0)
        num_samples = int(ray_length / step_size) + 1
        
        for i in range(num_samples):
            t = i / max(1, num_samples - 1)
            point = sensor_pos + ray_dir * (t * ray_length)
            voxel_coord = self._world_to_voxel_coord(point)
            
            if self._is_valid_voxel(voxel_coord):
                distance_to_target = np.linalg.norm(point - target_pos)
                
                if distance_to_target > 2.0:
                    prob_occupied = 0.05 * (1.0 - confidence)
                elif distance_to_target <= 1.0:
                    prob_occupied = 0.85 * confidence
                else:
                    prob_occupied = 0.3 * confidence
                
                update_weight = min(0.3, confidence)
                self._bayesian_update_voxel(voxel_coord, prob_occupied, update_weight, timestamp)
                updated_voxels.add(voxel_coord)

    def _bayesian_update_voxel(self, voxel_coord: Tuple[int, int, int], 
                              measurement_prob: float, weight: float, timestamp: float):
        """Weighted Bayesian update with bounds checking"""
        x, y, z = voxel_coord
        prior = self.grid.occupancy_probs[x, y, z]
        
        numerator = measurement_prob * prior
        denominator = numerator + (1.0 - measurement_prob) * (1.0 - prior)
        
        if denominator > 0:
            posterior = numerator / denominator
            updated_prob = prior + (posterior - prior) * weight
            self.grid.occupancy_probs[x, y, z] = np.clip(updated_prob, 0.01, 0.99)
        
        self.grid.confidence_scores[x, y, z] = min(1.0, 
            self.grid.confidence_scores[x, y, z] + weight * 0.1)
        self.grid.last_update_time[x, y, z] = timestamp

    def _apply_targeted_temporal_decay(self, updated_voxels: Set, current_time: float):
        """Selective temporal decay for non-updated voxels"""
        decay_rate = 0.95
        prior = 0.5
        
        all_coords = np.indices(self.grid.dimensions)
        all_voxels = set(zip(all_coords[0].ravel(), all_coords[1].ravel(), all_coords[2].ravel()))
        not_updated = all_voxels - updated_voxels
        
        for x, y, z in not_updated:
            time_diff = current_time - self.grid.last_update_time[x, y, z]
            if time_diff > 0.1:
                decay_factor = decay_rate ** time_diff
                self.grid.occupancy_probs[x, y, z] = (
                    prior + (self.grid.occupancy_probs[x, y, z] - prior) * decay_factor)

    def _world_to_voxel_coord(self, world_pos: np.ndarray) -> Tuple[int, int, int]:
        """Convert with bounds checking"""
        relative_pos = world_pos - self.grid.origin
        voxel_pos = (relative_pos / self.grid.resolution).astype(int)
        x = max(0, min(voxel_pos[0], self.grid.dimensions[0] - 1))
        y = max(0, min(voxel_pos[1], self.grid.dimensions[1] - 1))
        z = max(0, min(voxel_pos[2], self.grid.dimensions[2] - 1))
        return (x, y, z)

    def _is_valid_voxel(self, voxel_coord: Tuple[int, int, int]) -> bool:
        """Check if voxel coordinates are within grid bounds"""
        x, y, z = voxel_coord
        return (0 <= x < self.grid.dimensions[0] and
                0 <= y < self.grid.dimensions[1] and
                0 <= z < self.grid.dimensions[2])
