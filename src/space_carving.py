"""
Space Carving Algorithm Module

Implements space carving algorithms for 3D reconstruction in volumetric
motion detection systems using visual hull intersection techniques.

Author: Research implementation for academic publication
"""

import numpy as np
from typing import List
from bayesian_occupancy_updater import VoxelGrid


class SpaceCarvingAlgorithm:
    """Space carving implementation for 3D reconstruction"""
    
    def __init__(self, grid: VoxelGrid):
        self.grid = grid
        self.carved_volume = np.ones(grid.dimensions, dtype=bool)
        self.consistency_threshold = 0.6
        
    def carve_space(self, observations: List, timestamp: float) -> np.ndarray:
        """Perform space carving based on visual hull intersection"""
        current_carving = np.zeros(self.grid.dimensions, dtype=bool)
        detected_positions = []
        sensor_positions = []
        
        for obs in observations:
            if len(obs.detected_objects) > 0:
                sensor_pos = self._estimate_sensor_position(obs.camera_id)
                sensor_positions.append(sensor_pos)
                for obj in obs.detected_objects:
                    detected_positions.append(obj['world_position'])
        
        unique_detections = self._deduplicate_detections(detected_positions)
        
        if len(unique_detections) == 0:
            return np.zeros(self.grid.dimensions, dtype=bool)
        
        current_carving = self._mark_detected_regions(unique_detections)
        self.carved_volume = current_carving
        return self.carved_volume
    
    def _deduplicate_detections(self, detected_positions: List) -> List:
        """Remove duplicate detection positions"""
        if not detected_positions:
            return []
        
        unique_detections = []
        detection_threshold = 5.0
        
        for det_pos in detected_positions:
            det_pos = np.array(det_pos)
            is_duplicate = False
            for unique_pos in unique_detections:
                distance = np.linalg.norm(det_pos - np.array(unique_pos))
                if distance < detection_threshold:
                    is_duplicate = True
                    break
            
            if not is_duplicate:
                unique_detections.append(det_pos)
        
        return unique_detections
    
    def _mark_detected_regions(self, detected_positions: List) -> np.ndarray:
        """Mark regions around detections"""
        carved_volume = np.zeros(self.grid.dimensions, dtype=bool)
        
        for det_pos in detected_positions:
            det_pos = np.array(det_pos)
            det_voxel = ((det_pos - self.grid.origin) / self.grid.resolution).astype(int)
            
            if not (0 <= det_voxel[0] < self.grid.dimensions[0] and
                    0 <= det_voxel[1] < self.grid.dimensions[1] and
                    0 <= det_voxel[2] < self.grid.dimensions[2]):
                continue
            
            voxel_radius = int(np.ceil(5.0 / self.grid.resolution))
            
            x_start = max(0, det_voxel[0] - voxel_radius)
            x_end = min(self.grid.dimensions[0], det_voxel[0] + voxel_radius + 1)
            y_start = max(0, det_voxel[1] - voxel_radius)
            y_end = min(self.grid.dimensions[1], det_voxel[1] + voxel_radius + 1)
            z_start = max(0, det_voxel[2] - voxel_radius)
            z_end = min(self.grid.dimensions[2], det_voxel[2] + voxel_radius + 1)
            
            for x in range(x_start, x_end):
                for y in range(y_start, y_end):
                    for z in range(z_start, z_end):
                        world_pos = self.grid.origin + np.array([x, y, z]) * self.grid.resolution
                        distance = np.linalg.norm(world_pos - det_pos)
                        if distance <= 5.0:
                            carved_volume[x, y, z] = True
        
        return carved_volume
    
    def _estimate_sensor_position(self, camera_id: str) -> np.ndarray:
        """Estimate sensor position from camera ID"""
        try:
            cam_num = int(camera_id.split('_')[-1])
        except (ValueError, IndexError):
            cam_num = 0
        
        center_x = (self.grid.origin[0] + self.grid.dimensions[0] * self.grid.resolution) / 2
        center_y = (self.grid.origin[1] + self.grid.dimensions[1] * self.grid.resolution) / 2
        radius = min(self.grid.dimensions[0], self.grid.dimensions[1]) * self.grid.resolution * 0.6
        angle = 2 * np.pi * cam_num / 8
        cam_x = center_x + radius * np.cos(angle)
        cam_y = center_y + radius * np.sin(angle)
        cam_z = 100.0
        
        return np.array([cam_x, cam_y, cam_z])
