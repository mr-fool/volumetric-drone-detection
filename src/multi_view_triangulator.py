"""
Multi-View Triangulator Module

Implements multi-view geometry triangulation for precise 3D positioning
of targets from multiple sensor observations.

Author: Research implementation for academic publication
"""

import numpy as np
from typing import List, Optional, Dict, Tuple
from dataclasses import dataclass
from enum import Enum

# Import the DetectedTarget class (assuming it's moved to a common types module)
# For now, we'll define it locally to maintain compatibility
class DetectionMethod(Enum):
    OCCUPANCY_GRID = "occupancy_grid"
    SPACE_CARVING = "space_carving"
    TRIANGULATION = "triangulation"
    HYBRID = "hybrid"

@dataclass
class DetectedTarget:
    """3D target detection result"""
    position: np.ndarray  # [x, y, z] world coordinates
    velocity: np.ndarray  # [vx, vy, vz] estimated velocity
    confidence: float     # Overall detection confidence [0-1]
    volume_estimate: float  # Estimated target volume (m³)
    contributing_sensors: List[str]  # Sensor IDs that detected this target
    detection_method: DetectionMethod  # Primary detection method used
    covariance_matrix: np.ndarray  # Position uncertainty covariance
    timestamp: float      # Detection timestamp
    target_id: Optional[int] = None  # Tracking ID if available


class MultiViewTriangulator:
    """Multi-view geometry triangulation for precise 3D positioning"""
    
    def __init__(self, simulation_bounds=None):
        """
        Initialize the triangulator
        
        Args:
            simulation_bounds: Simulation bounds object with x_min, x_max, y_min, y_max, z_min, z_max
        """
        self.triangulation_cache = {}
        self.bounds = simulation_bounds
        
    def triangulate_target(self, observations: List, timestamp: float) -> Optional[DetectedTarget]:
        """
        Triangulate 3D position from multiple sensor observations
        
        Args:
            observations: List of sensor observations containing detected objects
            timestamp: Current timestamp
            
        Returns:
            DetectedTarget object if successful triangulation, None otherwise
        """
        if len(observations) < 2:
            return None
        
        sensor_positions = []
        target_positions = []
        confidences = []
        
        # Extract sensor positions and target positions from observations
        for obs in observations:
            if len(obs.detected_objects) > 0:
                sensor_pos = self._estimate_sensor_position(obs.camera_id)
                sensor_positions.append(sensor_pos)
                target_positions.append(obs.detected_objects[0]['world_position'])
                confidences.append(obs.confidence_scores[0] if len(obs.confidence_scores) > 0 else 0.5)
        
        if len(sensor_positions) < 2:
            return None
        
        # Perform triangulation (simplified approach using mean position)
        triangulated_pos = np.mean(target_positions, axis=0)
        position_std = np.std(target_positions, axis=0)
        
        # Calculate accuracy factor based on position variance
        accuracy_factor = 1.0 / (1.0 + np.mean(position_std))
        
        # Create covariance matrix
        covariance = np.diag(position_std ** 2 + 1.0)
        
        # Create the detected target
        target = DetectedTarget(
            position=triangulated_pos,
            velocity=np.zeros(3),  # Velocity will be estimated by tracker
            confidence=np.mean(confidences) * accuracy_factor,
            volume_estimate=1.0,  # Default volume estimate
            contributing_sensors=[obs.camera_id for obs in observations],
            detection_method=DetectionMethod.TRIANGULATION,
            covariance_matrix=covariance,
            timestamp=timestamp
        )
        
        return target
    
    def triangulate_advanced(self, observations: List, timestamp: float) -> Optional[DetectedTarget]:
        """
        Advanced triangulation using least squares optimization
        
        Args:
            observations: List of sensor observations
            timestamp: Current timestamp
            
        Returns:
            DetectedTarget object with improved position estimate
        """
        if len(observations) < 2:
            return None
            
        # Extract observation data
        sensor_positions = []
        observation_rays = []
        confidences = []
        
        for obs in observations:
            if len(obs.detected_objects) > 0:
                sensor_pos = self._estimate_sensor_position(obs.camera_id)
                target_pos = obs.detected_objects[0]['world_position']
                
                # Create observation ray (normalized direction vector)
                ray_direction = target_pos - sensor_pos
                ray_direction = ray_direction / np.linalg.norm(ray_direction)
                
                sensor_positions.append(sensor_pos)
                observation_rays.append(ray_direction)
                confidences.append(obs.confidence_scores[0] if len(obs.confidence_scores) > 0 else 0.5)
        
        if len(sensor_positions) < 2:
            return None
        
        # Use least squares to find best intersection point
        triangulated_pos = self._least_squares_triangulation(sensor_positions, observation_rays)
        
        if triangulated_pos is None:
            # Fallback to simple triangulation
            return self.triangulate_target(observations, timestamp)
        
        # Calculate position uncertainty
        position_errors = []
        for sensor_pos, ray_dir in zip(sensor_positions, observation_rays):
            # Calculate distance from triangulated point to observation ray
            to_point = triangulated_pos - sensor_pos
            projection = np.dot(to_point, ray_dir) * ray_dir
            perpendicular = to_point - projection
            error = np.linalg.norm(perpendicular)
            position_errors.append(error)
        
        # Create covariance matrix based on triangulation errors
        mean_error = np.mean(position_errors)
        covariance = np.eye(3) * (mean_error ** 2 + 0.1)  # Add minimum uncertainty
        
        # Calculate confidence based on triangulation quality
        base_confidence = np.mean(confidences)
        triangulation_quality = 1.0 / (1.0 + mean_error)
        final_confidence = base_confidence * triangulation_quality
        
        target = DetectedTarget(
            position=triangulated_pos,
            velocity=np.zeros(3),
            confidence=final_confidence,
            volume_estimate=1.0,
            contributing_sensors=[obs.camera_id for obs in observations],
            detection_method=DetectionMethod.TRIANGULATION,
            covariance_matrix=covariance,
            timestamp=timestamp
        )
        
        return target
    
    def _least_squares_triangulation(self, sensor_positions: List[np.ndarray], 
                                   observation_rays: List[np.ndarray]) -> Optional[np.ndarray]:
        """
        Perform least squares triangulation to find optimal 3D point
        
        Args:
            sensor_positions: List of sensor position vectors
            observation_rays: List of normalized observation ray directions
            
        Returns:
            Triangulated 3D position or None if optimization fails
        """
        try:
            from scipy.optimize import least_squares
            
            def residual_function(point_3d):
                """Calculate residuals for least squares optimization"""
                residuals = []
                
                for sensor_pos, ray_dir in zip(sensor_positions, observation_rays):
                    # Vector from sensor to estimated point
                    to_point = point_3d - sensor_pos
                    
                    # Project onto observation ray
                    projection_length = np.dot(to_point, ray_dir)
                    projection = projection_length * ray_dir
                    
                    # Calculate perpendicular distance (error)
                    perpendicular = to_point - projection
                    residuals.extend(perpendicular)
                
                return np.array(residuals)
            
            # Initial guess: centroid of all sensor positions
            initial_guess = np.mean(sensor_positions, axis=0)
            
            # Add offset in the general direction of observations
            mean_ray_direction = np.mean(observation_rays, axis=0)
            mean_ray_direction = mean_ray_direction / np.linalg.norm(mean_ray_direction)
            initial_guess += mean_ray_direction * 50.0  # 50m offset
            
            # Perform optimization
            result = least_squares(residual_function, initial_guess, method='lm')
            
            if result.success:
                return result.x
            else:
                return None
                
        except ImportError:
            # Fallback if scipy is not available
            return None
        except Exception:
            # Any other optimization error
            return None
    
    def _estimate_sensor_position(self, camera_id: str) -> np.ndarray:
        """
        Estimate sensor position based on camera ID
        
        Args:
            camera_id: String identifier for the camera/sensor
            
        Returns:
            Estimated 3D position of the sensor
        """
        if self.bounds is None:
            # Default position if no bounds specified
            return np.array([500, 500, 100])
        
        try:
            # Extract camera number from ID (assumes format like "camera_0", "sensor_1", etc.)
            cam_num = int(camera_id.split('_')[-1])
        except (ValueError, IndexError):
            cam_num = 0
        
        # Calculate center of simulation bounds
        center_x = (self.bounds.x_max + self.bounds.x_min) / 2
        center_y = (self.bounds.y_max + self.bounds.y_min) / 2
        
        # Calculate radius for camera placement (20% larger than bounds)
        radius = min(self.bounds.x_max - center_x, self.bounds.y_max - center_y) * 1.2
        
        # Place cameras in a circle around the simulation area
        angle = 2 * np.pi * cam_num / 8  # Assume 8 cameras maximum
        cam_x = center_x + radius * np.cos(angle)
        cam_y = center_y + radius * np.sin(angle)
        cam_z = 100.0  # Fixed height
        
        return np.array([cam_x, cam_y, cam_z])
    
    def validate_triangulation(self, target: DetectedTarget, observations: List) -> Dict[str, float]:
        """
        Validate triangulation quality
        
        Args:
            target: The triangulated target
            observations: Original observations used for triangulation
            
        Returns:
            Dictionary of validation metrics
        """
        validation_metrics = {
            'reprojection_error': 0.0,
            'geometric_consistency': 0.0,
            'observation_agreement': 0.0
        }
        
        if not observations:
            return validation_metrics
        
        # Calculate reprojection errors
        reprojection_errors = []
        for obs in observations:
            if len(obs.detected_objects) > 0:
                observed_pos = obs.detected_objects[0]['world_position']
                error = np.linalg.norm(target.position - observed_pos)
                reprojection_errors.append(error)
        
        if reprojection_errors:
            validation_metrics['reprojection_error'] = np.mean(reprojection_errors)
        
        # Calculate geometric consistency (how well observations agree)
        if len(observations) >= 2:
            all_positions = [obs.detected_objects[0]['world_position'] 
                           for obs in observations if len(obs.detected_objects) > 0]
            if len(all_positions) >= 2:
                position_std = np.std(all_positions, axis=0)
                validation_metrics['geometric_consistency'] = 1.0 / (1.0 + np.mean(position_std))
        
        # Calculate observation agreement
        confidences = [obs.confidence_scores[0] if len(obs.confidence_scores) > 0 else 0.5
                      for obs in observations]
        if confidences:
            validation_metrics['observation_agreement'] = np.mean(confidences)
        
        return validation_metrics
    
    def clear_cache(self):
        """Clear the triangulation cache"""
        self.triangulation_cache.clear()
    
    def get_cache_stats(self) -> Dict[str, int]:
        """Get statistics about the triangulation cache"""
        return {
            'cache_size': len(self.triangulation_cache),
            'cache_limit': 1000  # Could be configurable
        }
