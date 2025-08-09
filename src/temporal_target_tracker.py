"""
Temporal Target Tracker Module

Implements temporal tracking for target continuity and velocity estimation
across multiple detection frames. Provides track association, velocity estimation,
and position smoothing for volumetric detection systems.

Author: Research implementation for academic publication
"""

import numpy as np
from typing import List, Optional, Dict, Tuple
from dataclasses import dataclass
from enum import Enum

# Import the DetectedTarget class (assuming it's used across modules)
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

@dataclass
class TrackStatistics:
    """Statistics for a single track"""
    track_id: int
    total_detections: int
    track_duration: float
    average_velocity: np.ndarray
    position_variance: np.ndarray
    confidence_trend: float
    last_update_time: float

class TrackingMethod(Enum):
    """Different tracking algorithms available"""
    NEAREST_NEIGHBOR = "nearest_neighbor"
    KALMAN_FILTER = "kalman_filter"
    PARTICLE_FILTER = "particle_filter"

class TemporalTargetTracker:
    """
    Temporal tracking for target continuity and velocity estimation
    
    This class maintains tracks of detected targets across time, providing:
    - Track association using nearest neighbor or advanced algorithms
    - Velocity estimation from track history
    - Position smoothing and prediction
    - Track lifecycle management
    """
    
    def __init__(self, max_track_age: float = 5.0, max_association_distance: float = 20.0,
                 tracking_method: TrackingMethod = TrackingMethod.NEAREST_NEIGHBOR,
                 smoothing_window: int = 3):
        """
        Initialize the temporal target tracker
        
        Args:
            max_track_age: Maximum time (seconds) before a track is considered stale
            max_association_distance: Maximum distance for associating detections to tracks
            tracking_method: Algorithm to use for tracking (nearest neighbor, Kalman, etc.)
            smoothing_window: Number of recent positions to use for smoothing
        """
        self.tracks = {}
        self.next_track_id = 1
        self.max_track_age = max_track_age
        self.max_association_distance = max_association_distance
        self.tracking_method = tracking_method
        self.smoothing_window = smoothing_window
        
        # Track statistics
        self.track_statistics = {}
        self.total_tracks_created = 0
        self.total_tracks_terminated = 0
        
    def update_tracks(self, detections: List[DetectedTarget], timestamp: float) -> List[DetectedTarget]:
        """
        Update target tracks with new detections
        
        Args:
            detections: List of new target detections
            timestamp: Current timestamp
            
        Returns:
            List of updated detections with track assignments
        """
        # Remove old tracks first
        self._prune_old_tracks(timestamp)
        
        # Associate detections with existing tracks
        associations = self._associate_detections_to_tracks(detections, timestamp)
        
        # Update tracks and create new ones as needed
        updated_detections = []
        for detection, track_id in associations:
            if track_id is not None:
                updated_detection = self._update_existing_track(detection, track_id, timestamp)
            else:
                updated_detection = self._create_new_track(detection, timestamp)
            updated_detections.append(updated_detection)
        
        # Update track statistics
        self._update_track_statistics(timestamp)
        
        return updated_detections
    
    def _associate_detections_to_tracks(self, detections: List[DetectedTarget], 
                                      timestamp: float) -> List[Tuple[DetectedTarget, Optional[int]]]:
        """
        Associate detections with existing tracks using the selected tracking method
        
        Args:
            detections: List of detections to associate
            timestamp: Current timestamp
            
        Returns:
            List of (detection, track_id) pairs. track_id is None for new tracks
        """
        if self.tracking_method == TrackingMethod.NEAREST_NEIGHBOR:
            return self._nearest_neighbor_association(detections, timestamp)
        elif self.tracking_method == TrackingMethod.KALMAN_FILTER:
            return self._kalman_filter_association(detections, timestamp)
        else:
            # Fallback to nearest neighbor
            return self._nearest_neighbor_association(detections, timestamp)
    
    def _nearest_neighbor_association(self, detections: List[DetectedTarget], 
                                    timestamp: float) -> List[Tuple[DetectedTarget, Optional[int]]]:
        """Associate detections using nearest neighbor approach"""
        associations = []
        used_tracks = set()
        
        for detection in detections:
            best_track_id = None
            min_distance = float('inf')
            
            for track_id, track_data in self.tracks.items():
                if track_id in used_tracks:
                    continue
                
                predicted_pos = self._predict_track_position(track_data, timestamp)
                distance = np.linalg.norm(detection.position - predicted_pos)
                
                if distance < min_distance and distance < self.max_association_distance:
                    min_distance = distance
                    best_track_id = track_id
            
            if best_track_id is not None:
                used_tracks.add(best_track_id)
            associations.append((detection, best_track_id))
        
        return associations
    
    def _kalman_filter_association(self, detections: List[DetectedTarget], 
                                 timestamp: float) -> List[Tuple[DetectedTarget, Optional[int]]]:
        """
        Associate detections using Kalman filter predictions (simplified implementation)
        
        This is a placeholder for more advanced Kalman filter-based association.
        For now, it falls back to nearest neighbor with improved distance weighting.
        """
        associations = []
        used_tracks = set()
        
        for detection in detections:
            best_track_id = None
            min_cost = float('inf')
            
            for track_id, track_data in self.tracks.items():
                if track_id in used_tracks:
                    continue
                
                # Predict position and calculate innovation
                predicted_pos = self._predict_track_position(track_data, timestamp)
                position_error = detection.position - predicted_pos
                
                # Simple cost function (could be replaced with Mahalanobis distance)
                position_cost = np.linalg.norm(position_error)
                
                # Add velocity consistency cost
                if len(track_data['positions']) >= 2:
                    predicted_velocity = self._estimate_track_velocity(track_data)
                    velocity_cost = np.linalg.norm(detection.velocity - predicted_velocity)
                    total_cost = position_cost + 0.1 * velocity_cost
                else:
                    total_cost = position_cost
                
                if total_cost < min_cost and position_cost < self.max_association_distance:
                    min_cost = total_cost
                    best_track_id = track_id
            
            if best_track_id is not None:
                used_tracks.add(best_track_id)
            associations.append((detection, best_track_id))
        
        return associations
    
    def _predict_track_position(self, track_data: Dict, timestamp: float) -> np.ndarray:
        """
        Predict track position at given timestamp using linear motion model
        
        Args:
            track_data: Track history data
            timestamp: Time to predict position for
            
        Returns:
            Predicted 3D position
        """
        if len(track_data['positions']) < 2:
            return track_data['positions'][-1]
        
        last_pos = track_data['positions'][-1]
        last_time = track_data['timestamps'][-1]
        
        # Use velocity if we have enough history
        if len(track_data['positions']) >= 2:
            prev_pos = track_data['positions'][-2]
            prev_time = track_data['timestamps'][-2]
            
            if prev_time < last_time:
                velocity = (last_pos - prev_pos) / (last_time - prev_time)
                dt_predict = timestamp - last_time
                predicted_pos = last_pos + velocity * dt_predict
                return predicted_pos
        
        return last_pos
    
    def _update_existing_track(self, detection: DetectedTarget, track_id: int, 
                             timestamp: float) -> DetectedTarget:
        """
        Update existing track with new detection
        
        Args:
            detection: New detection to add to track
            track_id: ID of track to update
            timestamp: Current timestamp
            
        Returns:
            Updated detection with track information
        """
        track_data = self.tracks[track_id]
        
        # Add new detection to track history
        track_data['positions'].append(detection.position)
        track_data['timestamps'].append(timestamp)
        track_data['confidences'].append(detection.confidence)
        
        # Limit history size for performance
        max_history = 20
        if len(track_data['positions']) > max_history:
            track_data['positions'] = track_data['positions'][-max_history:]
            track_data['timestamps'] = track_data['timestamps'][-max_history:]
            track_data['confidences'] = track_data['confidences'][-max_history:]
        
        # Update detection with track information
        detection.target_id = track_id
        detection.velocity = self._estimate_track_velocity(track_data)
        detection.position = self._smooth_track_position(track_data)
        
        # Update track metadata
        track_data['last_update'] = timestamp
        
        return detection
    
    def _create_new_track(self, detection: DetectedTarget, timestamp: float) -> DetectedTarget:
        """
        Create new track for unassociated detection
        
        Args:
            detection: Detection to start new track with
            timestamp: Current timestamp
            
        Returns:
            Detection with new track ID assigned
        """
        track_id = self.next_track_id
        self.next_track_id += 1
        self.total_tracks_created += 1
        
        # Initialize new track
        self.tracks[track_id] = {
            'positions': [detection.position.copy()],
            'timestamps': [timestamp],
            'confidences': [detection.confidence],
            'created_time': timestamp,
            'last_update': timestamp
        }
        
        # Update detection with track information
        detection.target_id = track_id
        detection.velocity = np.zeros(3)  # No velocity for new tracks
        
        return detection
    
    def _estimate_track_velocity(self, track_data: Dict) -> np.ndarray:
        """
        Estimate velocity from track history using multiple data points
        
        Args:
            track_data: Track history data
            
        Returns:
            Estimated 3D velocity vector
        """
        if len(track_data['positions']) < 2:
            return np.zeros(3)
        
        # Use recent positions for velocity estimation
        positions = np.array(track_data['positions'][-5:])
        timestamps = np.array(track_data['timestamps'][-5:])
        
        if len(positions) < 2:
            return np.zeros(3)
        
        # Fit linear regression for each axis to get smooth velocity estimate
        velocities = []
        for axis in range(3):
            pos_axis = positions[:, axis]
            if len(pos_axis) > 1:
                dt = timestamps[-1] - timestamps[0]
                if dt > 0:
                    # Simple linear velocity
                    vel = (pos_axis[-1] - pos_axis[0]) / dt
                    velocities.append(vel)
                else:
                    velocities.append(0.0)
            else:
                velocities.append(0.0)
        
        return np.array(velocities)
    
    def _smooth_track_position(self, track_data: Dict) -> np.ndarray:
        """
        Apply smoothing to track position using weighted average
        
        Args:
            track_data: Track history data
            
        Returns:
            Smoothed 3D position
        """
        # Use recent positions within smoothing window
        window_size = min(self.smoothing_window, len(track_data['positions']))
        positions = np.array(track_data['positions'][-window_size:])
        confidences = np.array(track_data['confidences'][-window_size:])
        
        # Weight by confidence scores
        if np.sum(confidences) > 1e-6:
            weights = confidences / np.sum(confidences)
            smoothed_pos = np.average(positions, axis=0, weights=weights)
        else:
            smoothed_pos = np.mean(positions, axis=0)
        
        return smoothed_pos
    
    def _prune_old_tracks(self, current_time: float):
        """
        Remove tracks that haven't been updated recently
        
        Args:
            current_time: Current timestamp
        """
        tracks_to_remove = []
        
        for track_id, track_data in self.tracks.items():
            last_update = track_data.get('last_update', track_data['timestamps'][-1])
            if current_time - last_update > self.max_track_age:
                tracks_to_remove.append(track_id)
        
        for track_id in tracks_to_remove:
            del self.tracks[track_id]
            if track_id in self.track_statistics:
                del self.track_statistics[track_id]
            self.total_tracks_terminated += 1
    
    def _update_track_statistics(self, current_time: float):
        """Update statistics for all active tracks"""
        for track_id, track_data in self.tracks.items():
            if len(track_data['positions']) >= 2:
                duration = current_time - track_data['created_time']
                positions = np.array(track_data['positions'])
                
                self.track_statistics[track_id] = TrackStatistics(
                    track_id=track_id,
                    total_detections=len(track_data['positions']),
                    track_duration=duration,
                    average_velocity=self._estimate_track_velocity(track_data),
                    position_variance=np.var(positions, axis=0),
                    confidence_trend=np.mean(track_data['confidences'][-3:]) if len(track_data['confidences']) >= 3 else np.mean(track_data['confidences']),
                    last_update_time=track_data.get('last_update', track_data['timestamps'][-1])
                )
    
    def get_track_by_id(self, track_id: int) -> Optional[Dict]:
        """
        Get track data by ID
        
        Args:
            track_id: Track identifier
            
        Returns:
            Track data dictionary or None if not found
        """
        return self.tracks.get(track_id)
    
    def get_active_tracks(self) -> Dict[int, Dict]:
        """
        Get all currently active tracks
        
        Returns:
            Dictionary of track_id -> track_data
        """
        return self.tracks.copy()
    
    def get_track_statistics(self) -> Dict[int, TrackStatistics]:
        """
        Get statistics for all tracks
        
        Returns:
            Dictionary of track_id -> TrackStatistics
        """
        return self.track_statistics.copy()
    
    def get_summary_statistics(self) -> Dict[str, float]:
        """
        Get summary statistics for the tracker
        
        Returns:
            Dictionary of summary statistics
        """
        active_tracks = len(self.tracks)
        if active_tracks == 0:
            return {
                'active_tracks': 0,
                'total_tracks_created': self.total_tracks_created,
                'total_tracks_terminated': self.total_tracks_terminated,
                'average_track_duration': 0.0,
                'average_detections_per_track': 0.0
            }
        
        track_durations = []
        detections_per_track = []
        
        for stats in self.track_statistics.values():
            track_durations.append(stats.track_duration)
            detections_per_track.append(stats.total_detections)
        
        return {
            'active_tracks': active_tracks,
            'total_tracks_created': self.total_tracks_created,
            'total_tracks_terminated': self.total_tracks_terminated,
            'average_track_duration': np.mean(track_durations) if track_durations else 0.0,
            'average_detections_per_track': np.mean(detections_per_track) if detections_per_track else 0.0
        }
    
    def reset_tracker(self):
        """Reset the tracker to initial state"""
        self.tracks.clear()
        self.track_statistics.clear()
        self.next_track_id = 1
        self.total_tracks_created = 0
        self.total_tracks_terminated = 0
    
    def set_tracking_parameters(self, max_track_age: Optional[float] = None,
                              max_association_distance: Optional[float] = None,
                              tracking_method: Optional[TrackingMethod] = None):
        """
        Update tracking parameters
        
        Args:
            max_track_age: New maximum track age
            max_association_distance: New maximum association distance
            tracking_method: New tracking method
        """
        if max_track_age is not None:
            self.max_track_age = max_track_age
        if max_association_distance is not None:
            self.max_association_distance = max_association_distance
        if tracking_method is not None:
            self.tracking_method = tracking_method
