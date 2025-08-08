"""
Volumetric Motion Detection Pipeline for Multi-Drone Swarm Engagement
Academic implementation for JDMS paper submission

Implements:
- Probabilistic occupancy grids with Bayesian updates
- Space carving algorithms for 3D reconstruction
- Multi-view geometry for target triangulation
- Real-time processing optimized for desktop simulation

Author: Research implementation for academic publication
Performance target: 30-50 simultaneous drones on desktop hardware
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict, Union
from enum import Enum
import time
from collections import defaultdict, deque
import scipy.spatial
from scipy.optimize import least_squares
from scipy.ndimage import gaussian_filter
# Note: cv2 import removed as it's not essential for core functionality

class DetectionMethod(Enum):
    OCCUPANCY_GRID = "occupancy_grid"
    SPACE_CARVING = "space_carving"
    TRIANGULATION = "triangulation"
    HYBRID = "hybrid"

@dataclass
class VoxelGrid:
    """3D voxel grid for volumetric representation"""
    origin: np.ndarray  # [x, y, z] origin coordinates
    resolution: float   # meters per voxel
    dimensions: Tuple[int, int, int]  # [nx, ny, nz] voxel count
    occupancy_probs: np.ndarray = field(default=None)  # Bayesian occupancy probabilities
    confidence_scores: np.ndarray = field(default=None)  # Detection confidence per voxel
    last_update_time: np.ndarray = field(default=None)  # Temporal decay tracking
    
    def __post_init__(self):
        if self.occupancy_probs is None:
            self.occupancy_probs = np.full(self.dimensions, 0.5, dtype=np.float32)
        if self.confidence_scores is None:
            self.confidence_scores = np.zeros(self.dimensions, dtype=np.float32)
        if self.last_update_time is None:
            self.last_update_time = np.zeros(self.dimensions, dtype=np.float32)

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
class PerformanceMetrics:
    """Performance tracking for academic evaluation"""
    processing_time: float
    memory_usage_mb: float
    grid_updates_per_second: float
    triangulation_accuracy: float
    false_positive_rate: float
    detection_latency: float
    computational_complexity: Dict[str, float]

class VolumetricDetectionPipeline:
    """
    Main volumetric detection pipeline implementing multiple detection algorithms
    Optimized for real-time processing on desktop hardware
    """
    
    def __init__(self, simulation_bounds, voxel_resolution: float = 2.0,
                 temporal_decay_rate: float = 0.95, detection_threshold: float = 0.7):
        """
        Initialize volumetric detection pipeline
        
        Args:
            simulation_bounds: SimulationBounds object defining 3D volume
            voxel_resolution: Meters per voxel (smaller = higher resolution)
            temporal_decay_rate: Exponential decay for occupancy probabilities
            detection_threshold: Minimum probability for target detection
        """
        self.bounds = simulation_bounds
        self.voxel_resolution = voxel_resolution
        self.temporal_decay_rate = temporal_decay_rate
        self.detection_threshold = detection_threshold
        
        # Initialize 3D voxel grid
        self.grid = self._initialize_voxel_grid()
        
        # Detection algorithms
        self.occupancy_updater = BayesianOccupancyUpdater(self.grid)
        self.space_carver = SpaceCarvingAlgorithm(self.grid)
        self.triangulator = MultiViewTriangulator()
        
        # Target tracking
        self.target_tracker = TemporalTargetTracker()
        self.detection_history = deque(maxlen=100)  # Keep last 100 detections
        
        # Performance monitoring
        self.performance_monitor = PerformanceMonitor()
        
        # Algorithm parameters
        self.bayesian_params = {
            'prob_occupied_given_detection': 0.85,
            'prob_occupied_given_no_detection': 0.15,
            'prior_probability': 0.5,
            'minimum_evidence': 0.1
        }
        
        self.space_carving_params = {
            'carving_threshold': 0.3,
            'consistency_threshold': 0.6,
            'erosion_rate': 0.1
        }
        
    def _initialize_voxel_grid(self) -> VoxelGrid:
        """Initialize 3D voxel grid covering simulation volume"""
        
        # Calculate grid dimensions
        x_range = self.bounds.x_max - self.bounds.x_min
        y_range = self.bounds.y_max - self.bounds.y_min
        z_range = self.bounds.z_max - self.bounds.z_min
        
        nx = int(np.ceil(x_range / self.voxel_resolution))
        ny = int(np.ceil(y_range / self.voxel_resolution))
        nz = int(np.ceil(z_range / self.voxel_resolution))
        
        origin = np.array([self.bounds.x_min, self.bounds.y_min, self.bounds.z_min])
        
        return VoxelGrid(
            origin=origin,
            resolution=self.voxel_resolution,
            dimensions=(nx, ny, nz)
        )
    
    def process_sensor_observations(self, observations: List, timestamp: float,
                                  method: DetectionMethod = DetectionMethod.HYBRID) -> List[DetectedTarget]:
        """
        Main processing pipeline for sensor observations
        
        Args:
            observations: List of SensorObservation objects from sensor array
            timestamp: Current simulation timestamp
            method: Detection method to use
            
        Returns:
            List of detected targets with 3D positions and metadata
        """
        start_time = time.time()
        
        # Apply temporal decay to existing occupancy probabilities
        self._apply_temporal_decay(timestamp)
        
        # Process observations based on selected method
        if method == DetectionMethod.OCCUPANCY_GRID:
            targets = self._process_occupancy_grid_method(observations, timestamp)
        elif method == DetectionMethod.SPACE_CARVING:
            targets = self._process_space_carving_method(observations, timestamp)
        elif method == DetectionMethod.TRIANGULATION:
            targets = self._process_triangulation_method(observations, timestamp)
        elif method == DetectionMethod.HYBRID:
            targets = self._process_hybrid_method(observations, timestamp)
        else:
            raise ValueError(f"Unknown detection method: {method}")
        
        # Update target tracking
        tracked_targets = self.target_tracker.update_tracks(targets, timestamp)
        
        # Record performance metrics
        processing_time = time.time() - start_time
        self.performance_monitor.record_frame(processing_time, len(targets), 
                                            len(observations), self.grid)
        
        # Store detection history
        self.detection_history.append({
            'timestamp': timestamp,
            'targets': tracked_targets,
            'method': method,
            'processing_time': processing_time
        })
        
        return tracked_targets
    
    def _apply_temporal_decay(self, current_time: float):
        """Apply exponential temporal decay to occupancy probabilities"""
        
        # Calculate time since last update for each voxel
        time_diff = current_time - self.grid.last_update_time
        
        # Apply exponential decay where not recently updated
        decay_mask = time_diff > 0.1  # Only decay if > 0.1 seconds old
        decay_factor = np.power(self.temporal_decay_rate, time_diff)
        
        # Decay toward prior probability (0.5)
        prior = self.bayesian_params['prior_probability']
        self.grid.occupancy_probs[decay_mask] = (
            prior + (self.grid.occupancy_probs[decay_mask] - prior) * decay_factor[decay_mask]
        )
        
        # Decay confidence scores
        self.grid.confidence_scores *= np.power(0.98, time_diff)
    
    def _process_occupancy_grid_method(self, observations: List, timestamp: float) -> List[DetectedTarget]:
        """Process using Bayesian occupancy grid updates"""
        
        # Update occupancy grid with new observations
        self.occupancy_updater.update_grid(observations, timestamp)
        
        # Extract high-probability regions as targets
        targets = self._extract_targets_from_occupancy_grid(timestamp)
        
        return hybrid_targets
    
    def _process_hybrid_method(self, observations: List, timestamp: float) -> List[DetectedTarget]:
        """Process using hybrid approach combining multiple methods"""
        
        # Run triangulation for immediate high-confidence detections
        triangulation_targets = self._process_triangulation_method(observations, timestamp)
        
        # Update occupancy grid for spatial consistency
        self.occupancy_updater.update_grid(observations, timestamp)
        
        # Perform space carving for volume estimation
        carved_volume = self.space_carver.carve_space(observations, timestamp)
        
        # Combine results intelligently
        hybrid_targets = self._fuse_detection_results(
            triangulation_targets, carved_volume, timestamp
        )
        
        return hybrid_targets
    
    def _process_space_carving_method(self, observations: List, timestamp: float) -> List[DetectedTarget]:
        """Process using space carving algorithm"""
        
        # Perform space carving based on visual hulls
        carved_volume = self.space_carver.carve_space(observations, timestamp)
        
        # Extract targets from carved volume
        targets = self._extract_targets_from_carved_space(carved_volume, timestamp)
        
        return targets
    
    def _process_triangulation_method(self, observations: List, timestamp: float) -> List[DetectedTarget]:
        """Process using multi-view triangulation"""
        
        # Group observations by detected objects
        object_groups = self._group_observations_by_object(observations)
        
        # Triangulate 3D positions
        targets = []
        for group in object_groups:
            if len(group) >= 2:  # Need at least 2 views for triangulation
                target = self.triangulator.triangulate_target(group, timestamp)
                if target is not None:
                    targets.append(target)
        
        return targets
    
    def _group_observations_by_object(self, observations: List) -> List[List]:
        """Group observations by detected objects for triangulation"""
        
        # Simple grouping - assumes each sensor detects same objects
        # More sophisticated association could be implemented
        max_objects = 0
        for obs in observations:
            max_objects = max(max_objects, len(obs.detected_objects))
        
        object_groups = [[] for _ in range(max_objects)]
        
        for obs in observations:
            for i, obj in enumerate(obs.detected_objects):
                if i < len(object_groups):
                    object_groups[i].append(obs)
        
        return [group for group in object_groups if len(group) >= 2]
    
    def _fuse_detection_results(self, triangulation_targets: List[DetectedTarget], 
                               carved_volume: np.ndarray, timestamp: float) -> List[DetectedTarget]:
        """Fuse results from multiple detection methods"""
        
        # Start with triangulation targets (high confidence)
        fused_targets = triangulation_targets.copy()
        
        # Add targets from occupancy grid that don't overlap with triangulation
        occupancy_targets = self._extract_targets_from_occupancy_grid(timestamp)
        
        for occ_target in occupancy_targets:
            # Check if this target overlaps with existing triangulation targets
            is_duplicate = False
            for tri_target in triangulation_targets:
                distance = np.linalg.norm(occ_target.position - tri_target.position)
                if distance < 10.0:  # 10m overlap threshold
                    is_duplicate = True
                    break
            
            if not is_duplicate:
                # Adjust confidence based on carved volume support
                voxel_coord = self._world_to_voxel_coordinates(occ_target.position.reshape(1, -1))[0]
                if (0 <= voxel_coord[0] < carved_volume.shape[0] and
                    0 <= voxel_coord[1] < carved_volume.shape[1] and
                    0 <= voxel_coord[2] < carved_volume.shape[2]):
                    
                    if carved_volume[voxel_coord[0], voxel_coord[1], voxel_coord[2]]:
                        occ_target.confidence *= 1.2  # Boost confidence if supported by carving
                    else:
                        occ_target.confidence *= 0.8  # Reduce confidence if not supported
                
                fused_targets.append(occ_target)
        
        return fused_targets
    
    def _extract_targets_from_carved_space(self, carved_volume: np.ndarray, 
                                          timestamp: float) -> List[DetectedTarget]:
        """Extract targets from space carving results"""
        
        # Find connected components in carved volume
        carved_coords = np.where(carved_volume)
        
        if len(carved_coords[0]) == 0:
            return []
        
        # Convert to world coordinates
        world_positions = self._voxel_to_world_coordinates(carved_coords)
        
        # Create a mask for clustering
        mask = np.ones(len(world_positions), dtype=bool)
        
        # Cluster positions into discrete targets
        targets = self._cluster_detections(world_positions, mask, timestamp)
        
        # Update detection method for these targets
        for target in targets:
            target.detection_method = DetectionMethod.SPACE_CARVING
        
        return targets
    
    def _extract_targets_from_occupancy_grid(self, timestamp: float) -> List[DetectedTarget]:
        """Extract target detections from high-probability grid regions"""
        
        # Find high-probability voxels
        high_prob_mask = self.grid.occupancy_probs > self.detection_threshold
        high_conf_mask = self.grid.confidence_scores > 0.5
        detection_mask = high_prob_mask & high_conf_mask
        
        if not np.any(detection_mask):
            return []
        
        # Get coordinates of detected voxels
        detected_coords = np.where(detection_mask)
        detected_positions = self._voxel_to_world_coordinates(detected_coords)
        
        # Cluster nearby detections
        targets = self._cluster_detections(detected_positions, detection_mask, timestamp)
        
        return targets
    
    def _cluster_detections(self, positions: np.ndarray, mask: np.ndarray, 
                          timestamp: float) -> List[DetectedTarget]:
        """Cluster nearby detection points into discrete targets"""
        
        if len(positions) == 0:
            return []
        
        targets = []
        clustering_distance = self.voxel_resolution * 3  # Cluster within 3 voxels
        
        # Use hierarchical clustering for target separation
        from scipy.cluster.hierarchy import fcluster, linkage
        
        if len(positions) > 1:
            linkage_matrix = linkage(positions, method='ward')
            cluster_labels = fcluster(linkage_matrix, clustering_distance, criterion='distance')
        else:
            cluster_labels = [1]
        
        # Create target for each cluster
        for cluster_id in np.unique(cluster_labels):
            cluster_mask = cluster_labels == cluster_id
            cluster_positions = positions[cluster_mask]
            
            # Calculate cluster centroid and properties
            centroid = np.mean(cluster_positions, axis=0)
            
            # Estimate volume and confidence
            volume_estimate = len(cluster_positions) * (self.voxel_resolution ** 3)
            
            # Get voxel indices for this cluster
            voxel_coords = self._world_to_voxel_coordinates(cluster_positions)
            confidences = self.grid.confidence_scores[voxel_coords[:, 0], 
                                                    voxel_coords[:, 1], 
                                                    voxel_coords[:, 2]]
            avg_confidence = np.mean(confidences)
            
            # Estimate velocity from recent position changes
            velocity = self._estimate_velocity_from_history(centroid, timestamp)
            
            # Create covariance matrix (simplified)
            position_std = np.std(cluster_positions, axis=0)
            covariance = np.diag(position_std ** 2)
            
            target = DetectedTarget(
                position=centroid,
                velocity=velocity,
                confidence=avg_confidence,
                volume_estimate=volume_estimate,
                contributing_sensors=[],  # Will be filled by tracking
                detection_method=DetectionMethod.OCCUPANCY_GRID,
                covariance_matrix=covariance,
                timestamp=timestamp
            )
            
            targets.append(target)
        
        return targets
    
    def _voxel_to_world_coordinates(self, voxel_coords: Tuple) -> np.ndarray:
        """Convert voxel grid coordinates to world coordinates"""
        
        voxel_array = np.column_stack(voxel_coords)
        world_coords = (self.grid.origin + 
                       voxel_array * self.voxel_resolution + 
                       self.voxel_resolution / 2)  # Center of voxel
        
        return world_coords
    
    def _world_to_voxel_coordinates(self, world_coords: np.ndarray) -> np.ndarray:
        """Convert world coordinates to voxel grid indices"""
        
        relative_coords = world_coords - self.grid.origin
        voxel_coords = (relative_coords / self.voxel_resolution).astype(int)
        
        # Clip to grid bounds
        voxel_coords[:, 0] = np.clip(voxel_coords[:, 0], 0, self.grid.dimensions[0] - 1)
        voxel_coords[:, 1] = np.clip(voxel_coords[:, 1], 0, self.grid.dimensions[1] - 1)
        voxel_coords[:, 2] = np.clip(voxel_coords[:, 2], 0, self.grid.dimensions[2] - 1)
        
        return voxel_coords
    
    def _estimate_velocity_from_history(self, position: np.ndarray, timestamp: float) -> np.ndarray:
        """Estimate velocity using recent detection history"""
        
        if len(self.detection_history) < 2:
            return np.zeros(3)
        
        # Find closest historical detection
        min_distance = float('inf')
        best_match = None
        
        for hist_entry in reversed(list(self.detection_history)[-5:]):  # Check last 5 frames
            time_diff = timestamp - hist_entry['timestamp']
            if time_diff <= 0 or time_diff > 2.0:  # Skip if too old or same time
                continue
                
            for target in hist_entry['targets']:
                distance = np.linalg.norm(target.position - position)
                if distance < min_distance and distance < 20.0:  # Within 20m
                    min_distance = distance
                    best_match = (target.position, time_diff)
        
        if best_match is not None:
            old_position, time_diff = best_match
            velocity = (position - old_position) / time_diff
            return velocity
        
        return np.zeros(3)
    
    def get_detection_summary(self) -> Dict:
        """Get summary statistics for academic evaluation"""
        
        if not self.detection_history:
            return {}
        
        recent_detections = list(self.detection_history)[-10:]  # Last 10 frames
        
        summary = {
            'total_targets_detected': sum(len(d['targets']) for d in recent_detections),
            'average_processing_time': np.mean([d['processing_time'] for d in recent_detections]),
            'detection_rate': len(recent_detections) / 10.0 if recent_detections else 0,
            'grid_utilization': np.mean(self.grid.occupancy_probs > 0.6),
            'performance_metrics': self.performance_monitor.get_summary()
        }
        
        return summary
    
    def visualize_detection_volume(self, timestamp: float = None, 
                                 slice_elevation: float = None) -> plt.Figure:
        """Visualize current detection state for analysis"""
        
        fig = plt.figure(figsize=(16, 12))
        
        # Plot 1: 3D occupancy grid
        ax1 = fig.add_subplot(2, 3, 1, projection='3d')
        self._plot_occupancy_grid_3d(ax1)
        
        # Plot 2: XY slice at specified elevation
        ax2 = fig.add_subplot(2, 3, 2)
        if slice_elevation is None:
            slice_elevation = (self.bounds.z_min + self.bounds.z_max) / 2
        self._plot_occupancy_slice(ax2, slice_elevation, 'xy')
        
        # Plot 3: XZ slice
        ax3 = fig.add_subplot(2, 3, 3)
        mid_y = (self.bounds.y_min + self.bounds.y_max) / 2
        self._plot_occupancy_slice(ax3, mid_y, 'xz')
        
        # Plot 4: Detection confidence histogram
        ax4 = fig.add_subplot(2, 3, 4)
        self._plot_confidence_histogram(ax4)
        
        # Plot 5: Performance metrics over time
        ax5 = fig.add_subplot(2, 3, 5)
        self._plot_performance_history(ax5)
        
        # Plot 6: Target tracking visualization
        ax6 = fig.add_subplot(2, 3, 6)
        self._plot_target_tracks(ax6)
        
        plt.tight_layout()
        return fig
    
    def _plot_occupancy_grid_3d(self, ax):
        """Plot 3D visualization of occupancy grid"""
        
        # Sample high-probability voxels for visualization
        high_prob_mask = self.grid.occupancy_probs > 0.6
        coords = np.where(high_prob_mask)
        
        if len(coords[0]) > 0:
            world_coords = self._voxel_to_world_coordinates(coords)
            probs = self.grid.occupancy_probs[coords]
            
            # Subsample for performance if too many points
            if len(world_coords) > 1000:
                indices = np.random.choice(len(world_coords), 1000, replace=False)
                world_coords = world_coords[indices]
                probs = probs[indices]
            
            scatter = ax.scatter(world_coords[:, 0], world_coords[:, 1], world_coords[:, 2],
                               c=probs, cmap='hot', alpha=0.6, s=20)
            plt.colorbar(scatter, ax=ax, shrink=0.5)
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title('3D Occupancy Probabilities')
    
    def _plot_occupancy_slice(self, ax, slice_value: float, plane: str):
        """Plot 2D slice of occupancy grid"""
        
        if plane == 'xy':
            z_idx = int((slice_value - self.grid.origin[2]) / self.voxel_resolution)
            z_idx = np.clip(z_idx, 0, self.grid.dimensions[2] - 1)
            slice_data = self.grid.occupancy_probs[:, :, z_idx].T
            extent = [self.bounds.x_min, self.bounds.x_max, 
                     self.bounds.y_min, self.bounds.y_max]
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_title(f'XY Occupancy Slice (Z={slice_value:.1f}m)')
        elif plane == 'xz':
            y_idx = int((slice_value - self.grid.origin[1]) / self.voxel_resolution)
            y_idx = np.clip(y_idx, 0, self.grid.dimensions[1] - 1)
            slice_data = self.grid.occupancy_probs[:, y_idx, :].T
            extent = [self.bounds.x_min, self.bounds.x_max,
                     self.bounds.z_min, self.bounds.z_max]
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Z (m)')
            ax.set_title(f'XZ Occupancy Slice (Y={slice_value:.1f}m)')
        
        im = ax.imshow(slice_data, extent=extent, origin='lower', 
                      cmap='hot', alpha=0.8, aspect='auto')
        plt.colorbar(im, ax=ax)
        
    def _plot_confidence_histogram(self, ax):
        """Plot histogram of detection confidences"""
        
        confidences = self.grid.confidence_scores.flatten()
        confidences = confidences[confidences > 0.01]  # Only non-zero confidences
        
        if len(confidences) > 0:
            ax.hist(confidences, bins=50, alpha=0.7, color='blue')
            ax.set_xlabel('Confidence Score')
            ax.set_ylabel('Voxel Count')
            ax.set_title('Detection Confidence Distribution')
        else:
            ax.text(0.5, 0.5, 'No detections', transform=ax.transAxes, 
                   ha='center', va='center')
    
    def _plot_performance_history(self, ax):
        """Plot performance metrics over time"""
        
        if len(self.detection_history) > 1:
            times = [d['timestamp'] for d in self.detection_history]
            proc_times = [d['processing_time'] * 1000 for d in self.detection_history]  # Convert to ms
            
            ax.plot(times, proc_times, 'b-', linewidth=2)
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Processing Time (ms)')
            ax.set_title('Detection Pipeline Performance')
            ax.grid(True, alpha=0.3)
        
    def _plot_target_tracks(self, ax):
        """Plot recent target tracks"""
        
        # Collect target positions from recent history
        track_data = defaultdict(list)
        
        for hist_entry in self.detection_history:
            for target in hist_entry['targets']:
                if target.target_id is not None:
                    track_data[target.target_id].append({
                        'time': hist_entry['timestamp'],
                        'pos': target.position,
                        'conf': target.confidence
                    })
        
        # Plot tracks
        colors = plt.cm.tab10(np.linspace(0, 1, min(10, len(track_data))))
        
        for i, (track_id, positions) in enumerate(track_data.items()):
            if len(positions) > 1:
                pos_array = np.array([p['pos'][:2] for p in positions])  # XY only
                ax.plot(pos_array[:, 0], pos_array[:, 1], 
                       color=colors[i % len(colors)], linewidth=2, alpha=0.7,
                       label=f'Track {track_id}')
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Target Tracks')
        ax.set_xlim(self.bounds.x_min, self.bounds.x_max)
        ax.set_ylim(self.bounds.y_min, self.bounds.y_max)
        if len(track_data) <= 10:
            ax.legend()


class BayesianOccupancyUpdater:
    """Bayesian occupancy grid update algorithms"""
    
    def __init__(self, grid: VoxelGrid):
        self.grid = grid
        
    def update_grid(self, observations: List, timestamp: float):
        """Update occupancy probabilities using Bayesian inference"""
        
        for obs in observations:
            if len(obs.detected_objects) == 0:
                continue
                
            # Update grid based on sensor observations
            self._update_from_sensor_observation(obs, timestamp)
    
    def _update_from_sensor_observation(self, observation, timestamp: float):
        """Update grid based on single sensor observation"""
        
        # For each detected object, update voxels along sensor ray
        for i, obj in enumerate(observation.detected_objects):
            if i >= len(observation.range_estimates):
                continue
                
            # Get sensor position (simplified - assumes sensor array structure)
            sensor_pos = np.array([0, 0, 100])  # Placeholder
            target_pos = obj['world_position']
            confidence = observation.confidence_scores[i] if i < len(observation.confidence_scores) else 0.5
            
            # Update voxels along ray from sensor to target
            self._update_ray_voxels(sensor_pos, target_pos, confidence, timestamp)
    
    def _update_ray_voxels(self, sensor_pos: np.ndarray, target_pos: np.ndarray, 
                          confidence: float, timestamp: float):
        """Update voxels along sensor ray using inverse sensor model"""
        
        # Calculate ray direction and length
        ray_vec = target_pos - sensor_pos
        ray_length = np.linalg.norm(ray_vec)
        ray_dir = ray_vec / ray_length
        
        # Sample points along ray
        num_samples = int(ray_length / (self.grid.resolution * 0.5))
        sample_distances = np.linspace(0, ray_length, num_samples)
        
        for dist in sample_distances:
            point = sensor_pos + ray_dir * dist
            voxel_coord = self._world_to_voxel_coord(point)
            
            if self._is_valid_voxel(voxel_coord):
                # Inverse sensor model: high probability near target, low along ray
                if dist > ray_length * 0.9:  # Near target
                    prob_occupied = 0.8 * confidence
                else:  # Along ray (free space)
                    prob_occupied = 0.2 * (1 - confidence)
                
                # Bayesian update
                self._bayesian_update_voxel(voxel_coord, prob_occupied, timestamp)
    
    def _bayesian_update_voxel(self, voxel_coord: Tuple[int, int, int], 
                              measurement_prob: float, timestamp: float):
        """Perform Bayesian update on single voxel"""
        
        x, y, z = voxel_coord
        
        # Current belief
        prior = self.grid.occupancy_probs[x, y, z]
        
        # Bayesian update formula
        likelihood_occupied = measurement_prob
        likelihood_free = 1.0 - measurement_prob
        
        # Posterior calculation
        numerator = likelihood_occupied * prior
        denominator = numerator + likelihood_free * (1 - prior)
        
        if denominator > 0:
            posterior = numerator / denominator
            self.grid.occupancy_probs[x, y, z] = posterior
        
        # Update confidence and timestamp
        self.grid.confidence_scores[x, y, z] = min(1.0, 
            self.grid.confidence_scores[x, y, z] + measurement_prob * 0.1)
        self.grid.last_update_time[x, y, z] = timestamp
    
    def _world_to_voxel_coord(self, world_pos: np.ndarray) -> Tuple[int, int, int]:
        """Convert world position to voxel coordinates"""
        relative_pos = world_pos - self.grid.origin
        voxel_pos = (relative_pos / self.grid.resolution).astype(int)
        return tuple(voxel_pos)
    
    def _is_valid_voxel(self, voxel_coord: Tuple[int, int, int]) -> bool:
        """Check if voxel coordinates are within grid bounds"""
        x, y, z = voxel_coord
        return (0 <= x < self.grid.dimensions[0] and
                0 <= y < self.grid.dimensions[1] and
                0 <= z < self.grid.dimensions[2])


class SpaceCarvingAlgorithm:
    """Space carving implementation for 3D reconstruction"""
    
    def __init__(self, grid: VoxelGrid):
        self.grid = grid
        self.carved_volume = np.ones(grid.dimensions, dtype=bool)
    
    def carve_space(self, observations: List, timestamp: float) -> np.ndarray:
        """Perform space carving based on visual hull intersection"""
        
        # Reset carved volume
        current_carving = np.ones(self.grid.dimensions, dtype=bool)
        
        # Carve based on each sensor's field of view
        for obs in observations:
            if len(obs.detected_objects) > 0:
                sensor_carving = self._carve_from_sensor(obs)
                current_carving &= sensor_carving
        
        # Update persistent carved volume
        self.carved_volume = current_carving
        
        return self.carved_volume
    
    def _carve_from_sensor(self, observation) -> np.ndarray:
        """Carve space based on single sensor observation"""
        
        sensor_carving = np.zeros(self.grid.dimensions, dtype=bool)
        
        # For each detected object, mark voxels that could contain the object
        for obj in observation.detected_objects:
            target_pos = obj['world_position']
            
            # Convert to voxel coordinates
            voxel_coord = self._world_to_voxel_coord(target_pos)
            
            if self._is_valid_voxel(voxel_coord):
                # Mark neighborhood around detection as potentially occupied
                self._mark_detection_neighborhood(sensor_carving, voxel_coord)
        
        return sensor_carving
    
    def _mark_detection_neighborhood(self, carving_volume: np.ndarray, 
                                   center_voxel: Tuple[int, int, int]):
        """Mark voxel neighborhood as potentially occupied"""
        
        x, y, z = center_voxel
        radius = 2  # Voxel radius for neighborhood
        
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                for dz in range(-radius, radius + 1):
                    nx, ny, nz = x + dx, y + dy, z + dz
                    if self._is_valid_voxel((nx, ny, nz)):
                        carving_volume[nx, ny, nz] = True
    
    def _world_to_voxel_coord(self, world_pos: np.ndarray) -> Tuple[int, int, int]:
        """Convert world position to voxel coordinates"""
        relative_pos = world_pos - self.grid.origin
        voxel_pos = (relative_pos / self.grid.resolution).astype(int)
        return tuple(voxel_pos)
    
    def _is_valid_voxel(self, voxel_coord: Tuple[int, int, int]) -> bool:
        """Check if voxel coordinates are within grid bounds"""
        x, y, z = voxel_coord
        return (0 <= x < self.grid.dimensions[0] and
                0 <= y < self.grid.dimensions[1] and
                0 <= z < self.grid.dimensions[2])


class MultiViewTriangulator:
    """Multi-view geometry triangulation for precise 3D positioning"""
    
    def __init__(self):
        self.triangulation_cache = {}
        
    def triangulate_target(self, observations: List, timestamp: float) -> Optional[DetectedTarget]:
        """Triangulate 3D position from multiple sensor observations"""
        
        if len(observations) < 2:
            return None
        
        # Extract sensor positions and bearing vectors
        sensor_positions = []
        bearing_vectors = []
        confidences = []
        
        for obs in observations:
            if len(obs.detected_objects) == 0:
                continue
                
            # Get first detected object (simplified)
            obj = obs.detected_objects[0]
            
            # Estimate sensor position and bearing (simplified)
            sensor_pos = self._estimate_sensor_position(obs.camera_id)
            bearing = self._calculate_bearing_vector(sensor_pos, obj['world_position'])
            
            sensor_positions.append(sensor_pos)
            bearing_vectors.append(bearing)
            confidences.append(obs.confidence_scores[0] if len(obs.confidence_scores) > 0 else 0.5)
        
        if len(sensor_positions) < 2:
            return None
        
        # Perform triangulation
        triangulated_pos = self._least_squares_triangulation(sensor_positions, bearing_vectors)
        
        if triangulated_pos is None:
            return None
        
        # Calculate triangulation accuracy metrics
        accuracy_metrics = self._calculate_triangulation_accuracy(
            sensor_positions, bearing_vectors, triangulated_pos
        )
        
        # Estimate uncertainty covariance
        covariance = self._estimate_position_covariance(sensor_positions, bearing_vectors, confidences)
        
        # Create target detection
        target = DetectedTarget(
            position=triangulated_pos,
            velocity=np.zeros(3),  # Will be estimated by tracker
            confidence=np.mean(confidences) * accuracy_metrics['geometric_factor'],
            volume_estimate=1.0,  # Simplified volume estimate
            contributing_sensors=[obs.camera_id for obs in observations],
            detection_method=DetectionMethod.TRIANGULATION,
            covariance_matrix=covariance,
            timestamp=timestamp
        )
        
        return target
    
    def _estimate_sensor_position(self, camera_id: str) -> np.ndarray:
        """Estimate sensor position based on camera ID (simplified)"""
        # This would normally come from camera calibration data
        # For now, use simplified positioning based on ID
        return np.array([500, 500, 100])  # Placeholder
    
    def _calculate_bearing_vector(self, sensor_pos: np.ndarray, target_pos: np.ndarray) -> np.ndarray:
        """Calculate unit bearing vector from sensor to target"""
        bearing = target_pos - sensor_pos
        return bearing / np.linalg.norm(bearing)
    
    def _least_squares_triangulation(self, sensor_positions: List[np.ndarray], 
                                   bearing_vectors: List[np.ndarray]) -> Optional[np.ndarray]:
        """Perform least squares triangulation using multiple rays"""
        
        if len(sensor_positions) != len(bearing_vectors) or len(sensor_positions) < 2:
            return None
        
        # Set up least squares problem
        # Each ray contributes: (P - S) × d = 0, where P is target, S is sensor, d is direction
        
        def residual_function(target_pos):
            residuals = []
            for sensor_pos, bearing in zip(sensor_positions, bearing_vectors):
                # Cross product gives perpendicular distance from ray
                vec_to_target = target_pos - sensor_pos
                cross_prod = np.cross(vec_to_target, bearing)
                residuals.extend(cross_prod)
            return np.array(residuals)
        
        # Initial guess: midpoint of first two sensor positions
        initial_guess = (sensor_positions[0] + sensor_positions[1]) / 2
        
        # Solve least squares problem
        try:
            result = least_squares(residual_function, initial_guess, method='lm')
            if result.success:
                return result.x
        except Exception:
            pass
        
        # Fallback: simple two-ray intersection
        if len(sensor_positions) >= 2:
            return self._two_ray_intersection(sensor_positions[0], bearing_vectors[0],
                                            sensor_positions[1], bearing_vectors[1])
        
        return None
    
    def _two_ray_intersection(self, pos1: np.ndarray, dir1: np.ndarray,
                            pos2: np.ndarray, dir2: np.ndarray) -> Optional[np.ndarray]:
        """Find closest point between two 3D rays"""
        
        # Vector between ray origins
        w0 = pos1 - pos2
        
        # Dot products
        a = np.dot(dir1, dir1)
        b = np.dot(dir1, dir2)
        c = np.dot(dir2, dir2)
        d = np.dot(dir1, w0)
        e = np.dot(dir2, w0)
        
        # Solve for parameters
        denom = a * c - b * b
        if abs(denom) < 1e-10:  # Rays are parallel
            return None
        
        s = (b * e - c * d) / denom
        t = (a * e - b * d) / denom
        
        # Points on each ray
        point1 = pos1 + s * dir1
        point2 = pos2 + t * dir2
        
        # Return midpoint
        return (point1 + point2) / 2
    
    def _calculate_triangulation_accuracy(self, sensor_positions: List[np.ndarray],
                                        bearing_vectors: List[np.ndarray],
                                        target_pos: np.ndarray) -> Dict:
        """Calculate triangulation accuracy metrics"""
        
        # Calculate baseline angles
        angles = []
        for i in range(len(sensor_positions)):
            for j in range(i + 1, len(sensor_positions)):
                # Angle between bearing vectors
                dot_product = np.dot(bearing_vectors[i], bearing_vectors[j])
                angle = np.arccos(np.clip(dot_product, -1, 1))
                angles.append(np.degrees(angle))
        
        # Geometric dilution of precision factor
        if angles:
            min_angle = min(angles)
            # Better triangulation when rays are more perpendicular
            geometric_factor = np.sin(np.radians(min_angle))
        else:
            geometric_factor = 0.5
        
        # Calculate reprojection errors
        reprojection_errors = []
        for sensor_pos, bearing in zip(sensor_positions, bearing_vectors):
            vec_to_target = target_pos - sensor_pos
            distance = np.linalg.norm(vec_to_target)
            predicted_bearing = vec_to_target / distance
            angular_error = np.arccos(np.clip(np.dot(bearing, predicted_bearing), -1, 1))
            reprojection_errors.append(np.degrees(angular_error))
        
        return {
            'geometric_factor': geometric_factor,
            'baseline_angles': angles,
            'reprojection_errors': reprojection_errors,
            'mean_reprojection_error': np.mean(reprojection_errors) if reprojection_errors else 0
        }
    
    def _estimate_position_covariance(self, sensor_positions: List[np.ndarray],
                                    bearing_vectors: List[np.ndarray],
                                    confidences: List[float]) -> np.ndarray:
        """Estimate position uncertainty covariance matrix"""
        
        # Simplified covariance estimation based on geometry
        baseline_distance = 0
        if len(sensor_positions) >= 2:
            baseline_distance = np.linalg.norm(sensor_positions[0] - sensor_positions[1])
        
        # Uncertainty scales with range and inversely with baseline
        avg_range = np.mean([np.linalg.norm(pos) for pos in sensor_positions])
        avg_confidence = np.mean(confidences)
        
        # Base uncertainty
        base_std = max(1.0, avg_range / max(100.0, baseline_distance)) / avg_confidence
        
        # Create diagonal covariance (could be made more sophisticated)
        covariance = np.eye(3) * (base_std ** 2)
        
        return covariance


class TemporalTargetTracker:
    """Temporal tracking for target continuity and velocity estimation"""
    
    def __init__(self, max_track_age: float = 5.0, max_association_distance: float = 20.0):
        self.tracks = {}
        self.next_track_id = 1
        self.max_track_age = max_track_age
        self.max_association_distance = max_association_distance
        
    def update_tracks(self, detections: List[DetectedTarget], timestamp: float) -> List[DetectedTarget]:
        """Update target tracks with new detections"""
        
        # Remove old tracks
        self._prune_old_tracks(timestamp)
        
        # Associate detections with existing tracks
        associations = self._associate_detections_to_tracks(detections, timestamp)
        
        # Update existing tracks and create new ones
        updated_detections = []
        
        for detection, track_id in associations:
            if track_id is not None:
                # Update existing track
                updated_detection = self._update_existing_track(detection, track_id, timestamp)
            else:
                # Create new track
                updated_detection = self._create_new_track(detection, timestamp)
            
            updated_detections.append(updated_detection)
        
        return updated_detections
    
    def _associate_detections_to_tracks(self, detections: List[DetectedTarget], 
                                      timestamp: float) -> List[Tuple[DetectedTarget, Optional[int]]]:
        """Associate detections with existing tracks using nearest neighbor"""
        
        associations = []
        used_tracks = set()
        
        for detection in detections:
            best_track_id = None
            min_distance = float('inf')
            
            for track_id, track_data in self.tracks.items():
                if track_id in used_tracks:
                    continue
                
                # Predict track position at current time
                predicted_pos = self._predict_track_position(track_data, timestamp)
                distance = np.linalg.norm(detection.position - predicted_pos)
                
                if distance < min_distance and distance < self.max_association_distance:
                    min_distance = distance
                    best_track_id = track_id
            
            if best_track_id is not None:
                used_tracks.add(best_track_id)
            
            associations.append((detection, best_track_id))
        
        return associations
    
    def _predict_track_position(self, track_data: Dict, timestamp: float) -> np.ndarray:
        """Predict track position at given timestamp"""
        
        if len(track_data['positions']) < 2:
            return track_data['positions'][-1]
        
        # Simple linear extrapolation
        last_pos = track_data['positions'][-1]
        last_time = track_data['timestamps'][-1]
        
        if len(track_data['positions']) >= 2:
            prev_pos = track_data['positions'][-2]
            prev_time = track_data['timestamps'][-2]
            
            dt_history = last_time - prev_time
            if dt_history > 0:
                velocity = (last_pos - prev_pos) / dt_history
                dt_predict = timestamp - last_time
                predicted_pos = last_pos + velocity * dt_predict
                return predicted_pos
        
        return last_pos
    
    def _update_existing_track(self, detection: DetectedTarget, track_id: int, 
                             timestamp: float) -> DetectedTarget:
        """Update existing track with new detection"""
        
        track_data = self.tracks[track_id]
        
        # Add new position and timestamp
        track_data['positions'].append(detection.position)
        track_data['timestamps'].append(timestamp)
        track_data['confidences'].append(detection.confidence)
        
        # Limit history length
        max_history = 20
        if len(track_data['positions']) > max_history:
            track_data['positions'] = track_data['positions'][-max_history:]
            track_data['timestamps'] = track_data['timestamps'][-max_history:]
            track_data['confidences'] = track_data['confidences'][-max_history:]
        
        # Estimate velocity
        velocity = self._estimate_track_velocity(track_data)
        
        # Update detection with track information
        detection.target_id = track_id
        detection.velocity = velocity
        
        # Smooth position using track history
        smoothed_position = self._smooth_track_position(track_data)
        detection.position = smoothed_position
        
        return detection
    
    def _create_new_track(self, detection: DetectedTarget, timestamp: float) -> DetectedTarget:
        """Create new track for unassociated detection"""
        
        track_id = self.next_track_id
        self.next_track_id += 1
        
        # Initialize track data
        self.tracks[track_id] = {
            'positions': [detection.position],
            'timestamps': [timestamp],
            'confidences': [detection.confidence],
            'created_time': timestamp
        }
        
        # Update detection
        detection.target_id = track_id
        detection.velocity = np.zeros(3)  # No velocity estimate yet
        
        return detection
    
    def _estimate_track_velocity(self, track_data: Dict) -> np.ndarray:
        """Estimate velocity from track history"""
        
        if len(track_data['positions']) < 2:
            return np.zeros(3)
        
        # Use multiple recent positions for robust velocity estimation
        positions = np.array(track_data['positions'][-5:])  # Last 5 positions
        timestamps = np.array(track_data['timestamps'][-5:])
        
        if len(positions) < 2:
            return np.zeros(3)
        
        # Linear regression for velocity estimation
        dt = timestamps - timestamps[0]
        dt = dt.reshape(-1, 1)
        
        velocities = []
        for axis in range(3):
            pos_axis = positions[:, axis]
            if len(pos_axis) > 1 and np.std(dt.flatten()) > 0:
                # Simple finite difference
                vel = (pos_axis[-1] - pos_axis[0]) / (dt[-1] - dt[0])
                velocities.append(vel)
            else:
                velocities.append(0.0)
        
        return np.array(velocities)
    
    def _smooth_track_position(self, track_data: Dict) -> np.ndarray:
        """Apply smoothing to track position"""
        
        positions = np.array(track_data['positions'][-3:])  # Last 3 positions
        confidences = np.array(track_data['confidences'][-3:])
        
        # Weighted average based on confidences
        weights = confidences / np.sum(confidences)
        smoothed_pos = np.average(positions, axis=0, weights=weights)
        
        return smoothed_pos
    
    def _prune_old_tracks(self, current_time: float):
        """Remove tracks that haven't been updated recently"""
        
        tracks_to_remove = []
        
        for track_id, track_data in self.tracks.items():
            last_update = track_data['timestamps'][-1]
            age = current_time - last_update
            
            if age > self.max_track_age:
                tracks_to_remove.append(track_id)
        
        for track_id in tracks_to_remove:
            del self.tracks[track_id]


class PerformanceMonitor:
    """Performance monitoring for academic evaluation"""
    
    def __init__(self):
        self.frame_metrics = []
        self.processing_times = deque(maxlen=100)
        self.memory_usage = deque(maxlen=100)
        
    def record_frame(self, processing_time: float, num_targets: int, 
                    num_observations: int, grid: VoxelGrid):
        """Record performance metrics for single frame"""
        
        # Calculate memory usage
        grid_memory = (grid.occupancy_probs.nbytes + 
                      grid.confidence_scores.nbytes + 
                      grid.last_update_time.nbytes) / (1024 * 1024)  # MB
        
        metrics = {
            'processing_time': processing_time,
            'num_targets': num_targets,
            'num_observations': num_observations,
            'memory_usage_mb': grid_memory,
            'grid_updates_per_second': 1.0 / processing_time if processing_time > 0 else 0,
            'targets_per_second': num_targets / processing_time if processing_time > 0 else 0
        }
        
        self.frame_metrics.append(metrics)
        self.processing_times.append(processing_time)
        self.memory_usage.append(grid_memory)
        
        # Keep only recent metrics
        if len(self.frame_metrics) > 1000:
            self.frame_metrics = self.frame_metrics[-1000:]
    
    def get_summary(self) -> Dict:
        """Get performance summary statistics"""
        
        if not self.frame_metrics:
            return {}
        
        recent_metrics = self.frame_metrics[-50:]  # Last 50 frames
        
        summary = {
            'avg_processing_time': np.mean([m['processing_time'] for m in recent_metrics]),
            'max_processing_time': np.max([m['processing_time'] for m in recent_metrics]),
            'avg_memory_usage_mb': np.mean([m['memory_usage_mb'] for m in recent_metrics]),
            'avg_targets_detected': np.mean([m['num_targets'] for m in recent_metrics]),
            'processing_rate_hz': 1.0 / np.mean([m['processing_time'] for m in recent_metrics]),
            'computational_efficiency': self._calculate_efficiency(recent_metrics)
        }
        
        return summary
    
    def _calculate_efficiency(self, metrics: List[Dict]) -> float:
        """Calculate computational efficiency metric"""
        
        # Efficiency = (targets detected per second) / (processing time)
        total_targets = sum(m['num_targets'] for m in metrics)
        total_time = sum(m['processing_time'] for m in metrics)
        
        if total_time > 0:
            return total_targets / total_time
        return 0.0