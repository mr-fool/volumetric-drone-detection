"""
Virtual Sensor Array Simulation for Multi-Drone Detection
Simulates multi-camera sensor arrays with realistic limitations and noise
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict, Union
from enum import Enum
from concurrent.futures import ThreadPoolExecutor
from multiprocessing import cpu_count
import time

class SensorType(Enum):
    VISIBLE_SPECTRUM = "visible"
    THERMAL_INFRARED = "thermal"
    RADAR = "radar"
    LIDAR = "lidar"

@dataclass
class CameraSpecification:
    """Camera sensor specifications and limitations"""
    sensor_type: SensorType
    resolution: Tuple[int, int]  # (width, height) in pixels
    field_of_view: Tuple[float, float]  # (horizontal, vertical) in degrees
    max_range: float  # Maximum detection range in meters
    min_range: float  # Minimum detection range in meters
    noise_std: float  # Sensor noise standard deviation
    detection_threshold: float  # Minimum object size for detection (meters)
    frame_rate: float  # Frames per second
    angular_resolution: float  # Angular resolution in degrees

@dataclass
class CameraPosition:
    """3D position and orientation of a camera sensor"""
    position: np.ndarray  # [x, y, z] in meters
    orientation: np.ndarray  # [yaw, pitch, roll] in degrees
    camera_id: str
    spec: CameraSpecification

@dataclass
class SensorObservation:
    """Single sensor observation of detected targets"""
    camera_id: str
    timestamp: float
    detected_objects: List[Dict]  # List of detected object data
    image_coordinates: np.ndarray  # 2D pixel coordinates of detections
    confidence_scores: np.ndarray  # Detection confidence [0-1]
    range_estimates: np.ndarray  # Estimated ranges to targets
    sensor_noise_level: float  # Current noise level

class VirtualSensorArray:
    """
    Simulates a multi-camera sensor array for drone swarm detection.
    Includes realistic sensor limitations, noise, and detection capabilities.
    """
    
    def __init__(self, simulation_bounds):
        self.bounds = simulation_bounds
        self.cameras = []
        self.current_time = 0.0
        
        # Environmental conditions
        self.weather_conditions = {
            'visibility': 1.0,  # 1.0 = clear, 0.0 = completely obscured
            'wind_speed': 5.0,  # m/s
            'temperature': 20.0,  # Celsius
            'humidity': 0.6  # 0-1
        }
        
        # Detection history for tracking
        self.detection_history = []
        
    def add_camera(self, position: np.ndarray, orientation: np.ndarray, 
                   spec: CameraSpecification, camera_id: str) -> None:
        """Add a camera to the sensor array"""
        camera = CameraPosition(
            position=np.array(position),
            orientation=np.array(orientation),
            camera_id=camera_id,
            spec=spec
        )
        self.cameras.append(camera)
        
    def setup_perimeter_array(self, num_cameras: int = 8, 
                            sensor_type: SensorType = SensorType.VISIBLE_SPECTRUM) -> None:
        """Set up cameras in a perimeter configuration around the surveillance area"""
        
        # Define camera specifications based on type
        if sensor_type == SensorType.VISIBLE_SPECTRUM:
            spec = CameraSpecification(
                sensor_type=sensor_type,
                resolution=(1920, 1080),
                field_of_view=(60.0, 45.0),  # degrees
                max_range=2000.0,  # meters
                min_range=50.0,
                noise_std=2.0,  # pixels
                detection_threshold=0.3,  # meters
                frame_rate=30.0,
                angular_resolution=0.05  # degrees
            )
        elif sensor_type == SensorType.THERMAL_INFRARED:
            spec = CameraSpecification(
                sensor_type=sensor_type,
                resolution=(640, 480),
                field_of_view=(45.0, 35.0),
                max_range=1500.0,
                min_range=30.0,
                noise_std=1.5,
                detection_threshold=0.4,
                frame_rate=25.0,
                angular_resolution=0.08
            )
        else:
            # Default visible spectrum
            spec = CameraSpecification(
                sensor_type=sensor_type,
                resolution=(1280, 720),
                field_of_view=(50.0, 40.0),
                max_range=1800.0,
                min_range=40.0,
                noise_std=2.5,
                detection_threshold=0.35,
                frame_rate=25.0,
                angular_resolution=0.06
            )
        
        # Position cameras around perimeter
        center_x = (self.bounds.x_max + self.bounds.x_min) / 2
        center_y = (self.bounds.y_max + self.bounds.y_min) / 2
        radius = min(self.bounds.x_max - center_x, self.bounds.y_max - center_y) * 1.2
        
        for i in range(num_cameras):
            angle = 2 * np.pi * i / num_cameras
            
            # Camera position (outside the surveillance area)
            cam_x = center_x + radius * np.cos(angle)
            cam_y = center_y + radius * np.sin(angle)
            cam_z = 100.0  # 100m elevation
            
            # Camera orientation (looking toward center)
            yaw = np.degrees(angle + np.pi)  # Look toward center
            pitch = -15.0  # Slight downward angle
            roll = 0.0
            
            self.add_camera(
                position=[cam_x, cam_y, cam_z],
                orientation=[yaw, pitch, roll],
                spec=spec,
                camera_id=f"{sensor_type.value}_cam_{i:02d}"
            )
    
    def setup_triangulation_array(self, baseline_distance: float = 300.0,
                                sensor_type: SensorType = SensorType.VISIBLE_SPECTRUM) -> None:
        """Set up optimized camera array for triangulation accuracy"""
        
        spec = CameraSpecification(
            sensor_type=sensor_type,
            resolution=(2048, 1536),  # Higher resolution for accuracy
            field_of_view=(65.0, 50.0),
            max_range=2500.0,
            min_range=30.0,
            noise_std=1.0,  # Lower noise for precision
            detection_threshold=0.25,
            frame_rate=30.0,
            angular_resolution=0.03
        )
        
        center_x = (self.bounds.x_max + self.bounds.x_min) / 2
        center_y = (self.bounds.y_max + self.bounds.y_min) / 2
        
        # Optimal camera positions for triangulation
        positions = [
            [center_x - baseline_distance, center_y - baseline_distance, 150],  # SW
            [center_x + baseline_distance, center_y - baseline_distance, 150],  # SE
            [center_x + baseline_distance, center_y + baseline_distance, 150],  # NE
            [center_x - baseline_distance, center_y + baseline_distance, 150],  # NW
            [center_x, center_y - baseline_distance * 1.5, 200],               # S (elevated)
            [center_x, center_y + baseline_distance * 1.5, 200]                # N (elevated)
        ]
        
        for i, pos in enumerate(positions):
            # Calculate orientation toward surveillance center
            dx = center_x - pos[0]
            dy = center_y - pos[1]
            yaw = np.degrees(np.arctan2(dy, dx))
            pitch = -10.0  # Slight downward angle
            
            self.add_camera(
                position=pos,
                orientation=[yaw, pitch, 0.0],
                spec=spec,
                camera_id=f"triangulation_cam_{i:02d}"
            )
    
    def observe_targets(self, drone_positions: np.ndarray, 
                       timestamp: float) -> List[SensorObservation]:
        """
        Generate sensor observations for given drone positions
        
        Args:
            drone_positions: Array of shape (num_drones, 3) with [x, y, z] positions
            timestamp: Current simulation time
            
        Returns:
            List of sensor observations from each camera
        """
        observations = []
        self.current_time = timestamp
        
        for camera in self.cameras:
            observation = self._simulate_camera_observation(camera, drone_positions, timestamp)
            observations.append(observation)
            
        return observations
    
    def observe_targets_parallel(self, drone_positions: np.ndarray, 
                                timestamp: float, max_workers: int = None) -> List[SensorObservation]:
        """
        Parallel version of observe_targets - processes cameras simultaneously

        Args:
            drone_positions: Array of shape (num_drones, 3) with [x, y, z] positions
            timestamp: Current simulation time
            max_workers: Maximum number of worker threads (default: auto-detect)

        Returns:
            List of sensor observations from each camera (same format as observe_targets)
        """
        if max_workers is None:
            max_workers = min(cpu_count(), len(self.cameras), 8)

        self.current_time = timestamp

        def process_camera(camera):
            """Process a single camera - same logic as the original"""
            return self._simulate_camera_observation(camera, drone_positions, timestamp)

        # Process all cameras in parallel
        with ThreadPoolExecutor(max_workers=max_workers) as executor:
            futures = [executor.submit(process_camera, camera) for camera in self.cameras]
            observations = [future.result() for future in futures]

        return observations


    def _simulate_camera_observation(self, camera: CameraPosition, 
                                   drone_positions: np.ndarray,
                                   timestamp: float) -> SensorObservation:
        """Simulate observation from a single camera"""
        
        detected_objects = []
        image_coordinates = []
        confidence_scores = []
        range_estimates = []
        
        for i, drone_pos in enumerate(drone_positions):
            # Check if drone is within sensor range and field of view
            detection_result = self._check_detection_feasibility(camera, drone_pos)
            
            if detection_result['detectable']:
                # Calculate image coordinates
                img_coords = self._world_to_image_coordinates(camera, drone_pos)
                
                # Add sensor noise
                noisy_coords = self._add_sensor_noise(camera, img_coords)
                
                # Calculate range estimate with noise
                true_range = detection_result['range']
                range_noise = np.random.normal(0, true_range * 0.02)  # 2% range error
                estimated_range = true_range + range_noise
                
                # Calculate confidence based on range, size, and conditions
                confidence = self._calculate_detection_confidence(camera, drone_pos, detection_result)
                
                detected_objects.append({
                    'drone_id': i,
                    'world_position': drone_pos,
                    'estimated_size': detection_result['apparent_size'],
                    'detection_quality': detection_result['quality']
                })
                
                image_coordinates.append(noisy_coords)
                confidence_scores.append(confidence)
                range_estimates.append(estimated_range)
        
        # Convert to numpy arrays
        image_coordinates = np.array(image_coordinates) if image_coordinates else np.empty((0, 2))
        confidence_scores = np.array(confidence_scores) if confidence_scores else np.empty(0)
        range_estimates = np.array(range_estimates) if range_estimates else np.empty(0)
        
        # Calculate current sensor noise level based on conditions
        noise_level = self._calculate_environmental_noise_factor(camera)
        
        return SensorObservation(
            camera_id=camera.camera_id,
            timestamp=timestamp,
            detected_objects=detected_objects,
            image_coordinates=image_coordinates,
            confidence_scores=confidence_scores,
            range_estimates=range_estimates,
            sensor_noise_level=noise_level
        )
    
    def _check_detection_feasibility(self, camera: CameraPosition, 
                                   drone_pos: np.ndarray) -> Dict:
        """Check if a drone can be detected by the camera"""
        
        # Calculate relative position
        relative_pos = drone_pos - camera.position
        range_to_target = np.linalg.norm(relative_pos)
        
        # Range check
        if range_to_target < camera.spec.min_range or range_to_target > camera.spec.max_range:
            return {'detectable': False, 'reason': 'out_of_range'}
        
        # Field of view check
        if not self._is_in_field_of_view(camera, drone_pos):
            return {'detectable': False, 'reason': 'outside_fov'}
        
        # Calculate apparent size
        drone_size = 0.8  # Assume 0.8m wingspan
        apparent_size = drone_size / range_to_target
        
        # Size threshold check
        if apparent_size < camera.spec.detection_threshold / range_to_target:
            return {'detectable': False, 'reason': 'too_small'}
        
        # Environmental effects
        visibility_factor = self._calculate_visibility_factor(range_to_target)
        if visibility_factor < 0.3:  # 30% minimum visibility
            return {'detectable': False, 'reason': 'poor_visibility'}
        
        # Detection quality based on multiple factors
        quality = min(1.0, apparent_size * 100) * visibility_factor
        
        return {
            'detectable': True,
            'range': range_to_target,
            'apparent_size': apparent_size,
            'quality': quality,
            'visibility_factor': visibility_factor
        }
    
    def _is_in_field_of_view(self, camera: CameraPosition, target_pos: np.ndarray) -> bool:
        """Check if target is within camera's field of view"""
        
        # Transform to camera coordinate system
        relative_pos = target_pos - camera.position
        
        # Apply camera rotation (simplified - assumes only yaw rotation for now)
        yaw_rad = np.radians(camera.orientation[0])
        cos_yaw, sin_yaw = np.cos(yaw_rad), np.sin(yaw_rad)
        
        # Rotate to camera frame
        cam_x = relative_pos[0] * cos_yaw + relative_pos[1] * sin_yaw
        cam_y = -relative_pos[0] * sin_yaw + relative_pos[1] * cos_yaw
        cam_z = relative_pos[2]
        
        # Check if in front of camera
        if cam_x <= 0:
            return False
        
        # Calculate angles
        horizontal_angle = np.degrees(np.arctan2(cam_y, cam_x))
        vertical_angle = np.degrees(np.arctan2(cam_z, cam_x))
        
        # Check field of view bounds
        h_fov, v_fov = camera.spec.field_of_view
        
        return (abs(horizontal_angle) <= h_fov/2 and 
                abs(vertical_angle) <= v_fov/2)
    
    def _world_to_image_coordinates(self, camera: CameraPosition, 
                                  world_pos: np.ndarray) -> np.ndarray:
        """Convert world coordinates to image pixel coordinates"""
        
        # Simplified projection model
        relative_pos = world_pos - camera.position
        range_to_target = np.linalg.norm(relative_pos)
        
        # Apply camera rotation
        yaw_rad = np.radians(camera.orientation[0])
        cos_yaw, sin_yaw = np.cos(yaw_rad), np.sin(yaw_rad)
        
        cam_x = relative_pos[0] * cos_yaw + relative_pos[1] * sin_yaw
        cam_y = -relative_pos[0] * sin_yaw + relative_pos[1] * cos_yaw
        cam_z = relative_pos[2]
        
        # Project to image plane
        if cam_x > 0:  # In front of camera
            h_angle = np.arctan2(cam_y, cam_x)
            v_angle = np.arctan2(cam_z, cam_x)
            
            # Convert to pixel coordinates
            h_fov_rad = np.radians(camera.spec.field_of_view[0])
            v_fov_rad = np.radians(camera.spec.field_of_view[1])
            
            # Normalize to [-1, 1] then to pixel coordinates
            norm_x = h_angle / (h_fov_rad / 2)
            norm_y = v_angle / (v_fov_rad / 2)
            
            pixel_x = (norm_x + 1) * camera.spec.resolution[0] / 2
            pixel_y = (1 - norm_y) * camera.spec.resolution[1] / 2  # Flip Y axis
            
            return np.array([pixel_x, pixel_y])
        
        return np.array([0, 0])  # Default for behind camera
    
    def _add_sensor_noise(self, camera: CameraPosition, 
                         image_coords: np.ndarray) -> np.ndarray:
        """Add realistic sensor noise to image coordinates"""
        
        # Environmental noise factor
        env_factor = self._calculate_environmental_noise_factor(camera)
        
        # Base sensor noise
        base_noise = camera.spec.noise_std
        
        # Total noise (environmental effects increase noise)
        total_noise = base_noise * (1.0 + env_factor)
        
        # Add Gaussian noise
        noise = np.random.normal(0, total_noise, 2)
        
        return image_coords + noise
    
    def _calculate_detection_confidence(self, camera: CameraPosition,
                                      drone_pos: np.ndarray,
                                      detection_info: Dict) -> float:
        """Calculate detection confidence score [0-1]"""
        
        base_confidence = 0.8  # Base confidence for good conditions
        
        # Range factor (closer is better)
        range_factor = 1.0 - (detection_info['range'] / camera.spec.max_range)
        
        # Size factor (larger apparent size is better)
        size_factor = min(1.0, detection_info['apparent_size'] * 500)
        
        # Environmental factor
        env_factor = detection_info['visibility_factor']
        
        # Sensor quality factor
        quality_factor = detection_info['quality']
        
        # Combine factors
        confidence = (base_confidence * 
                     (0.3 * range_factor + 
                      0.3 * size_factor + 
                      0.2 * env_factor + 
                      0.2 * quality_factor))
        
        # Add some random variation
        confidence += np.random.normal(0, 0.05)
        
        return np.clip(confidence, 0.1, 0.95)  # Keep within reasonable bounds
    
    def _calculate_visibility_factor(self, range_to_target: float) -> float:
        """Calculate visibility factor based on environmental conditions and range"""
        
        base_visibility = self.weather_conditions['visibility']
        
        # Atmospheric attenuation with distance
        attenuation = np.exp(-range_to_target / 5000.0)  # 5km characteristic distance
        
        # Weather effects
        humidity_effect = 1.0 - 0.3 * self.weather_conditions['humidity']
        
        return base_visibility * attenuation * humidity_effect
    
    def _calculate_environmental_noise_factor(self, camera: CameraPosition) -> float:
        """Calculate environmental noise factor"""
        
        base_factor = 0.1
        
        # Weather effects
        visibility_effect = (1.0 - self.weather_conditions['visibility']) * 0.5
        wind_effect = min(0.3, self.weather_conditions['wind_speed'] / 20.0)
        
        # Time of day effect (simplified)
        time_effect = 0.1 * np.sin(self.current_time / 86400 * 2 * np.pi) ** 2
        
        return base_factor + visibility_effect + wind_effect + time_effect
    
    def get_detection_statistics(self, observations: List[SensorObservation]) -> Dict:
        """Calculate detection statistics across all sensors"""
        
        total_detections = sum(len(obs.detected_objects) for obs in observations)
        active_cameras = len([obs for obs in observations if len(obs.detected_objects) > 0])
        
        if total_detections > 0:
            avg_confidence = np.mean([np.mean(obs.confidence_scores) 
                                    for obs in observations 
                                    if len(obs.confidence_scores) > 0])
            avg_range = np.mean([np.mean(obs.range_estimates) 
                               for obs in observations 
                               if len(obs.range_estimates) > 0])
        else:
            avg_confidence = 0.0
            avg_range = 0.0
        
        return {
            'total_detections': total_detections,
            'active_cameras': active_cameras,
            'total_cameras': len(self.cameras),
            'detection_density': total_detections / len(self.cameras) if self.cameras else 0,
            'average_confidence': avg_confidence,
            'average_range': avg_range,
            'timestamp': self.current_time
        }
    
    def visualize_sensor_coverage(self, elevation_slice: float = 200.0) -> plt.Figure:
        """Visualize sensor array coverage at specified elevation"""
        
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
        
        # Plot 1: Sensor positions and coverage areas
        ax1.set_xlim(self.bounds.x_min, self.bounds.x_max)
        ax1.set_ylim(self.bounds.y_min, self.bounds.y_max)
        
        for camera in self.cameras:
            # Plot camera position
            ax1.plot(camera.position[0], camera.position[1], 'ro', markersize=8)
            ax1.annotate(camera.camera_id, 
                        (camera.position[0], camera.position[1]),
                        xytext=(5, 5), textcoords='offset points',
                        fontsize=8)
            
            # Draw field of view cone (simplified 2D projection)
            yaw = np.radians(camera.orientation[0])
            h_fov = np.radians(camera.spec.field_of_view[0])
            
            # FOV boundary rays
            angles = [yaw - h_fov/2, yaw + h_fov/2]
            for angle in angles:
                end_x = camera.position[0] + camera.spec.max_range * np.cos(angle)
                end_y = camera.position[1] + camera.spec.max_range * np.sin(angle)
                ax1.plot([camera.position[0], end_x], 
                        [camera.position[1], end_y], 'b--', alpha=0.3)
        
        ax1.set_xlabel('X (meters)')
        ax1.set_ylabel('Y (meters)')
        ax1.set_title('Sensor Array Layout and Coverage')
        ax1.grid(True, alpha=0.3)
        ax1.set_aspect('equal')
        
        # Plot 2: Detection range vs angle for first camera
        if self.cameras:
            camera = self.cameras[0]
            angles = np.linspace(-90, 90, 181)
            detection_ranges = []
            
            for angle in angles:
                # Test detection at this angle
                test_range = camera.spec.max_range * 0.8
                test_x = camera.position[0] + test_range * np.cos(np.radians(angle))
                test_y = camera.position[1] + test_range * np.sin(np.radians(angle))
                test_pos = np.array([test_x, test_y, elevation_slice])
                
                detection = self._check_detection_feasibility(camera, test_pos)
                if detection['detectable']:
                    detection_ranges.append(detection['range'])
                else:
                    detection_ranges.append(0)
            
            ax2.plot(angles, detection_ranges, 'g-', linewidth=2)
            ax2.set_xlabel('Angle (degrees)')
            ax2.set_ylabel('Detection Range (meters)')
            ax2.set_title(f'Detection Range Profile - {camera.camera_id}')
            ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        # Fix axis label clipping issue
        fig.subplots_adjust(left=0.05, right=0.95, top=0.95, bottom=0.05)
        return fig

def create_standard_sensor_array(simulation_bounds, array_type: str = "perimeter") -> VirtualSensorArray:
    """Create a standard sensor array configuration"""
    
    sensor_array = VirtualSensorArray(simulation_bounds)
    
    if array_type == "perimeter":
        sensor_array.setup_perimeter_array(num_cameras=8, 
                                         sensor_type=SensorType.VISIBLE_SPECTRUM)
    elif array_type == "triangulation":
        sensor_array.setup_triangulation_array(baseline_distance=300.0,
                                             sensor_type=SensorType.VISIBLE_SPECTRUM)
    elif array_type == "mixed":
        # Mixed sensor types
        sensor_array.setup_perimeter_array(num_cameras=6, 
                                         sensor_type=SensorType.VISIBLE_SPECTRUM)
        # Add thermal cameras at key positions
        thermal_array = VirtualSensorArray(simulation_bounds)
        thermal_array.setup_triangulation_array(baseline_distance=250.0,
                                               sensor_type=SensorType.THERMAL_INFRARED)
        # Merge thermal cameras
        for camera in thermal_array.cameras[:4]:  # Take first 4 thermal cameras
            sensor_array.cameras.append(camera)
    
    return sensor_array

# Example usage and testing
if __name__ == "__main__":
    from drone_trajectory_generator import SimulationBounds, DroneSwarmGenerator, FlightPattern
    
    # Create simulation environment
    bounds = SimulationBounds()
    
    # Test sensor array creation
    print("Creating virtual sensor array...")
    sensor_array = create_standard_sensor_array(bounds, "perimeter")
    
    print(f"Created sensor array with {len(sensor_array.cameras)} cameras")
    for cam in sensor_array.cameras:
        print(f"  {cam.camera_id}: {cam.spec.sensor_type.value} at {cam.position}")
    
    # Generate some test drone trajectories
    drone_gen = DroneSwarmGenerator(bounds)
    trajectory_data = drone_gen.generate_swarm_trajectories(
        num_drones=15,
        pattern=FlightPattern.COORDINATED_ATTACK,
        duration=10.0,
        timestep=0.5,
        drone_type='small'
    )
    
    # Test sensor observations
    print("\nTesting sensor observations...")
    trajectories = trajectory_data['trajectories']
    times = trajectory_data['times']
    
    # Sample a few time points
    test_times = [0, len(times)//4, len(times)//2, 3*len(times)//4, len(times)-1]
    
    for t_idx in test_times:
        drone_positions = trajectories[:, t_idx, :]
        timestamp = times[t_idx]
        
        observations = sensor_array.observe_targets(drone_positions, timestamp)
        stats = sensor_array.get_detection_statistics(observations)
        
        print(f"Time {timestamp:5.1f}s: {stats['total_detections']:2d} detections "
              f"from {stats['active_cameras']:2d}/{stats['total_cameras']:2d} cameras, "
              f"avg confidence: {stats['average_confidence']:.2f}")
    
    # Create visualization
    print("\nGenerating sensor coverage visualization...")
    try:
        fig = sensor_array.visualize_sensor_coverage()
        fig.savefig('sensor_coverage_analysis.png', dpi=150, bbox_inches='tight')
        plt.close()
        print("Sensor coverage visualization saved as 'sensor_coverage_analysis.png'")
    except Exception as e:
        print(f"Visualization error: {e}")
    
    print("Sensor simulation system ready!")