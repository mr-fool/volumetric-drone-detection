"""
Debug test for space carving algorithm
Tests the space carving implementation step by step
"""

import sys
import os
import numpy as np

class TeeOutput:
    """Class to output to both console and file simultaneously"""
    def __init__(self, filename):
        self.terminal = sys.stdout
        self.log = open(filename, 'w', encoding='utf-8')
    
    def write(self, message):
        self.terminal.write(message)
        self.terminal.flush()
        self.log.write(message)
        self.log.flush()
    
    def flush(self):
        self.terminal.flush()
        self.log.flush()
    
    def close(self):
        self.log.close()

# Add src directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

try:
    from drone_trajectory_generator import DroneSwarmGenerator, FlightPattern, SimulationBounds
    from sensor_simulation import create_standard_sensor_array
    from volumetric_detection import VolumetricDetectionPipeline, DetectionMethod
    
    def debug_space_carving():
        print("DEBUG: Space Carving Algorithm Test")
        print("=" * 50)
        
        # Setup
        bounds = SimulationBounds()
        drone_generator = DroneSwarmGenerator(bounds)
        sensor_array = create_standard_sensor_array(bounds, "perimeter")
        detection_pipeline = VolumetricDetectionPipeline(bounds, voxel_resolution=2.0)
        
        print(f"Grid dimensions: {detection_pipeline.grid.dimensions}")
        print(f"Grid origin: {detection_pipeline.grid.origin}")
        print(f"Grid resolution: {detection_pipeline.grid.resolution}")
        print(f"Number of sensors: {len(sensor_array.cameras)}")
        
        # Generate simple test scenario
        trajectory_data = drone_generator.generate_swarm_trajectories(
            num_drones=5,
            pattern=FlightPattern.FORMATION_FLYING,  # This works better
            duration=1.0,
            timestep=1.0,
            drone_type='small'
        )
        
        trajectories = trajectory_data['trajectories']
        times = trajectory_data['times']
        
        # Test one frame
        drone_positions = trajectories[:, 0, :]
        timestamp = times[0]
        
        print(f"\nDrone positions:")
        for i, pos in enumerate(drone_positions):
            print(f"  Drone {i}: {pos}")
        
        # Get sensor observations
        observations = sensor_array.observe_targets(drone_positions, timestamp)
        
        print(f"\nSensor observations:")
        total_detections = 0
        for obs in observations:
            print(f"  {obs.camera_id}: {len(obs.detected_objects)} detections")
            total_detections += len(obs.detected_objects)
            
            for j, obj in enumerate(obs.detected_objects):
                print(f"    Detection {j}: {obj['world_position']}")
        
        print(f"Total detections across all sensors: {total_detections}")
        
        if total_detections == 0:
            print("ERROR: No detections from sensors - cannot test space carving")
            return
        
        # Test space carving directly
        print(f"\nTesting space carving algorithm...")
        space_carver = detection_pipeline.space_carver
        
        import time
        carving_start = time.time()
        
        # Test carving
        carved_volume = space_carver.carve_space(observations, timestamp)
        
        carving_time = time.time() - carving_start
        carved_voxels = np.sum(carved_volume)
        
        print(f"Space carving completed in {carving_time:.2f} seconds")
        print(f"Carved volume voxels: {carved_voxels}")
        print(f"Carved volume percentage: {carved_voxels / np.prod(carved_volume.shape) * 100:.2f}%")
        
        # WARNING: Check for over-carving
        if carved_voxels > np.prod(carved_volume.shape) * 0.1:  # More than 10% carved
            print(f"WARNING: Space carving carved {carved_voxels / np.prod(carved_volume.shape) * 100:.1f}% of total volume!")
            print("This indicates over-carving - algorithm may be too permissive")
        elif carved_voxels == 0:
            print("WARNING: No voxels carved - algorithm may be too restrictive")
        else:
            print(f"Space carving looks reasonable ({carved_voxels} voxels)")
        
        if carved_voxels > 0:
            # Find carved regions
            carved_coords = np.where(carved_volume)
            print(f"Carved regions found at {len(carved_coords[0])} voxels")
            
            # Show some carved voxel coordinates
            for i in range(min(5, len(carved_coords[0]))):
                voxel_coord = [carved_coords[0][i], carved_coords[1][i], carved_coords[2][i]]
                world_coord = (detection_pipeline.grid.origin + 
                             np.array(voxel_coord) * detection_pipeline.grid.resolution)
                print(f"  Voxel {voxel_coord} -> World {world_coord}")
        else:
            print("ERROR: No voxels carved - debugging space carving algorithm...")
            
            # Debug sensor positions
            print("\nDEBUG: Sensor positions:")
            for obs in observations:
                if len(obs.detected_objects) > 0:
                    sensor_pos = space_carver._estimate_sensor_position(obs.camera_id)
                    print(f"  {obs.camera_id}: {sensor_pos}")
            
            # Debug detection positions vs sensor positions
            print("\nDEBUG: Detection analysis:")
            for obs in observations:
                if len(obs.detected_objects) > 0:
                    sensor_pos = space_carver._estimate_sensor_position(obs.camera_id)
                    for obj in obs.detected_objects:
                        det_pos = obj['world_position']
                        distance = np.linalg.norm(np.array(det_pos) - np.array(sensor_pos))
                        print(f"  Detection at {det_pos}, sensor at {sensor_pos}, distance: {distance:.1f}m")
        
        # Test space carving with different methods
        print(f"\nTesting all detection methods:")
        
        methods = [DetectionMethod.SPACE_CARVING, DetectionMethod.TRIANGULATION, DetectionMethod.HYBRID]
        
        for method in methods:
            try:
                detected_targets = detection_pipeline.process_sensor_observations(
                    observations, timestamp, method
                )
                print(f"  {method.value}: {len(detected_targets)} targets detected")
                
                for i, target in enumerate(detected_targets):
                    print(f"    Target {i}: pos={target.position}, conf={target.confidence:.3f}, method={target.detection_method.value}")
                    
            except Exception as e:
                print(f"  {method.value}: ERROR - {e}")
        
        print(f"\nSpace carving debug test completed!")
        
    if __name__ == "__main__":
        # Setup dual output to both console and file
        output_filename = 'debug_space_carving_results_final.txt'
        tee_output = TeeOutput(output_filename)
        original_stdout = sys.stdout
        sys.stdout = tee_output
        
        try:
            debug_space_carving()
            print(f"\nDebug results saved to: {output_filename}")
        finally:
            # Restore original stdout and close file
            sys.stdout = original_stdout
            tee_output.close()
        
except ImportError as e:
    print(f"Import error: {e}")
    print("Make sure all files are in the src/ directory")
except Exception as e:
    print(f"Test error: {e}")
    import traceback
    traceback.print_exc()
