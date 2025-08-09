"""
Quick validation test for volumetric detection pipeline
Tests core functionality without full validation suite
Now uses parallel processing for better performance
"""

import sys
import os
import time
import numpy as np
# Force more aggressive threading
os.environ['OMP_NUM_THREADS'] = '8'
os.environ['MKL_NUM_THREADS'] = '8'
os.environ['NUMEXPR_NUM_THREADS'] = '8'

# Add src directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

try:
    from drone_trajectory_generator import DroneSwarmGenerator, FlightPattern, SimulationBounds
    from sensor_simulation import create_standard_sensor_array
    from volumetric_detection import VolumetricDetectionPipeline, DetectionMethod
    
    def quick_test():
        print("Quick Volumetric Detection Test")
        print("=" * 40)
        
        # Setup
        bounds = SimulationBounds()
        drone_generator = DroneSwarmGenerator(bounds)
        sensor_array = create_standard_sensor_array(bounds, "perimeter")
        detection_pipeline = VolumetricDetectionPipeline(bounds, voxel_resolution=3.0)
        
        print(f"Setup complete: {len(sensor_array.cameras)} cameras")
        
        # Generate simple test scenario
        trajectory_data = drone_generator.generate_swarm_trajectories(
            num_drones=5,
            pattern=FlightPattern.COORDINATED_ATTACK,
            duration=2.0,
            timestep=0.5,
            drone_type='small'
        )
        
        trajectories = trajectory_data['trajectories']
        times = trajectory_data['times']
        
        print(f"Generated trajectories: {trajectories.shape}")
        
        # Test detection methods
        methods = [DetectionMethod.TRIANGULATION, DetectionMethod.SPACE_CARVING, DetectionMethod.HYBRID]
        
        for method in methods:
            print(f"\nTesting {method.value}:")
            
            try:
                # Test one frame
                drone_positions = trajectories[:, 0, :]
                timestamp = times[0]
                
                start_time = time.time()
                # Use parallel sensor observations for better performance
                observations = sensor_array.observe_targets_parallel(drone_positions, timestamp)
                obs_time = time.time() - start_time
                
                start_time = time.time()
                detected_targets = detection_pipeline.process_sensor_observations_parallel(
                    observations, timestamp, method
                )
                detect_time = time.time() - start_time
                
                print(f"  Observations: {len(observations)} sensors, {sum(len(obs.detected_objects) for obs in observations)} detections")
                print(f"  Processing: {obs_time*1000:.1f}ms obs + {detect_time*1000:.1f}ms detect = {(obs_time+detect_time)*1000:.1f}ms total")
                print(f"  Results: {len(detected_targets)} targets detected")
                
                if detected_targets:
                    for i, target in enumerate(detected_targets):
                        print(f"    Target {i+1}: pos={target.position}, conf={target.confidence:.3f}")
                
            except Exception as e:
                print(f"  ERROR: {e}")
        
        print("\nQuick test completed!")
        
    if __name__ == "__main__":
        quick_test()
        
except ImportError as e:
    print(f"Import error: {e}")
    print("Make sure all files are in the src/ directory")
except Exception as e:
    print(f"Test error: {e}")