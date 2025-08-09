"""
Debug test script to check voxel count and optimize resolution
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
    
    def calculate_voxel_count(bounds, resolution):
        """Calculate how many voxels will be created"""
        x_range = bounds.x_max - bounds.x_min
        y_range = bounds.y_max - bounds.y_min
        z_range = bounds.z_max - bounds.z_min
        
        nx = int(np.ceil(x_range / resolution))
        ny = int(np.ceil(y_range / resolution))
        nz = int(np.ceil(z_range / resolution))
        
        total_voxels = nx * ny * nz
        memory_mb = total_voxels * 3 * 8 / (1024 * 1024)  # 3 coordinates, 8 bytes each
        
        return total_voxels, memory_mb, (nx, ny, nz)
    
    def quick_test():
        print("Debug Volumetric Detection Test")
        print("=" * 40)
        
        # Setup
        bounds = SimulationBounds()
        print(f"Simulation bounds: {bounds.x_max-bounds.x_min}x{bounds.y_max-bounds.y_min}x{bounds.z_max-bounds.z_min}m")
        
        # Test different resolutions
        resolutions = [10.0, 7.0, 5.0, 3.0]
        
        print("\nVoxel count analysis:")
        for res in resolutions:
            voxel_count, memory_mb, dims = calculate_voxel_count(bounds, res)
            print(f"  Resolution {res}m: {voxel_count:,} voxels ({dims}), ~{memory_mb:.1f}MB")
        
        # Choose optimal resolution (target < 1 million voxels)
        optimal_resolution = 10.0  # Start conservative
        for res in resolutions:
            voxel_count, _, _ = calculate_voxel_count(bounds, res)
            if voxel_count < 1000000:  # Less than 1 million voxels
                optimal_resolution = res
                break
        
        print(f"\nUsing resolution: {optimal_resolution}m")
        
        drone_generator = DroneSwarmGenerator(bounds)
        sensor_array = create_standard_sensor_array(bounds, "perimeter")
        detection_pipeline = VolumetricDetectionPipeline(bounds, voxel_resolution=optimal_resolution)
        
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
        
        # Test only space carving first to debug
        method = DetectionMethod.SPACE_CARVING
        print(f"\nTesting {method.value} (debug mode):")
        
        try:
            # Test one frame
            drone_positions = trajectories[:, 0, :]
            timestamp = times[0]
            
            print("  Step 1: Generating observations...")
            start_time = time.time()
            observations = sensor_array.observe_targets_parallel(drone_positions, timestamp)
            obs_time = time.time() - start_time
            print(f"    Observations took: {obs_time*1000:.1f}ms")
            
            print("  Step 2: Getting voxel coordinates...")
            start_time = time.time()
            voxel_coords = detection_pipeline._get_all_voxel_coordinates()
            coord_time = time.time() - start_time
            print(f"    Voxel generation took: {coord_time*1000:.1f}ms")
            print(f"    Created {len(voxel_coords):,} voxel coordinates")
            
            print("  Step 3: Processing detection...")
            start_time = time.time()
            detected_targets = detection_pipeline._parallel_space_carving(
                observations, timestamp, max_workers=8
            )
            detect_time = time.time() - start_time
            
            print(f"  Results:")
            print(f"    Observations: {len(observations)} sensors, {sum(len(obs.detected_objects) for obs in observations)} detections")
            print(f"    Processing: {obs_time*1000:.1f}ms obs + {detect_time*1000:.1f}ms detect = {(obs_time+detect_time)*1000:.1f}ms total")
            print(f"    Targets detected: {len(detected_targets)}")
            
            if detected_targets:
                for i, target in enumerate(detected_targets[:3]):  # Show first 3
                    print(f"      Target {i+1}: pos={target.position}, conf={target.confidence:.3f}")
            
        except Exception as e:
            print(f"  ERROR: {e}")
            import traceback
            traceback.print_exc()
        
        print("\nDebug test completed!")
        
    if __name__ == "__main__":
        quick_test()
        
except ImportError as e:
    print(f"Import error: {e}")
    print("Make sure all files are in the src/ directory")
except Exception as e:
    print(f"Test error: {e}")
    import traceback
    traceback.print_exc()