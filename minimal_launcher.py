"""
Minimal launcher that uses your working debug test approach
"""

import sys
import os
import time
import argparse
import json
from datetime import datetime

# Add src directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

def main():
    print("Minimal Volumetric Detection Test Launcher")
    print("=" * 50)
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--resolution', type=float, default=10.0,
                       help='Voxel resolution in meters (default: 10.0)')
    parser.add_argument('--drones', type=int, default=5,
                       help='Number of drones to test (default: 5)')
    parser.add_argument('--duration', type=float, default=2.0,
                       help='Simulation duration in seconds (default: 2.0)')
    args = parser.parse_args()
    
    try:
        from drone_trajectory_generator import DroneSwarmGenerator, FlightPattern, SimulationBounds
        from sensor_simulation import create_standard_sensor_array
        from volumetric_detection import VolumetricDetectionPipeline, DetectionMethod
        
        # Setup with configurable parameters
        bounds = SimulationBounds()
        print(f"Simulation bounds: {bounds.x_max-bounds.x_min}x{bounds.y_max-bounds.y_min}x{bounds.z_max-bounds.z_min}m")
        
        # Check voxel count before proceeding
        x_range = bounds.x_max - bounds.x_min
        y_range = bounds.y_max - bounds.y_min
        z_range = bounds.z_max - bounds.z_min
        
        nx = int(np.ceil(x_range / args.resolution))
        ny = int(np.ceil(y_range / args.resolution))
        nz = int(np.ceil(z_range / args.resolution))
        total_voxels = nx * ny * nz
        memory_mb = total_voxels * 3 * 8 / (1024 * 1024)
        
        print(f"Voxel resolution: {args.resolution}m")
        print(f"Voxel count: {total_voxels:,} ({nx}x{ny}x{nz})")
        print(f"Estimated memory: {memory_mb:.1f}MB")
        
        if total_voxels > 2000000:
            print("⚠️ Warning: High voxel count may cause performance issues")
            response = input("Continue? (y/N): ")
            if response.lower() != 'y':
                return
        
        # Initialize components
        print("\nInitializing components...")
        drone_generator = DroneSwarmGenerator(bounds)
        sensor_array = create_standard_sensor_array(bounds, "perimeter")
        detection_pipeline = VolumetricDetectionPipeline(bounds, voxel_resolution=args.resolution)
        
        print(f"Setup complete: {len(sensor_array.cameras)} cameras")
        
        # Generate trajectories
        print(f"Generating {args.drones} drone trajectories...")
        trajectory_data = drone_generator.generate_swarm_trajectories(
            num_drones=args.drones,
            pattern=FlightPattern.COORDINATED_ATTACK,
            duration=args.duration,
            timestep=0.5,
            drone_type='small'
        )
        
        trajectories = trajectory_data['trajectories']
        times = trajectory_data['times']
        print(f"Generated trajectories: {trajectories.shape}")
        
        # Test all detection methods
        methods = [DetectionMethod.TRIANGULATION, DetectionMethod.SPACE_CARVING, DetectionMethod.HYBRID]
        results = {}
        
        for method in methods:
            print(f"\nTesting {method.value}:")
            method_results = []
            
            try:
                for frame_idx in range(len(times)):
                    drone_positions = trajectories[:, frame_idx, :]
                    timestamp = times[frame_idx]
                    
                    start_time = time.time()
                    observations = sensor_array.observe_targets_parallel(drone_positions, timestamp)
                    obs_time = time.time() - start_time
                    
                    start_time = time.time()
                    if method == DetectionMethod.SPACE_CARVING:
                        detected_targets = detection_pipeline._parallel_space_carving(
                            observations, timestamp, max_workers=8
                        )
                    else:
                        detected_targets = detection_pipeline.process_sensor_observations_parallel(
                            observations, timestamp, method
                        )
                    detect_time = time.time() - start_time
                    
                    total_detections = sum(len(obs.detected_objects) for obs in observations)
                    
                    frame_result = {
                        'frame': frame_idx,
                        'timestamp': timestamp,
                        'observations': len(observations),
                        'detections': total_detections,
                        'targets_found': len(detected_targets),
                        'obs_time_ms': obs_time * 1000,
                        'detect_time_ms': detect_time * 1000,
                        'total_time_ms': (obs_time + detect_time) * 1000
                    }
                    method_results.append(frame_result)
                    
                    print(f"  Frame {frame_idx}: {total_detections} detections → {len(detected_targets)} targets ({(obs_time+detect_time)*1000:.1f}ms)")
                
                results[method.value] = method_results
                
                # Summary for this method
                avg_time = sum(r['total_time_ms'] for r in method_results) / len(method_results)
                total_targets = sum(r['targets_found'] for r in method_results)
                print(f"  Average processing time: {avg_time:.1f}ms")
                print(f"  Total targets detected: {total_targets}")
                
            except Exception as e:
                print(f"  ERROR in {method.value}: {e}")
                results[method.value] = {'error': str(e)}
        
        # Save results
        result_data = {
            'timestamp': datetime.now().isoformat(),
            'parameters': {
                'resolution': args.resolution,
                'drones': args.drones,
                'duration': args.duration,
                'voxel_count': total_voxels,
                'memory_mb': memory_mb
            },
            'results': results
        }
        
        output_file = 'minimal_test_results.json'
        with open(output_file, 'w') as f:
            json.dump(result_data, f, indent=2)
        
        print(f"\nTest completed! Results saved to {output_file}")
        
    except ImportError as e:
        print(f"Import error: {e}")
        print("Make sure all files are in the src/ directory")
        sys.exit(1)
    except Exception as e:
        print(f"Test error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    import numpy as np  # Import here to avoid issues
    main()
