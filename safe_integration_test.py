"""
Safe version of integration test with reasonable parameters
Based on your working debug_test.py configuration
"""

import sys
import os
import time
import json
import numpy as np
from datetime import datetime
from typing import Dict, List, Any

# Force reasonable threading
os.environ['OMP_NUM_THREADS'] = '8'
os.environ['MKL_NUM_THREADS'] = '8' 
os.environ['NUMEXPR_NUM_THREADS'] = '8'

# Add src directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

try:
    from drone_trajectory_generator import DroneSwarmGenerator, FlightPattern, SimulationBounds
    from sensor_simulation import create_standard_sensor_array
    from volumetric_detection import VolumetricDetectionPipeline, DetectionMethod

    def calculate_safe_parameters():
        """Calculate safe parameters to avoid memory issues"""
        bounds = SimulationBounds()
        
        # Test different configurations
        configs = [
            {'resolution': 10.0, 'drones': 5, 'duration': 2.0, 'name': 'minimal'},
            {'resolution': 7.0, 'drones': 8, 'duration': 3.0, 'name': 'light'},
            {'resolution': 5.0, 'drones': 12, 'duration': 4.0, 'name': 'moderate'},
        ]
        
        safe_configs = []
        
        for config in configs:
            # Calculate voxel count
            x_range = bounds.x_max - bounds.x_min
            y_range = bounds.y_max - bounds.y_min
            z_range = bounds.z_max - bounds.z_min
            
            nx = int(np.ceil(x_range / config['resolution']))
            ny = int(np.ceil(y_range / config['resolution']))
            nz = int(np.ceil(z_range / config['resolution']))
            
            total_voxels = nx * ny * nz
            memory_mb = total_voxels * 3 * 8 / (1024 * 1024)
            
            config['voxels'] = total_voxels
            config['memory_mb'] = memory_mb
            
            # Only include safe configurations (< 100MB memory)
            if memory_mb < 100:
                safe_configs.append(config)
                print(f"Safe config '{config['name']}': {config['resolution']}m res, "
                      f"{config['drones']} drones, {config['duration']}s, "
                      f"{total_voxels:,} voxels, {memory_mb:.1f}MB")
            else:
                print(f"Skipping '{config['name']}': {memory_mb:.1f}MB too high")
        
        return safe_configs

    def run_safe_integration_test():
        """Run integration test with safe parameters"""
        print("Safe Volumetric Detection Integration Test")
        print("=" * 60)
        print("Using conservative parameters to avoid memory issues")
        print()
        
        # Calculate safe parameters
        safe_configs = calculate_safe_parameters()
        
        if not safe_configs:
            print("No safe configurations found!")
            return None
        
        print(f"\nTesting {len(safe_configs)} safe configurations...")
        print()
        
        # Setup base components
        bounds = SimulationBounds()
        drone_generator = DroneSwarmGenerator(bounds)
        sensor_array = create_standard_sensor_array(bounds, "perimeter")
        
        print(f"Base setup: {len(sensor_array.cameras)} cameras")
        print(f"Simulation bounds: {bounds.x_max-bounds.x_min}x{bounds.y_max-bounds.y_min}x{bounds.z_max-bounds.z_min}m")
        print()
        
        results = {
            'timestamp': datetime.now().isoformat(),
            'configurations': [],
            'summary': {}
        }
        
        # Test each safe configuration
        for config_idx, config in enumerate(safe_configs):
            print(f"Configuration {config_idx + 1}/{len(safe_configs)}: {config['name']}")
            print(f"  Resolution: {config['resolution']}m, Drones: {config['drones']}, Duration: {config['duration']}s")
            
            config_results = {
                'name': config['name'],
                'parameters': config,
                'methods': {}
            }
            
            try:
                # Create detection pipeline for this configuration
                detection_pipeline = VolumetricDetectionPipeline(
                    bounds, 
                    voxel_resolution=config['resolution']
                )
                
                # Generate trajectories
                trajectory_data = drone_generator.generate_swarm_trajectories(
                    num_drones=config['drones'],
                    pattern=FlightPattern.COORDINATED_ATTACK,
                    duration=config['duration'],
                    timestep=0.5,
                    drone_type='small'
                )
                
                trajectories = trajectory_data['trajectories']
                times = trajectory_data['times']
                
                print(f"    Generated {trajectories.shape[0]} drone trajectories over {len(times)} timesteps")
                
                # Test detection methods
                methods = [DetectionMethod.TRIANGULATION, DetectionMethod.SPACE_CARVING, DetectionMethod.HYBRID]
                
                for method in methods:
                    print(f"    Testing {method.value}...")
                    
                    method_results = {
                        'frames': [],
                        'total_time_ms': 0,
                        'avg_time_ms': 0,
                        'total_targets': 0,
                        'avg_targets': 0
                    }
                    
                    frame_times = []
                    frame_targets = []
                    
                    # Process each frame
                    for frame_idx in range(min(len(times), 5)):  # Limit to 5 frames for safety
                        drone_positions = trajectories[:, frame_idx, :]
                        timestamp = times[frame_idx]
                        
                        start_time = time.time()
                        
                        # Generate observations
                        observations = sensor_array.observe_targets_parallel(drone_positions, timestamp)
                        
                        # Process detection
                        if method == DetectionMethod.SPACE_CARVING:
                            detected_targets = detection_pipeline._parallel_space_carving(
                                observations, timestamp, max_workers=8
                            )
                        else:
                            detected_targets = detection_pipeline.process_sensor_observations_parallel(
                                observations, timestamp, method
                            )
                        
                        frame_time = (time.time() - start_time) * 1000
                        num_targets = len(detected_targets)
                        
                        frame_times.append(frame_time)
                        frame_targets.append(num_targets)
                        
                        method_results['frames'].append({
                            'frame': frame_idx,
                            'time_ms': frame_time,
                            'targets': num_targets,
                            'observations': len(observations),
                            'detections': sum(len(obs.detected_objects) for obs in observations)
                        })
                    
                    # Calculate method statistics
                    method_results['total_time_ms'] = sum(frame_times)
                    method_results['avg_time_ms'] = np.mean(frame_times)
                    method_results['total_targets'] = sum(frame_targets)
                    method_results['avg_targets'] = np.mean(frame_targets)
                    
                    config_results['methods'][method.value] = method_results
                    
                    print(f"      Avg time: {method_results['avg_time_ms']:.1f}ms, "
                          f"Avg targets: {method_results['avg_targets']:.1f}")
                
                print(f"    Configuration '{config['name']}' completed successfully")
                
            except Exception as e:
                print(f"    ERROR in configuration '{config['name']}': {e}")
                config_results['error'] = str(e)
            
            results['configurations'].append(config_results)
            print()
        
        # Generate summary
        successful_configs = [c for c in results['configurations'] if 'error' not in c]
        
        if successful_configs:
            print("Summary of Results:")
            print("-" * 40)
            
            for method_name in ['triangulation', 'space_carving', 'hybrid']:
                method_times = []
                method_targets = []
                
                for config in successful_configs:
                    if method_name in config['methods']:
                        method_data = config['methods'][method_name]
                        method_times.append(method_data['avg_time_ms'])
                        method_targets.append(method_data['avg_targets'])
                
                if method_times:
                    avg_time = np.mean(method_times)
                    avg_targets = np.mean(method_targets)
                    print(f"{method_name.capitalize()}: {avg_time:.1f}ms avg, {avg_targets:.1f} targets avg")
            
            results['summary'] = {
                'successful_configurations': len(successful_configs),
                'total_configurations': len(results['configurations']),
                'test_status': 'SUCCESS'
            }
        else:
            results['summary'] = {
                'successful_configurations': 0,
                'total_configurations': len(results['configurations']),
                'test_status': 'FAILED'
            }
        
        # Save results
        output_file = 'safe_integration_results.json'
        with open(output_file, 'w') as f:
            json.dump(results, f, indent=2)
        
        print(f"\nResults saved to: {output_file}")
        print(f"Test completed: {results['summary']['successful_configurations']}/{results['summary']['total_configurations']} configurations successful")
        
        return results

    if __name__ == "__main__":
        run_safe_integration_test()

except ImportError as e:
    print(f"Import error: {e}")
    print("Make sure all files are in the src/ directory")
except Exception as e:
    print(f"Test error: {e}")
    import traceback
    traceback.print_exc()
