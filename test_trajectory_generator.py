"""
Test script for drone_trajectory_generator.py
Tests functionality and performance on your hardware setup
"""

import sys
import time
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend for testing
import matplotlib.pyplot as plt
from drone_trajectory_generator import (
    DroneSwarmGenerator, 
    FlightPattern, 
    SimulationBounds,
    visualize_trajectories
)

def test_basic_functionality():
    """Test basic trajectory generation functionality"""
    print("=" * 60)
    print("TESTING BASIC FUNCTIONALITY")
    print("=" * 60)
    
    # Initialize generator
    bounds = SimulationBounds()
    generator = DroneSwarmGenerator(bounds)
    
    print(f"Simulation bounds: {bounds.x_max}m x {bounds.y_max}m x {bounds.z_max}m")
    print(f"Available drone types: {list(generator.drone_types.keys())}")
    
    # Test simple trajectory generation
    try:
        data = generator.generate_swarm_trajectories(
            num_drones=5,
            pattern=FlightPattern.COORDINATED_ATTACK,
            duration=10.0,
            timestep=0.1,
            drone_type='small'
        )
        
        trajectories = data['trajectories']
        print(f"SUCCESS: Generated trajectories shape: {trajectories.shape}")
        print(f"Time steps: {len(data['times'])}")
        print(f"Duration: {data['times'][-1]:.1f} seconds")
        
        return True
        
    except Exception as e:
        print(f"ERROR in basic functionality test: {e}")
        return False

def test_all_flight_patterns():
    """Test all available flight patterns"""
    print("\n" + "=" * 60)
    print("TESTING ALL FLIGHT PATTERNS")
    print("=" * 60)
    
    bounds = SimulationBounds()
    generator = DroneSwarmGenerator(bounds)
    
    patterns = [
        FlightPattern.COORDINATED_ATTACK,
        FlightPattern.FORMATION_FLYING,
        FlightPattern.RANDOM_DISPERSAL,
        FlightPattern.EVASIVE_MANEUVERS,
        FlightPattern.PERIMETER_SWEEP
    ]
    
    results = {}
    
    for pattern in patterns:
        try:
            start_time = time.time()
            
            data = generator.generate_swarm_trajectories(
                num_drones=10,
                pattern=pattern,
                duration=15.0,
                timestep=0.1,
                drone_type='small'
            )
            
            generation_time = time.time() - start_time
            
            trajectories = data['trajectories']
            velocities = data['velocities']
            
            # Calculate statistics
            avg_speed = np.mean(np.linalg.norm(velocities, axis=2))
            max_altitude = np.max(trajectories[:, :, 2])
            min_altitude = np.min(trajectories[:, :, 2])
            
            results[pattern.value] = {
                'success': True,
                'generation_time': generation_time,
                'avg_speed': avg_speed,
                'altitude_range': (min_altitude, max_altitude),
                'shape': trajectories.shape
            }
            
            print(f"PATTERN: {pattern.value}")
            print(f"  Generation time: {generation_time:.3f} seconds")
            print(f"  Average speed: {avg_speed:.2f} m/s")
            print(f"  Altitude range: {min_altitude:.1f} - {max_altitude:.1f} m")
            print(f"  Data shape: {trajectories.shape}")
            
        except Exception as e:
            print(f"ERROR with pattern {pattern.value}: {e}")
            results[pattern.value] = {'success': False, 'error': str(e)}
    
    return results

def test_performance_scaling():
    """Test performance with different numbers of drones"""
    print("\n" + "=" * 60)
    print("TESTING PERFORMANCE SCALING")
    print("=" * 60)
    
    bounds = SimulationBounds()
    generator = DroneSwarmGenerator(bounds)
    
    drone_counts = [5, 10, 20, 30, 40, 50]
    performance_results = {}
    
    for num_drones in drone_counts:
        try:
            print(f"Testing with {num_drones} drones...")
            
            start_time = time.time()
            
            data = generator.generate_swarm_trajectories(
                num_drones=num_drones,
                pattern=FlightPattern.COORDINATED_ATTACK,
                duration=20.0,
                timestep=0.1,
                drone_type='small'
            )
            
            generation_time = time.time() - start_time
            
            # Memory usage estimation
            trajectories = data['trajectories']
            memory_mb = trajectories.nbytes / (1024 * 1024)
            
            performance_results[num_drones] = {
                'generation_time': generation_time,
                'memory_usage_mb': memory_mb,
                'time_per_drone': generation_time / num_drones,
                'success': True
            }
            
            print(f"  Generation time: {generation_time:.3f} seconds")
            print(f"  Time per drone: {generation_time/num_drones:.4f} seconds")
            print(f"  Memory usage: {memory_mb:.2f} MB")
            print(f"  Performance ratio: {num_drones/generation_time:.1f} drones/second")
            
        except Exception as e:
            print(f"  ERROR with {num_drones} drones: {e}")
            performance_results[num_drones] = {'success': False, 'error': str(e)}
    
    return performance_results

def test_data_validation():
    """Validate the quality and consistency of generated data"""
    print("\n" + "=" * 60)
    print("TESTING DATA VALIDATION")
    print("=" * 60)
    
    bounds = SimulationBounds()
    generator = DroneSwarmGenerator(bounds)
    
    data = generator.generate_swarm_trajectories(
        num_drones=15,
        pattern=FlightPattern.COORDINATED_ATTACK,
        duration=30.0,
        timestep=0.1,
        drone_type='small'
    )
    
    trajectories = data['trajectories']
    velocities = data['velocities']
    times = data['times']
    spec = data['spec']
    
    print(f"Validating data for {data['num_drones']} drones...")
    
    # Check data consistency
    validation_results = {}
    
    # 1. Check trajectory bounds
    x_coords = trajectories[:, :, 0]
    y_coords = trajectories[:, :, 1] 
    z_coords = trajectories[:, :, 2]
    
    x_in_bounds = np.all((x_coords >= bounds.x_min) & (x_coords <= bounds.x_max))
    y_in_bounds = np.all((y_coords >= bounds.y_min) & (y_coords <= bounds.y_max))
    z_in_bounds = np.all((z_coords >= bounds.z_min) & (z_coords <= bounds.z_max))
    
    validation_results['bounds_check'] = x_in_bounds and y_in_bounds and z_in_bounds
    print(f"Boundary constraints: {'PASS' if validation_results['bounds_check'] else 'FAIL'}")
    
    # 2. Check velocity constraints
    velocity_magnitudes = np.linalg.norm(velocities, axis=2)
    max_observed_velocity = np.max(velocity_magnitudes)
    velocity_constraint_ok = max_observed_velocity <= spec.max_velocity * 1.1  # 10% tolerance
    
    validation_results['velocity_check'] = velocity_constraint_ok
    print(f"Velocity constraints: {'PASS' if velocity_constraint_ok else 'FAIL'}")
    print(f"  Max observed velocity: {max_observed_velocity:.2f} m/s")
    print(f"  Spec max velocity: {spec.max_velocity:.2f} m/s")
    
    # 3. Check for NaN or infinite values
    has_nan = np.any(np.isnan(trajectories)) or np.any(np.isnan(velocities))
    has_inf = np.any(np.isinf(trajectories)) or np.any(np.isinf(velocities))
    
    validation_results['data_quality'] = not (has_nan or has_inf)
    print(f"Data quality (no NaN/Inf): {'PASS' if validation_results['data_quality'] else 'FAIL'}")
    
    # 4. Check temporal consistency
    time_diffs = np.diff(times)
    consistent_timestep = np.allclose(time_diffs, time_diffs[0], rtol=1e-10)
    
    validation_results['temporal_consistency'] = consistent_timestep
    print(f"Temporal consistency: {'PASS' if consistent_timestep else 'FAIL'}")
    
    return validation_results

def generate_test_visualizations():
    """Generate test visualizations (saved as files)"""
    print("\n" + "=" * 60)
    print("GENERATING TEST VISUALIZATIONS")
    print("=" * 60)
    
    bounds = SimulationBounds()
    generator = DroneSwarmGenerator(bounds)
    
    # Generate sample data
    data = generator.generate_swarm_trajectories(
        num_drones=12,
        pattern=FlightPattern.COORDINATED_ATTACK,
        duration=25.0,
        timestep=0.1,
        drone_type='small'
    )
    
    try:
        # Create visualization
        fig = visualize_trajectories(data, max_drones_display=8)
        
        # Save to file instead of showing
        filename = 'test_trajectory_visualization.png'
        plt.savefig(filename, dpi=150, bbox_inches='tight')
        plt.close()
        
        print(f"SUCCESS: Visualization saved as {filename}")
        return True
        
    except Exception as e:
        print(f"ERROR generating visualization: {e}")
        return False

def run_comprehensive_test():
    """Run all tests and generate summary report"""
    print("DRONE TRAJECTORY GENERATOR - COMPREHENSIVE TEST")
    print("Python version:", sys.version)
    print("Test started at:", time.strftime("%Y-%m-%d %H:%M:%S"))
    print()
    
    test_results = {}
    
    # Run all tests
    test_results['basic_functionality'] = test_basic_functionality()
    test_results['flight_patterns'] = test_all_flight_patterns()
    test_results['performance_scaling'] = test_performance_scaling()
    test_results['data_validation'] = test_data_validation()
    test_results['visualization'] = generate_test_visualizations()
    
    # Generate summary
    print("\n" + "=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)
    
    passed_tests = sum(1 for result in test_results.values() if result is True or 
                      (isinstance(result, dict) and 
                       all(v.get('success', False) if isinstance(v, dict) else v 
                           for v in result.values())))
    
    total_tests = len(test_results)
    
    print(f"Tests passed: {passed_tests}/{total_tests}")
    
    for test_name, result in test_results.items():
        status = "PASS" if (result is True or 
                          (isinstance(result, dict) and 
                           all(v.get('success', False) if isinstance(v, dict) else v 
                               for v in result.values()))) else "FAIL"
        print(f"  {test_name}: {status}")
    
    if passed_tests == total_tests:
        print("\nALL TESTS PASSED - Trajectory generator is ready for use!")
    else:
        print(f"\n{total_tests - passed_tests} tests failed - Check error messages above")
    
    print(f"\nTest completed at: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    
    return test_results

if __name__ == "__main__":
    # Run comprehensive test
    results = run_comprehensive_test()
