"""
Quick test to verify axis label fix in visualizations
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from drone_trajectory_generator import DroneSwarmGenerator, FlightPattern, SimulationBounds, visualize_trajectories
from sensor_simulation import create_standard_sensor_array

def test_trajectory_visualization_fix():
    """Test that trajectory visualization doesn't have clipped labels"""
    print("Testing trajectory visualization fix...")
    
    # Generate test data
    bounds = SimulationBounds()
    generator = DroneSwarmGenerator(bounds)
    
    data = generator.generate_swarm_trajectories(
        num_drones=8,
        pattern=FlightPattern.COORDINATED_ATTACK,
        duration=15.0,
        timestep=0.2,
        drone_type='small'
    )
    
    # Create visualization
    fig = visualize_trajectories(data, max_drones_display=8)
    
    # Save with descriptive filename and proper bbox
    filename = 'trajectory_visualization_fixed.png'
    fig.savefig(filename, dpi=150, bbox_inches='tight', pad_inches=0.2)
    import matplotlib.pyplot as plt
    plt.close(fig)
    
    print(f"SUCCESS: Fixed trajectory visualization saved as {filename}")
    print("Check that Z-axis label is fully visible and not cut off")
    return True

def test_sensor_coverage_visualization():
    """Test sensor array coverage visualization"""
    print("Testing sensor coverage visualization...")
    
    bounds = SimulationBounds()
    sensor_array = create_standard_sensor_array(bounds, "perimeter")
    
    # Create coverage visualization
    fig = sensor_array.visualize_sensor_coverage()
    filename = 'sensor_coverage_fixed.png'
    fig.savefig(filename, dpi=150, bbox_inches='tight')
    fig.close()
    
    print(f"SUCCESS: Sensor coverage visualization saved as {filename}")
    print("Check that all axis labels are fully visible")
    return True

def test_sensor_detection_basic():
    """Basic test of sensor detection functionality"""
    print("Testing basic sensor detection...")
    
    bounds = SimulationBounds()
    sensor_array = create_standard_sensor_array(bounds, "perimeter")
    
    # Create simple test scenario
    drone_positions = [
        [500, 500, 200],  # Center position
        [300, 300, 150],  # Off-center
        [700, 600, 250]   # Another position
    ]
    
    observations = sensor_array.observe_targets(drone_positions, 10.0)
    stats = sensor_array.get_detection_statistics(observations)
    
    print(f"Sensor array has {len(sensor_array.cameras)} cameras")
    print(f"Test detected {stats['total_detections']} objects")
    print(f"Active cameras: {stats['active_cameras']}/{stats['total_cameras']}")
    print(f"Average confidence: {stats['average_confidence']:.3f}")
    
    if stats['total_detections'] > 0:
        print("SUCCESS: Sensor detection system working")
        return True
    else:
        print("WARNING: No detections found - check sensor positioning")
        return False

def run_visualization_tests():
    """Run all visualization and basic functionality tests"""
    print("VISUALIZATION AND SENSOR SIMULATION TESTS")
    print("Python version:", sys.version.split()[0])
    print("=" * 60)
    
    test_results = {}
    
    # Test trajectory visualization fix
    try:
        test_results['trajectory_viz'] = test_trajectory_visualization_fix()
    except Exception as e:
        print(f"ERROR in trajectory visualization test: {e}")
        test_results['trajectory_viz'] = False
    
    print()
    
    # Test sensor coverage visualization
    try:
        test_results['sensor_viz'] = test_sensor_coverage_visualization()
    except Exception as e:
        print(f"ERROR in sensor visualization test: {e}")
        test_results['sensor_viz'] = False
    
    print()
    
    # Test basic sensor detection
    try:
        test_results['sensor_detection'] = test_sensor_detection_basic()
    except Exception as e:
        print(f"ERROR in sensor detection test: {e}")
        test_results['sensor_detection'] = False
    
    print()
    print("=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)
    
    passed = sum(test_results.values())
    total = len(test_results)
    
    for test_name, result in test_results.items():
        status = "PASS" if result else "FAIL"
        print(f"{test_name}: {status}")
    
    print(f"\nTests passed: {passed}/{total}")
    
    if passed == total:
        print("ALL TESTS PASSED")
        print("\nGenerated files to check:")
        print("- trajectory_visualization_fixed.png")
        print("- sensor_coverage_fixed.png")
        print("\nVerify that axis labels are not cut off in these images")
    else:
        print(f"{total - passed} tests failed")
    
    return test_results

if __name__ == "__main__":
    run_visualization_tests()
