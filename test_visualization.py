"""
Enhanced visualization test with customizable parameters
Saves images to 'images' folder with drone count and attack pattern options
"""

import sys
import os
import argparse
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from drone_trajectory_generator import DroneSwarmGenerator, FlightPattern, SimulationBounds, visualize_trajectories
from sensor_simulation import create_standard_sensor_array

def ensure_images_folder():
    """Create images folder if it doesn't exist"""
    images_dir = 'images'
    if not os.path.exists(images_dir):
        os.makedirs(images_dir)
        print(f"Created '{images_dir}' folder")
    return images_dir

def get_flight_pattern(pattern_name):
    """Convert string pattern name to FlightPattern enum"""
    # Updated pattern mappings based on actual available enum values
    pattern_map = {
        'coordinated': 'COORDINATED_ATTACK',
        'attack': 'COORDINATED_ATTACK',
        'evasive': 'EVASIVE_MANEUVERS',
        'maneuvers': 'EVASIVE_MANEUVERS',
        'formation': 'FORMATION_FLYING',
        'flying': 'FORMATION_FLYING',
        'perimeter': 'PERIMETER_SWEEP',
        'sweep': 'PERIMETER_SWEEP',
        'patrol': 'PERIMETER_SWEEP',  # Map patrol to perimeter sweep
        'random': 'RANDOM_DISPERSAL',
        'dispersal': 'RANDOM_DISPERSAL',
        'swarm': 'RANDOM_DISPERSAL',  # Map swarm to random dispersal
        'surveillance': 'PERIMETER_SWEEP'  # Map surveillance to perimeter sweep
    }
    
    pattern_key = pattern_name.lower()
    
    # Try to find matching pattern
    if pattern_key in pattern_map:
        attr_name = pattern_map[pattern_key]
        if hasattr(FlightPattern, attr_name):
            return getattr(FlightPattern, attr_name)
    
    # If no exact match, show available options and use fallback
    print(f"Pattern '{pattern_name}' mapped to available patterns. Available FlightPattern attributes:")
    available_attrs = [attr for attr in dir(FlightPattern) if not attr.startswith('_') and not callable(getattr(FlightPattern, attr, None))]
    for attr in available_attrs:
        print(f"  - {attr}")
    
    # Use COORDINATED_ATTACK as default fallback
    print("Using COORDINATED_ATTACK as fallback")
    return FlightPattern.COORDINATED_ATTACK

def test_trajectory_visualization_custom(num_drones=8, attack_pattern='coordinated', duration=15.0, drone_type='small'):
    """Test trajectory visualization with custom parameters"""
    print(f"Testing trajectory visualization with {num_drones} drones, {attack_pattern} pattern...")
    
    try:
        images_dir = ensure_images_folder()
        
        # Generate test data
        bounds = SimulationBounds()
        generator = DroneSwarmGenerator(bounds)
        
        flight_pattern = get_flight_pattern(attack_pattern)
        
        # Try generating trajectories with error handling for drone_type
        try:
            data = generator.generate_swarm_trajectories(
                num_drones=num_drones,
                pattern=flight_pattern,
                duration=duration,
                timestep=0.2,
                drone_type=drone_type
            )
        except Exception as drone_type_error:
            print(f"Error with drone_type '{drone_type}': {drone_type_error}")
            print("Trying with default drone_type 'small'...")
            data = generator.generate_swarm_trajectories(
                num_drones=num_drones,
                pattern=flight_pattern,
                duration=duration,
                timestep=0.2,
                drone_type='small'  # Fallback to 'small'
            )
            drone_type = 'small'  # Update for filename
        
        # Create visualization
        fig = visualize_trajectories(data, max_drones_display=num_drones)
        
        # Save with descriptive filename
        filename = f'trajectory_{attack_pattern}_{num_drones}drones_{drone_type}.png'
        filepath = os.path.join(images_dir, filename)
        fig.savefig(filepath, dpi=150, bbox_inches='tight', pad_inches=0.2)
        
        import matplotlib.pyplot as plt
        plt.close(fig)
        
        print(f"SUCCESS: Trajectory visualization saved as {filepath}")
        print(f"  Drones: {num_drones}")
        print(f"  Pattern: {attack_pattern} -> {flight_pattern}")
        print(f"  Duration: {duration}s")
        print(f"  Type: {drone_type}")
        return True, filepath
        
    except Exception as e:
        print(f"ERROR in trajectory visualization: {e}")
        import traceback
        traceback.print_exc()
        return False, None

def test_sensor_coverage_visualization():
    """Test sensor array coverage visualization"""
    print("Testing sensor coverage visualization...")
    
    try:
        images_dir = ensure_images_folder()
        
        bounds = SimulationBounds()
        sensor_array = create_standard_sensor_array(bounds, "perimeter")
        
        # Create coverage visualization
        fig = sensor_array.visualize_sensor_coverage()
        filename = 'sensor_coverage.png'
        filepath = os.path.join(images_dir, filename)
        fig.savefig(filepath, dpi=150, bbox_inches='tight')
        
        import matplotlib.pyplot as plt
        plt.close(fig)
        
        print(f"SUCCESS: Sensor coverage visualization saved as {filepath}")
        return True, filepath
        
    except Exception as e:
        print(f"ERROR in sensor coverage visualization: {e}")
        return False, None

def test_sensor_detection_basic():
    """Basic test of sensor detection functionality"""
    print("Testing basic sensor detection...")
    
    try:
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
            
    except Exception as e:
        print(f"ERROR in sensor detection: {e}")
        return False

def run_enhanced_visualization_tests(num_drones=8, attack_pattern='coordinated', duration=15.0, drone_type='small'):
    """Run visualization tests with custom parameters"""
    print("ENHANCED VISUALIZATION AND SENSOR SIMULATION TESTS")
    print("Python version:", sys.version.split()[0])
    print("=" * 60)
    print(f"Configuration:")
    print(f"  Drones: {num_drones}")
    print(f"  Attack Pattern: {attack_pattern}")
    print(f"  Duration: {duration}s")
    print(f"  Drone Type: {drone_type}")
    print("=" * 60)
    
    test_results = {}
    generated_files = []
    
    # Test trajectory visualization with custom parameters
    try:
        success, filepath = test_trajectory_visualization_custom(num_drones, attack_pattern, duration, drone_type)
        test_results['trajectory_viz'] = success
        if filepath:
            generated_files.append(filepath)
    except Exception as e:
        print(f"ERROR in trajectory visualization test: {e}")
        test_results['trajectory_viz'] = False
    
    print()
    
    # Test sensor coverage visualization
    try:
        success, filepath = test_sensor_coverage_visualization()
        test_results['sensor_viz'] = success
        if filepath:
            generated_files.append(filepath)
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
    else:
        print(f"{total - passed} tests failed")
    
    if generated_files:
        print(f"\nGenerated files in 'images' folder:")
        for filepath in generated_files:
            print(f"  - {filepath}")
        print("\nVerify that axis labels are not cut off in these images")
    
    return test_results

def main():
    """Main function with command line argument parsing"""
    parser = argparse.ArgumentParser(description='Enhanced Visualization Test for Drone Detection System')
    
    parser.add_argument('--drones', '-d', type=int, default=8,
                       help='Number of drones to simulate (default: 8)')
    
    parser.add_argument('--pattern', '-p', type=str, default='coordinated',
                       help='Attack pattern type (default: coordinated). Use --list-patterns to see available options')
    
    parser.add_argument('--duration', type=float, default=15.0,
                       help='Simulation duration in seconds (default: 15.0)')
    
    parser.add_argument('--type', '-t', type=str, default='small',
                       choices=['small', 'medium', 'large'],
                       help='Drone type (default: small)')
    
    parser.add_argument('--list-patterns', action='store_true',
                       help='List available attack patterns and exit')
    
    args = parser.parse_args()
    
    if args.list_patterns:
        print("Available attack patterns based on your FlightPattern enum:")
        print("  coordinated  - COORDINATED_ATTACK formation")
        print("  evasive      - EVASIVE_MANEUVERS pattern")
        print("  formation    - FORMATION_FLYING pattern")
        print("  perimeter    - PERIMETER_SWEEP pattern")
        print("  random       - RANDOM_DISPERSAL pattern")
        print("  swarm        - Maps to RANDOM_DISPERSAL")
        print("  patrol       - Maps to PERIMETER_SWEEP")
        print("  surveillance - Maps to PERIMETER_SWEEP")
        return
    
    # Validate parameters
    if args.drones < 1 or args.drones > 50:
        print("ERROR: Number of drones must be between 1 and 50")
        return
    
    if args.duration < 1.0 or args.duration > 60.0:
        print("ERROR: Duration must be between 1.0 and 60.0 seconds")
        return
    
    # Run tests with custom parameters
    run_enhanced_visualization_tests(
        num_drones=args.drones,
        attack_pattern=args.pattern,
        duration=args.duration,
        drone_type=args.type
    )

if __name__ == "__main__":
    main()
