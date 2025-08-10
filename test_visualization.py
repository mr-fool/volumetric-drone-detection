"""
Enhanced visualization test with customizable parameters
Saves images to 'images' folder with drone count and attack pattern options
NOW INCLUDES: Detection range analysis options while preserving all original functionality
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

# NEW DETECTION RANGE ANALYSIS FUNCTIONS
def test_all_camera_detection_ranges():
    """Generate individual detection range plots for each camera"""
    print("Testing individual camera detection range visualization...")
    
    try:
        images_dir = ensure_images_folder()
        
        bounds = SimulationBounds()
        sensor_array = create_standard_sensor_array(bounds, "perimeter")
        
        # Check if the new methods exist
        if not hasattr(sensor_array, 'visualize_all_camera_detection_ranges'):
            print("ERROR: Detection range analysis methods not found in sensor_simulation.py")
            print("Please add the enhanced methods to your VirtualSensorArray class first.")
            return False, []
        
        # Generate individual camera plots
        saved_files = sensor_array.visualize_all_camera_detection_ranges(
            elevation_slice=200.0, 
            output_dir=images_dir
        )
        
        print(f"SUCCESS: Generated {len(saved_files)} detection range plots")
        for filepath in saved_files:
            print(f"  Saved: {os.path.basename(filepath)}")
        
        return True, saved_files
        
    except Exception as e:
        print(f"ERROR in individual camera detection range visualization: {e}")
        print("Make sure you've added the enhanced methods to sensor_simulation.py")
        import traceback
        traceback.print_exc()
        return False, []

def test_representative_camera_explanation():
    """Generate single comprehensive plot explaining detection range concept"""
    print("Testing representative camera explanation visualization...")
    
    try:
        images_dir = ensure_images_folder()
        
        bounds = SimulationBounds()
        sensor_array = create_standard_sensor_array(bounds, "perimeter")
        
        # Check if the new methods exist
        if not hasattr(sensor_array, 'visualize_sensor_coverage_single_representative'):
            print("ERROR: Representative visualization method not found in sensor_simulation.py")
            print("Please add the enhanced methods to your VirtualSensorArray class first.")
            return False, None
        
        # Create comprehensive explanation plot
        fig = sensor_array.visualize_sensor_coverage_single_representative()
        
        filename = 'detection_range_explanation.png'
        filepath = os.path.join(images_dir, filename)
        fig.savefig(filepath, dpi=150, bbox_inches='tight')
        
        import matplotlib.pyplot as plt
        plt.close(fig)
        
        print(f"SUCCESS: Detection range explanation saved as {filepath}")
        return True, filepath
        
    except Exception as e:
        print(f"ERROR in detection range explanation: {e}")
        print("Make sure you've added the enhanced methods to sensor_simulation.py")
        import traceback
        traceback.print_exc()
        return False, None

def run_detection_range_analysis(analysis_type='representative'):
    """
    Run detection range analysis with different visualization options
    
    Args:
        analysis_type: 'individual', 'representative', or 'both'
    """
    print("DETECTION RANGE ANALYSIS")
    print(f"Analysis type: {analysis_type}")
    print("=" * 60)
    
    results = {}
    generated_files = []
    
    if analysis_type in ['individual', 'both']:
        print("\n1. GENERATING INDIVIDUAL CAMERA PLOTS...")
        success, files = test_all_camera_detection_ranges()
        results['individual_cameras'] = success
        if files:
            generated_files.extend(files)
            
        print(f"\nGenerated {len(files) if files else 0} individual camera plots")
    
    if analysis_type in ['representative', 'both']:
        print("\n2. GENERATING REPRESENTATIVE EXPLANATION...")
        success, filepath = test_representative_camera_explanation()
        results['representative'] = success
        if filepath:
            generated_files.append(filepath)
    
    # Also generate the original sensor coverage for comparison
    print("\n3. GENERATING ORIGINAL SENSOR COVERAGE...")
    success, filepath = test_sensor_coverage_visualization()
    results['original_coverage'] = success
    if filepath:
        generated_files.append(filepath)
    
    print("\n" + "=" * 60)
    print("DETECTION RANGE ANALYSIS SUMMARY")
    print("=" * 60)
    
    for analysis_name, success in results.items():
        status = "PASS" if success else "FAIL"
        print(f"{analysis_name}: {status}")
    
    if generated_files:
        print(f"\nGenerated {len(generated_files)} files in 'images' folder:")
        for filepath in generated_files:
            print(f"  - {os.path.basename(filepath)}")
        
        print("\nRECOMMENDATIONS:")
        if analysis_type == 'individual':
            print("• Use individual camera plots for detailed technical analysis")
            print("• Each camera shows its unique detection profile")
            print("• Good for troubleshooting specific camera performance")
        elif analysis_type == 'representative':
            print("• Use representative plot for audience education")
            print("• Explains detection range concept clearly")
            print("• Shows how environmental factors affect detection")
        else:
            print("• Individual plots: Technical/engineering audience")
            print("• Representative plot: General/executive audience")
            print("• Original coverage: System overview")
    
    return results

def run_enhanced_visualization_tests(num_drones=8, attack_pattern='coordinated', duration=15.0, drone_type='small'):
    """Run visualization tests with custom parameters - ORIGINAL FUNCTIONALITY PRESERVED"""
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
            print(f"  - {os.path.basename(filepath)}")
        print("\nVerify that axis labels are not cut off in these images")
    
    return test_results

def main():
    """Main function with enhanced command line argument parsing"""
    parser = argparse.ArgumentParser(description='Enhanced Visualization Test for Drone Detection System')
    
    # ORIGINAL ARGUMENTS - PRESERVED
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
    
    # NEW DETECTION RANGE ARGUMENTS
    parser.add_argument('--detection-analysis', '-da', type=str, 
                       choices=['individual', 'representative', 'both'], 
                       default=None,
                       help='Run detection range analysis (individual=8 separate plots, representative=single explanatory plot, both=all)')
    
    parser.add_argument('--detection-only', action='store_true',
                       help='Only run detection range analysis, skip trajectory visualization')
    
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
    
    # Run detection range analysis if requested
    if args.detection_analysis:
        run_detection_range_analysis(args.detection_analysis)
        if args.detection_only:
            return
    
    # Run regular tests if not detection-only
    if not args.detection_only:
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

"""
USAGE EXAMPLES - ALL ORIGINAL FUNCTIONALITY PRESERVED:

# Original trajectory visualizations still work exactly the same:
python test_visualization.py --drones 15 --pattern evasive --type small
python test_visualization.py --drones 25 --pattern swarm --type small
python test_visualization.py --drones 20 --pattern formation --type small
python test_visualization.py --drones 12 --pattern perimeter --type small
python test_visualization.py --list-patterns

# NEW: Detection range analysis options:
python test_visualization.py --detection-analysis individual
python test_visualization.py --detection-analysis representative
python test_visualization.py --detection-analysis both

# NEW: Combine trajectory + detection analysis:
python test_visualization.py --drones 15 --pattern evasive --detection-analysis representative

# NEW: Only detection analysis, skip trajectories:
python test_visualization.py --detection-analysis both --detection-only
"""