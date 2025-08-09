"""
Safe launcher for volumetric detection test
Handles missing dependencies gracefully and provides dual output
Supports both parallel and sequential processing modes
"""

import sys
import os
import argparse

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

def check_dependencies():
    """Check for required dependencies"""
    missing = []
    
    try:
        import numpy
    except ImportError:
        missing.append('numpy')
    
    try:
        import matplotlib
    except ImportError:
        missing.append('matplotlib')
    
    try:
        import scipy
    except ImportError:
        missing.append('scipy')
    
    if missing:
        print("Missing required dependencies:")
        for dep in missing:
            print(f"  - {dep}")
        print("\nInstall with: pip install " + " ".join(missing))
        return False
    
    return True

def main():
    """Main launcher function"""
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Volumetric Detection Pipeline Test Launcher')
    parser.add_argument('--parallel', action='store_true', 
                       help='Use parallel processing for faster validation')
    parser.add_argument('--sequential', action='store_true',
                       help='Use sequential processing (original)')
    args = parser.parse_args()
    
    # Default to parallel if no specific mode chosen
    use_parallel = args.parallel or not args.sequential
    
    if not check_dependencies():
        sys.exit(1)
    
    # Setup dual output to both console and file
    output_filename = 'validation_results_parallel.txt' if use_parallel else 'validation_results_sequential.txt'
    tee_output = TeeOutput(output_filename)
    original_stdout = sys.stdout
    sys.stdout = tee_output
    
    try:
        # Add src directory to path
        src_dir = os.path.join(os.path.dirname(__file__), 'src')
        if os.path.exists(src_dir):
            sys.path.insert(0, src_dir)
        
        # Import and run the appropriate test
        if use_parallel:
            print("Starting Parallel Volumetric Detection Pipeline Integration Test...")
            print("Research validation framework with multiprocessing acceleration")
            from parallel_volumetric_test import run_parallel_integration_test
            test_runner = run_parallel_integration_test
        else:
            print("Starting Sequential Volumetric Detection Pipeline Integration Test...")
            print("Research validation framework for multi-drone swarm detection")
            from volumetric_integration_test import run_integration_test
            test_runner = run_integration_test
        
        print(f"Results will be saved to: {output_filename}")
        print()
        
        # Run test
        test_results = test_runner()
        
        print("\nIntegration test completed!")
        print("Check 'volumetric_detection_validation.json' for detailed results.")
        print(f"Console output saved to: {output_filename}")
        
        return test_results
        
    except ImportError as e:
        print(f"Import error: {e}")
        print("Make sure all required files are in the correct locations:")
        print("  - volumetric_detection.py should be in src/ directory")
        print("  - drone_trajectory_generator.py should be in src/ directory")
        print("  - sensor_simulation.py should be in src/ directory")
        if use_parallel:
            print("  - parallel_volumetric_test.py should be in root directory")
        else:
            print("  - volumetric_integration_test.py should be in root directory")
        sys.exit(1)
    
    except Exception as e:
        print(f"Error during test execution: {e}")
        sys.exit(1)
    
    finally:
        # Restore original stdout and close file
        sys.stdout = original_stdout
        tee_output.close()

if __name__ == "__main__":
    main()
