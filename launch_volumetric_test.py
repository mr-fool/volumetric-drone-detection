"""
Safe launcher for volumetric detection test
Handles missing dependencies gracefully and provides dual output
"""

import sys
import os

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
    
    if not check_dependencies():
        sys.exit(1)
    
    # Setup dual output to both console and file
    output_filename = 'validation_results_fixed.txt'
    tee_output = TeeOutput(output_filename)
    original_stdout = sys.stdout
    sys.stdout = tee_output
    
    try:
        # Add src directory to path
        src_dir = os.path.join(os.path.dirname(__file__), 'src')
        if os.path.exists(src_dir):
            sys.path.insert(0, src_dir)
        
        # Import and run the test
        from volumetric_integration_test import run_integration_test
        
        print("Starting Volumetric Detection Pipeline Integration Test...")
        print("Academic validation for JDMS paper submission")
        print(f"Results will be saved to: {output_filename}")
        print()
        
        # Run comprehensive test
        test_results = run_integration_test()
        
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
