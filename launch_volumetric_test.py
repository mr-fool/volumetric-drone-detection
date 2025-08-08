"""
Safe launcher for volumetric detection test
Handles missing dependencies gracefully
"""

import sys
import os

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
    
    # Add src directory to path
    src_dir = os.path.join(os.path.dirname(__file__), 'src')
    if os.path.exists(src_dir):
        sys.path.insert(0, src_dir)
    
    try:
        # Import and run the test
        from volumetric_integration_test import run_integration_test
        
        print("Starting Volumetric Detection Pipeline Integration Test...")
        print("Academic validation for JDMS paper submission")
        print()
        
        # Run comprehensive test
        test_results = run_integration_test()
        
        print("\nIntegration test completed!")
        print("Check 'volumetric_detection_validation.json' for detailed results.")
        
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

if __name__ == "__main__":
    main()
