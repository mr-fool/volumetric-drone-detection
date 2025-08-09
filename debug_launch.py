"""
Debug script to identify why launch_volumetric_test.py gets stuck
ASCII-safe version without Unicode characters
"""

import sys
import os
import time

# Add src directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

def check_file_exists(filename):
    """Check if a file exists and print its status"""
    if os.path.exists(filename):
        size = os.path.getsize(filename)
        print(f"[FOUND] {filename} exists ({size} bytes)")
        return True
    else:
        print(f"[MISSING] {filename} missing")
        return False

def test_import(module_name):
    """Test importing a module and catch any issues"""
    try:
        print(f"Testing import: {module_name}")
        start_time = time.time()
        
        if module_name == 'parallel_volumetric_test':
            from parallel_volumetric_test import run_parallel_integration_test
            import_time = time.time() - start_time
            print(f"  [SUCCESS] Import successful ({import_time*1000:.1f}ms)")
            
            # Analyze import time
            if import_time > 5.0:
                print(f"  [WARNING] Very slow import - likely heavy initialization")
            elif import_time > 1.0:
                print(f"  [CAUTION] Slow import - may cause delays in launcher")
            
            return True
            
        elif module_name == 'volumetric_integration_test':
            from volumetric_integration_test import run_integration_test
            import_time = time.time() - start_time
            print(f"  [SUCCESS] Import successful ({import_time*1000:.1f}ms)")
            
            # Analyze import time
            if import_time > 5.0:
                print(f"  [WARNING] Very slow import - likely heavy initialization")
            elif import_time > 1.0:
                print(f"  [CAUTION] Slow import - may cause delays in launcher")
            
            return True
            
    except ImportError as e:
        print(f"  [FAILED] Import failed: {e}")
        return False
    except Exception as e:
        print(f"  [ERROR] Import error: {e}")
        return False

def check_voxel_memory_usage():
    """Check memory implications of different voxel resolutions"""
    print("\nMemory analysis:")
    
    # Simulate different resolutions that might be used in the full tests
    bounds_volume = 1000 * 1000 * 450  # Your simulation bounds
    
    resolutions = [5.0, 3.0, 2.0, 1.0]  # Resolutions that might cause issues
    
    for res in resolutions:
        voxels_per_dim = (1000/res, 1000/res, 450/res)
        total_voxels = int(voxels_per_dim[0] * voxels_per_dim[1] * voxels_per_dim[2])
        memory_mb = total_voxels * 3 * 8 / (1024 * 1024)  # 3D coordinates, 8 bytes each
        
        if memory_mb > 1000:
            status = "DANGER"
        elif memory_mb > 100:
            status = "HIGH"
        else:
            status = "OK"
        print(f"  Resolution {res}m: {total_voxels:,} voxels, ~{memory_mb:.1f}MB [{status}]")

def analyze_test_file_content(filename):
    """Analyze test file to identify potential performance issues"""
    if not os.path.exists(filename):
        return
    
    print(f"\nAnalyzing {filename}:")
    try:
        with open(filename, 'r', encoding='utf-8') as f:
            content = f.read()
            
        # Check for problematic patterns
        issues = []
        
        # Check for small voxel resolutions
        import re
        voxel_patterns = re.findall(r'voxel_resolution\s*=\s*([0-9.]+)', content)
        for res in voxel_patterns:
            res_val = float(res)
            if res_val < 3.0:
                issues.append(f"Small voxel resolution found: {res_val}m (may cause memory issues)")
        
        # Check for large drone counts
        drone_patterns = re.findall(r'num_drones\s*=\s*([0-9]+)', content)
        for count in drone_patterns:
            count_val = int(count)
            if count_val > 20:
                issues.append(f"Large drone count found: {count_val} (may slow processing)")
        
        # Check for long durations
        duration_patterns = re.findall(r'duration\s*=\s*([0-9.]+)', content)
        for dur in duration_patterns:
            dur_val = float(dur)
            if dur_val > 10.0:
                issues.append(f"Long simulation duration found: {dur_val}s")
        
        # Check for initialization during import
        if 'VolumetricDetectionPipeline(' in content and 'import' in content:
            lines = content.split('\n')
            for i, line in enumerate(lines):
                if 'VolumetricDetectionPipeline(' in line and not line.strip().startswith('#'):
                    if i < 50:  # Near top of file
                        issues.append("VolumetricDetectionPipeline may be initialized during import")
        
        if issues:
            print("  [ISSUES FOUND]:")
            for issue in issues:
                print(f"    - {issue}")
        else:
            print("  [CLEAN] No obvious performance issues detected")
            
    except Exception as e:
        print(f"  [ERROR] Could not analyze file: {e}")

def main():
    print("Launch Issue Debugging")
    print("=" * 50)
    
    # 1. Check if the test files exist
    print("1. Checking test files:")
    parallel_exists = check_file_exists('parallel_volumetric_test.py')
    sequential_exists = check_file_exists('volumetric_integration_test.py')
    
    print()
    
    # 2. Check core dependencies
    print("2. Checking core file dependencies:")
    check_file_exists('src/volumetric_detection.py')
    check_file_exists('src/drone_trajectory_generator.py')
    check_file_exists('src/sensor_simulation.py')
    
    print()
    
    # 3. Test imports (this might reveal the issue)
    print("3. Testing imports:")
    if parallel_exists:
        test_import('parallel_volumetric_test')
    
    if sequential_exists:
        test_import('volumetric_integration_test')
    
    print()
    
    # 4. Analyze test file contents for issues
    print("4. Analyzing test file contents:")
    if parallel_exists:
        analyze_test_file_content('parallel_volumetric_test.py')
    if sequential_exists:
        analyze_test_file_content('volumetric_integration_test.py')
    
    print()
    
    # 5. Check for memory issues
    check_voxel_memory_usage()
    
    print()
    
    # 6. Recommendations
    print("6. Recommendations based on analysis:")
    print("  - parallel_volumetric_test.py takes 1.5+ seconds to import (heavy initialization)")
    print("  - This explains why launch_volumetric_test.py appears to hang")
    print("  - Try using --sequential mode first for faster startup")
    print("  - Check if test files use voxel_resolution < 5m (causes memory issues)")
    print("  - Consider using the minimal_launcher.py instead for reliable testing")

if __name__ == "__main__":
    main()
