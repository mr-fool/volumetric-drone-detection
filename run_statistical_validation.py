"""
Statistical Validation Launcher

Complete workflow for running statistical validation and generating 
academic-ready results for the volumetric detection research paper.

Usage:
    python run_statistical_validation.py --quick     # 10 runs for testing
    python run_statistical_validation.py --standard  # 50 runs (paper quality)
    python run_statistical_validation.py --thorough  # 100 runs (comprehensive)
    python run_statistical_validation.py --custom --runs 25 --workers 8
"""

import os
import sys
import argparse
import time
from datetime import datetime

def main():
    parser = argparse.ArgumentParser(description='Run Statistical Validation for Academic Paper')
    
    # Predefined configurations
    parser.add_argument('--quick', action='store_true',
                       help='Quick test: 10 runs, 2 workers (for testing)')
    parser.add_argument('--standard', action='store_true',
                       help='Standard validation: 50 runs, 4 workers (paper quality)')
    parser.add_argument('--thorough', action='store_true',
                       help='Thorough validation: 100 runs, 6 workers (comprehensive)')
    
    # Custom configuration
    parser.add_argument('--custom', action='store_true',
                       help='Use custom parameters (specify with --runs and --workers)')
    parser.add_argument('--runs', type=int, default=50,
                       help='Number of Monte Carlo runs (default: 50)')
    parser.add_argument('--workers', type=int, default=4,
                       help='Number of parallel workers (default: 4)')
    parser.add_argument('--confidence', type=float, default=0.95,
                       help='Confidence level (default: 0.95)')
    
    # Processing options
    parser.add_argument('--sequential', action='store_true',
                       help='Run sequentially instead of in parallel')
    parser.add_argument('--analyze-only', type=str, metavar='RESULTS_FILE',
                       help='Skip validation, only analyze existing results file')
    parser.add_argument('--no-analysis', action='store_true',
                       help='Skip analysis step after validation')
    parser.add_argument('--plots', action='store_true',
                       help='Generate visualization plots')
    
    args = parser.parse_args()
    
    # Determine configuration
    if args.quick:
        num_runs, num_workers = 10, 2
        config_name = "Quick Test"
    elif args.standard:
        num_runs, num_workers = 50, 4
        config_name = "Standard Validation"
    elif args.thorough:
        num_runs, num_workers = 100, 6
        config_name = "Thorough Validation"
    elif args.custom:
        num_runs, num_workers = args.runs, args.workers
        config_name = f"Custom ({num_runs} runs)"
    else:
        # Default to standard
        num_runs, num_workers = 50, 4
        config_name = "Standard Validation (default)"
    
    print("Statistical Validation Launcher")
    print("=" * 60)
    print(f"Configuration: {config_name}")
    print(f"Monte Carlo runs: {num_runs}")
    print(f"Parallel workers: {num_workers if not args.sequential else 'Sequential'}")
    print(f"Confidence level: {args.confidence * 100}%")
    print("=" * 60)
    
    # Check if we're only analyzing existing results
    if args.analyze_only:
        print("Analyzing existing results...")
        return analyze_results(args.analyze_only, args.plots)
    
    # Estimate runtime
    estimated_minutes = estimate_runtime(num_runs, num_workers, args.sequential)
    print(f"Estimated runtime: {estimated_minutes:.1f} minutes")
    
    # Confirm before long runs
    if num_runs >= 50:
        response = input("Continue with validation? (y/N): ")
        if response.lower() != 'y':
            print("Validation cancelled.")
            return
    
    print("\nStarting statistical validation...")
    start_time = time.time()
    
    # Run validation
    try:
        validation_command = build_validation_command(
            num_runs, args.confidence, num_workers, args.sequential
        )
        
        print(f"Executing: {validation_command}")
        exit_code = os.system(validation_command)
        
        if exit_code != 0:
            print("ERROR: Statistical validation failed!")
            return exit_code
        
        elapsed_time = (time.time() - start_time) / 60
        print(f"\nValidation completed in {elapsed_time:.1f} minutes")
        
        # Find the results file (most recent)
        results_file = find_most_recent_results_file()
        
        if results_file and not args.no_analysis:
            print(f"\nAnalyzing results from: {results_file}")
            analyze_results(results_file, args.plots)
        else:
            print(f"Results saved. Run analysis with:")
            print(f"python statistical_analysis.py {results_file}")
    
    except KeyboardInterrupt:
        print("\nValidation interrupted by user")
        return 1
    except Exception as e:
        print(f"ERROR: {e}")
        return 1

def estimate_runtime(num_runs: int, num_workers: int, sequential: bool) -> float:
    """Estimate runtime in minutes"""
    # Base time per run (empirically determined)
    base_time_per_run = 0.8  # minutes
    
    if sequential:
        return num_runs * base_time_per_run
    else:
        # Parallel execution with some overhead
        parallel_efficiency = 0.75
        return (num_runs * base_time_per_run) / (num_workers * parallel_efficiency)

def build_validation_command(num_runs: int, confidence: float, 
                           num_workers: int, sequential: bool) -> str:
    """Build the command to run statistical validation"""
    cmd_parts = ["python", "statistical_validation.py"]
    cmd_parts.extend(["--runs", str(num_runs)])
    cmd_parts.extend(["--confidence", str(confidence)])
    
    if sequential:
        cmd_parts.append("--sequential")
    else:
        cmd_parts.extend(["--workers", str(num_workers)])
    
    return " ".join(cmd_parts)

def find_most_recent_results_file() -> str:
    """Find the most recent statistical validation results file"""
    import glob
    
    pattern = "statistical_validation_results_*.json"
    results_files = glob.glob(pattern)
    
    if not results_files:
        return None
    
    # Return most recent file
    return max(results_files, key=os.path.getctime)

def analyze_results(results_file: str, generate_plots: bool = False) -> int:
    """Analyze statistical results and generate reports"""
    if not os.path.exists(results_file):
        print(f"ERROR: Results file '{results_file}' not found")
        return 1
    
    try:
        # Build analysis command
        analysis_cmd = ["python", "statistical_analysis.py", results_file]
        
        # Generate timestamped output file
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_file = f"statistical_analysis_report_{timestamp}.txt"
        analysis_cmd.extend(["--output", output_file])
        
        if generate_plots:
            analysis_cmd.append("--plots")
        
        cmd_string = " ".join(analysis_cmd)
        print(f"Running analysis: {cmd_string}")
        
        exit_code = os.system(cmd_string)
        
        if exit_code == 0:
            print(f"\nAnalysis completed successfully!")
            print(f"Report saved to: {output_file}")
            
            if generate_plots:
                print("Visualization plots saved to: statistical_plots/")
            
            print("\nFor LaTeX tables only, run:")
            print(f"python statistical_analysis.py {results_file} --latex-only")
        else:
            print("ERROR: Analysis failed!")
            return exit_code
    
    except Exception as e:
        print(f"ERROR in analysis: {e}")
        return 1
    
    return 0

def check_dependencies():
    """Check if required files and dependencies are available"""
    required_files = [
        "statistical_validation.py",
        "statistical_analysis.py",
        "src/drone_trajectory_generator.py",
        "src/sensor_simulation.py", 
        "src/volumetric_detection.py"
    ]
    
    missing_files = []
    for file_path in required_files:
        if not os.path.exists(file_path):
            missing_files.append(file_path)
    
    if missing_files:
        print("ERROR: Missing required files:")
        for file_path in missing_files:
            print(f"  - {file_path}")
        return False
    
    # Check Python packages
    try:
        import numpy
        import scipy
        import matplotlib
        print("Dependencies check: OK")
        return True
    except ImportError as e:
        print(f"ERROR: Missing Python package: {e}")
        print("Install with: pip install numpy scipy matplotlib pandas seaborn")
        return False

if __name__ == "__main__":
    print("Statistical Validation Framework")
    print("For Volumetric Detection Research Paper")
    print()
    
    # Check dependencies first
    if not check_dependencies():
        sys.exit(1)
    
    exit_code = main()
    sys.exit(exit_code or 0)