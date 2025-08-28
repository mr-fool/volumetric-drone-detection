"""
Statistical Validation Framework for Volumetric Detection Research

Implements Monte Carlo validation, confidence interval calculations, and
comprehensive statistical analysis to support academic research claims.

This module provides the statistical foundation for the research paper's
quantitative claims about detection performance, processing speeds, and
algorithm comparisons.
"""

import sys
import os
import time
import json
import numpy as np
import scipy.stats as stats
from datetime import datetime
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass
from collections import defaultdict
import concurrent.futures
import warnings

# Add src directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from drone_trajectory_generator import DroneSwarmGenerator, FlightPattern, SimulationBounds
from sensor_simulation import create_standard_sensor_array
from volumetric_detection import VolumetricDetectionPipeline, DetectionMethod

@dataclass
class StatisticalResults:
    """Container for statistical analysis results"""
    mean: float
    std: float
    confidence_interval: Tuple[float, float]
    sample_size: int
    confidence_level: float
    method: str  # bootstrap, t_distribution, etc.

@dataclass 
class ValidationScenario:
    """Configuration for a validation scenario"""
    num_drones: int
    pattern: FlightPattern
    duration: float
    voxel_resolution: float
    environmental_factor: float  # 0.0 = perfect, 1.0 = degraded
    noise_level: float
    scenario_id: str

class StatisticalValidator:
    """
    Main class for conducting statistical validation of detection algorithms
    """
    
    def __init__(self, confidence_level: float = 0.95, num_monte_carlo_runs: int = 50):
        self.confidence_level = confidence_level
        self.num_runs = num_monte_carlo_runs
        self.results_history = []
        
        # Base simulation setup
        self.bounds = SimulationBounds()
        self.drone_generator = DroneSwarmGenerator(self.bounds)
        
        # Statistical data storage
        self.raw_data = defaultdict(list)
        self.processed_statistics = {}
        
    def calculate_confidence_interval(self, data: np.ndarray, 
                                    confidence_level: float = None,
                                    method: str = 'bootstrap') -> StatisticalResults:
        """
        Calculate confidence intervals using bootstrap or t-distribution
        """
        if confidence_level is None:
            confidence_level = self.confidence_level
            
        data_array = np.array(data)
        mean_val = np.mean(data_array)
        std_val = np.std(data_array, ddof=1)
        n = len(data_array)
        
        if method == 'bootstrap' and n >= 10:
            # Manual bootstrap implementation (compatible with all SciPy versions)
            rng = np.random.default_rng(42)  # Fixed seed for reproducibility
            bootstrap_samples = []
            n_bootstrap = 1000
            
            for _ in range(n_bootstrap):
                # Resample with replacement
                sample_indices = rng.choice(n, size=n, replace=True)
                bootstrap_sample = data_array[sample_indices]
                bootstrap_samples.append(np.mean(bootstrap_sample))
            
            bootstrap_samples = np.array(bootstrap_samples)
            alpha = 1 - confidence_level
            lower_percentile = (alpha/2) * 100
            upper_percentile = (1 - alpha/2) * 100
            
            ci_lower = np.percentile(bootstrap_samples, lower_percentile)
            ci_upper = np.percentile(bootstrap_samples, upper_percentile)
            
        else:
            # T-distribution confidence interval (fallback for small samples)
            alpha = 1 - confidence_level
            t_critical = stats.t.ppf(1 - alpha/2, df=n-1)
            margin_of_error = t_critical * (std_val / np.sqrt(n))
            ci_lower = mean_val - margin_of_error
            ci_upper = mean_val + margin_of_error
        
        return StatisticalResults(
            mean=mean_val,
            std=std_val,
            confidence_interval=(ci_lower, ci_upper),
            sample_size=n,
            confidence_level=confidence_level,
            method=method
        )
    
    def generate_randomized_scenarios(self) -> List[ValidationScenario]:
        """
        Generate randomized scenarios for Monte Carlo validation
        """
        scenarios = []
        
        # Base scenario parameters with randomization
        base_scenarios = [
            {'drones': 8, 'pattern': FlightPattern.COORDINATED_ATTACK, 'resolution': 5.0},
            {'drones': 12, 'pattern': FlightPattern.FORMATION_FLYING, 'resolution': 7.0},
            {'drones': 15, 'pattern': FlightPattern.EVASIVE_MANEUVERS, 'resolution': 5.0},
            {'drones': 10, 'pattern': FlightPattern.RANDOM_DISPERSAL, 'resolution': 6.0}
        ]
        
        np.random.seed(42)  # Fixed seed for reproducibility
        
        for run_id in range(self.num_runs):
            # Select base scenario randomly
            base = np.random.choice(base_scenarios)
            
            # Add randomization within bounds
            drone_variation = np.random.randint(-2, 3)  # ±2 drones
            num_drones = max(5, min(20, base['drones'] + drone_variation))
            
            # Duration randomization
            duration = np.random.uniform(3.0, 8.0)
            
            # Environmental factor (0=perfect, 1=degraded)
            environmental_factor = np.random.uniform(0.0, 0.8)
            
            # Sensor noise level
            noise_level = np.random.uniform(0.5, 2.0)
            
            # Resolution variation
            resolution_variation = np.random.uniform(-1.0, 1.0)
            voxel_resolution = max(3.0, min(10.0, base['resolution'] + resolution_variation))
            
            scenario = ValidationScenario(
                num_drones=num_drones,
                pattern=base['pattern'],
                duration=duration,
                voxel_resolution=voxel_resolution,
                environmental_factor=environmental_factor,
                noise_level=noise_level,
                scenario_id=f"run_{run_id:03d}"
            )
            scenarios.append(scenario)
        
        return scenarios
    
    def run_single_validation_scenario(self, scenario: ValidationScenario) -> Dict[str, Any]:
        """
        Run a single validation scenario and collect performance metrics
        """
        try:
            # Setup for this scenario
            sensor_array = create_standard_sensor_array(self.bounds, "perimeter")
            detection_pipeline = VolumetricDetectionPipeline(
                self.bounds, 
                voxel_resolution=scenario.voxel_resolution
            )
            
            # Apply environmental degradation
            sensor_array.weather_conditions['visibility'] = 1.0 - scenario.environmental_factor * 0.6
            sensor_array.weather_conditions['wind_speed'] = 5.0 + scenario.environmental_factor * 10.0
            
            # Generate trajectory data
            trajectory_data = self.drone_generator.generate_swarm_trajectories(
                num_drones=scenario.num_drones,
                pattern=scenario.pattern,
                duration=scenario.duration,
                timestep=0.5,
                drone_type='small'
            )
            
            trajectories = trajectory_data['trajectories']
            times = trajectory_data['times']
            
            # Test each detection method
            method_results = {}
            methods = [DetectionMethod.TRIANGULATION, DetectionMethod.SPACE_CARVING, DetectionMethod.HYBRID]
            
            for method in methods:
                method_data = {
                    'processing_times': [],
                    'detection_counts': [],
                    'confidence_scores': []
                }
                
                # Process multiple frames for statistical sampling
                num_frames = min(len(times), 8)  # Process up to 8 frames
                
                for frame_idx in range(num_frames):
                    drone_positions = trajectories[:, frame_idx, :]
                    timestamp = times[frame_idx]
                    
                    # Generate observations with noise
                    observations = sensor_array.observe_targets_parallel(drone_positions, timestamp)
                    
                    # Apply additional sensor noise
                    for obs in observations:
                        obs.sensor_noise_level *= scenario.noise_level
                        if len(obs.confidence_scores) > 0:
                            obs.confidence_scores *= np.random.uniform(0.8, 1.2, len(obs.confidence_scores))
                            obs.confidence_scores = np.clip(obs.confidence_scores, 0.1, 0.95)
                    
                    # Time the detection process
                    start_time = time.time()
                    
                    if method == DetectionMethod.SPACE_CARVING:
                        detected_targets = detection_pipeline._parallel_space_carving(
                            observations, timestamp, max_workers=8
                        )
                    else:
                        detected_targets = detection_pipeline.process_sensor_observations_parallel(
                            observations, timestamp, method
                        )
                    
                    processing_time = (time.time() - start_time) * 1000  # Convert to ms
                    
                    # Collect metrics
                    method_data['processing_times'].append(processing_time)
                    method_data['detection_counts'].append(len(detected_targets))
                    
                    # Collect confidence scores
                    if hasattr(detected_targets, '__iter__') and detected_targets:
                        confidences = [getattr(target, 'confidence', 0.5) for target in detected_targets]
                        method_data['confidence_scores'].extend(confidences)
                
                method_results[method.value] = method_data
            
            return {
                'scenario_id': scenario.scenario_id,
                'scenario_params': {
                    'num_drones': scenario.num_drones,
                    'pattern': scenario.pattern.value,
                    'duration': scenario.duration,
                    'voxel_resolution': scenario.voxel_resolution,
                    'environmental_factor': scenario.environmental_factor,
                    'noise_level': scenario.noise_level
                },
                'method_results': method_results,
                'success': True
            }
            
        except Exception as e:
            return {
                'scenario_id': scenario.scenario_id,
                'error': str(e),
                'success': False
            }
    
    def run_monte_carlo_validation(self, parallel: bool = True, max_workers: int = 4) -> Dict[str, Any]:
        """
        Run complete Monte Carlo validation with statistical analysis
        """
        print(f"Starting Monte Carlo validation with {self.num_runs} runs...")
        print(f"Confidence level: {self.confidence_level * 100}%")
        print(f"Parallel processing: {'Yes' if parallel else 'No'}")
        print("=" * 60)
        
        # Generate randomized scenarios
        scenarios = self.generate_randomized_scenarios()
        
        # Run scenarios
        if parallel and max_workers > 1:
            # Parallel execution
            results = []
            with concurrent.futures.ThreadPoolExecutor(max_workers=max_workers) as executor:
                future_to_scenario = {
                    executor.submit(self.run_single_validation_scenario, scenario): scenario 
                    for scenario in scenarios
                }
                
                completed = 0
                for future in concurrent.futures.as_completed(future_to_scenario):
                    result = future.result()
                    results.append(result)
                    completed += 1
                    if completed % 10 == 0 or completed == len(scenarios):
                        print(f"Completed {completed}/{len(scenarios)} scenarios...")
        else:
            # Sequential execution
            results = []
            for i, scenario in enumerate(scenarios):
                print(f"Processing scenario {i+1}/{len(scenarios)}: {scenario.scenario_id}")
                result = self.run_single_validation_scenario(scenario)
                results.append(result)
        
        # Process results and calculate statistics
        successful_results = [r for r in results if r.get('success', False)]
        
        print(f"\nSuccessful runs: {len(successful_results)}/{len(results)}")
        
        if len(successful_results) < 10:
            print("WARNING: Too few successful runs for reliable statistics")
        
        # Aggregate data by method
        method_statistics = self._calculate_method_statistics(successful_results)
        
        # Calculate correlation analysis
        correlation_results = self._calculate_environmental_correlations(successful_results)
        
        # Generate final validation report
        validation_report = {
            'timestamp': datetime.now().isoformat(),
            'configuration': {
                'num_runs': self.num_runs,
                'successful_runs': len(successful_results),
                'confidence_level': self.confidence_level,
                'parallel_execution': parallel
            },
            'method_statistics': method_statistics,
            'correlation_analysis': correlation_results,
            'raw_results': successful_results[:10]  # Include first 10 for reference
        }
        
        # Save results
        output_file = f'statistical_validation_results_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json'
        with open(output_file, 'w') as f:
            json.dump(validation_report, f, indent=2, default=str)
        
        print(f"\nValidation completed! Results saved to: {output_file}")
        
        # Print summary statistics
        self._print_validation_summary(method_statistics, correlation_results)
        
        return validation_report
    
    def _calculate_method_statistics(self, results: List[Dict]) -> Dict[str, Dict]:
        """
        Calculate comprehensive statistics for each detection method
        """
        # Aggregate all data by method
        method_data = defaultdict(lambda: {
            'processing_times': [],
            'detection_counts': [],
            'confidence_scores': []
        })
        
        for result in results:
            method_results = result.get('method_results', {})
            for method_name, method_metrics in method_results.items():
                method_data[method_name]['processing_times'].extend(
                    method_metrics.get('processing_times', [])
                )
                method_data[method_name]['detection_counts'].extend(
                    method_metrics.get('detection_counts', [])
                )
                method_data[method_name]['confidence_scores'].extend(
                    method_metrics.get('confidence_scores', [])
                )
        
        # Calculate statistics for each method
        statistics = {}
        
        for method_name, data in method_data.items():
            method_stats = {}
            
            # Processing time statistics
            if data['processing_times']:
                processing_stats = self.calculate_confidence_interval(
                    data['processing_times'], method='bootstrap'
                )
                method_stats['processing_time'] = {
                    'mean_ms': processing_stats.mean,
                    'std_ms': processing_stats.std,
                    'confidence_interval_ms': processing_stats.confidence_interval,
                    'sample_size': processing_stats.sample_size
                }
            
            # Detection count statistics
            if data['detection_counts']:
                detection_stats = self.calculate_confidence_interval(
                    data['detection_counts'], method='bootstrap'
                )
                method_stats['detection_count'] = {
                    'mean': detection_stats.mean,
                    'std': detection_stats.std,
                    'confidence_interval': detection_stats.confidence_interval,
                    'sample_size': detection_stats.sample_size
                }
            
            # Confidence score statistics
            if data['confidence_scores']:
                confidence_stats = self.calculate_confidence_interval(
                    data['confidence_scores'], method='bootstrap'
                )
                method_stats['confidence_score'] = {
                    'mean': confidence_stats.mean,
                    'std': confidence_stats.std,
                    'confidence_interval': confidence_stats.confidence_interval,
                    'sample_size': confidence_stats.sample_size
                }
            
            statistics[method_name] = method_stats
        
        return statistics
    
    def _calculate_environmental_correlations(self, results: List[Dict]) -> Dict[str, float]:
        """
        Calculate correlations between environmental factors and detection performance
        """
        correlations = {}
        
        # Collect environmental factors and performance metrics
        environmental_factors = []
        visibility_factors = []
        confidence_scores = []
        detection_rates = []
        
        for result in results:
            env_factor = result.get('scenario_params', {}).get('environmental_factor', 0)
            environmental_factors.append(env_factor)
            visibility_factors.append(1.0 - env_factor * 0.6)  # Convert to visibility
            
            # Aggregate performance across methods
            method_results = result.get('method_results', {})
            all_confidences = []
            total_detections = 0
            
            for method_data in method_results.values():
                all_confidences.extend(method_data.get('confidence_scores', []))
                total_detections += sum(method_data.get('detection_counts', []))
            
            avg_confidence = np.mean(all_confidences) if all_confidences else 0.0
            confidence_scores.append(avg_confidence)
            detection_rates.append(total_detections)
        
        # Calculate correlations
        if len(visibility_factors) > 3:
            # Visibility vs confidence correlation
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                visibility_confidence_corr, _ = stats.pearsonr(visibility_factors, confidence_scores)
                correlations['visibility_confidence'] = float(visibility_confidence_corr)
            
            # Environmental factor vs detection rate correlation
            if detection_rates:
                env_detection_corr, _ = stats.pearsonr(environmental_factors, detection_rates)
                correlations['environmental_detection_rate'] = float(env_detection_corr)
        
        return correlations
    
    def _print_validation_summary(self, method_statistics: Dict, correlations: Dict):
        """
        Print a summary of validation results in a format suitable for research papers
        """
        print("\n" + "=" * 80)
        print("STATISTICAL VALIDATION SUMMARY")
        print("=" * 80)
        
        print("\nProcessing Time Analysis (95% Confidence Intervals):")
        print("-" * 60)
        
        for method, stats in method_statistics.items():
            if 'processing_time' in stats:
                pt_stats = stats['processing_time']
                mean = pt_stats['mean_ms']
                ci_lower, ci_upper = pt_stats['confidence_interval_ms']
                n = pt_stats['sample_size']
                
                print(f"{method:15}: μ = {mean:5.1f} ± {(ci_upper-mean):4.1f} ms (95% CI, n={n})")
        
        print("\nDetection Count Analysis (95% Confidence Intervals):")
        print("-" * 60)
        
        for method, stats in method_statistics.items():
            if 'detection_count' in stats:
                dc_stats = stats['detection_count']
                mean = dc_stats['mean']
                ci_lower, ci_upper = dc_stats['confidence_interval']
                n = dc_stats['sample_size']
                
                print(f"{method:15}: μ = {mean:5.1f} ± {(ci_upper-mean):4.1f} targets (95% CI, n={n})")
        
        if correlations:
            print("\nEnvironmental Correlation Analysis:")
            print("-" * 60)
            for correlation_name, correlation_value in correlations.items():
                print(f"{correlation_name:25}: r = {correlation_value:6.3f}")
        
        print("\n" + "=" * 80)

def main():
    """
    Main function for running statistical validation
    """
    import argparse
    
    parser = argparse.ArgumentParser(description='Statistical Validation for Volumetric Detection')
    parser.add_argument('--runs', type=int, default=50,
                       help='Number of Monte Carlo runs (default: 50)')
    parser.add_argument('--confidence', type=float, default=0.95,
                       help='Confidence level (default: 0.95)')
    parser.add_argument('--workers', type=int, default=4,
                       help='Number of parallel workers (default: 4)')
    parser.add_argument('--sequential', action='store_true',
                       help='Run sequentially instead of in parallel')
    
    args = parser.parse_args()
    
    print("Statistical Validation Framework for Volumetric Detection")
    print("=" * 80)
    print(f"Configuration:")
    print(f"  Monte Carlo runs: {args.runs}")
    print(f"  Confidence level: {args.confidence * 100}%")
    print(f"  Parallel workers: {args.workers if not args.sequential else 'Sequential'}")
    print("=" * 80)
    
    # Initialize validator
    validator = StatisticalValidator(
        confidence_level=args.confidence,
        num_monte_carlo_runs=args.runs
    )
    
    # Run validation
    try:
        results = validator.run_monte_carlo_validation(
            parallel=not args.sequential,
            max_workers=args.workers
        )
        
        print("\nStatistical validation completed successfully!")
        
    except KeyboardInterrupt:
        print("\nValidation interrupted by user")
    except Exception as e:
        print(f"\nValidation failed with error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
