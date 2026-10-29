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
from evaluation import score_frame, observable_drones

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
    
    def __init__(self, confidence_level: float = 0.95, num_monte_carlo_runs: int = 50, seed: int = 42):
        self.confidence_level = confidence_level
        self.num_runs = num_monte_carlo_runs
        # Master seed: scenario list, per-scenario seeds and (via the sensor
        # array) per-camera noise streams are all derived from it.
        self.seed = seed
        self.results_history = []
        
        # Base simulation setup
        self.bounds = SimulationBounds()
        self.drone_generator = DroneSwarmGenerator(self.bounds)
        
        # Statistical data storage
        self.raw_data = defaultdict(list)
        self.processed_statistics = {}
    
    def measure_memory_scaling(self, test_resolutions: List[float] = None) -> Dict[float, Dict]:
        """
        Test memory usage across different voxel resolutions
        """
        if test_resolutions is None:
            test_resolutions = [2.0, 3.0, 5.0, 7.0, 10.0, 15.0]
        
        memory_results = {}
        
        for resolution in test_resolutions:
            try:
                # Calculate voxel count
                x_range = self.bounds.x_max - self.bounds.x_min
                y_range = self.bounds.y_max - self.bounds.y_min  
                z_range = self.bounds.z_max - self.bounds.z_min
                
                nx = int(np.ceil(x_range / resolution))
                ny = int(np.ceil(y_range / resolution))
                nz = int(np.ceil(z_range / resolution))
                total_voxels = nx * ny * nz
                
                # One float32 log-odds array per grid
                memory_mb = total_voxels * 4 / (1024 * 1024)
                
                # Test if resolution is practically feasible
                feasible = memory_mb < 500  # 500MB threshold
                
                memory_results[resolution] = {
                    'voxels': total_voxels,
                    'memory_mb': memory_mb,
                    'feasible': feasible
                }
                
            except Exception as e:
                memory_results[resolution] = {
                    'error': str(e),
                    'feasible': False
                }
        
        return memory_results

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
        
        np.random.seed(self.seed)  # master seed -> scenario list
        
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
    
    METHODS = [DetectionMethod.TRIANGULATION, DetectionMethod.SPACE_CARVING,
               DetectionMethod.OCCUPANCY_GRID, DetectionMethod.HYBRID]
    METRICS = ['precision', 'recall', 'recall_observable', 'f1', 'mean_error_m', 'fp']

    def run_single_validation_scenario(self, scenario: ValidationScenario, seed: int = None) -> Dict[str, Any]:
        """
        Run one scenario. Every method sees the *same* sensor observations in
        each frame, and every output is scored against the true drone positions.
        """
        try:
            if seed is not None:
                np.random.seed(seed)
            sensor_array = create_standard_sensor_array(self.bounds, "perimeter")
            sensor_array.weather_conditions['visibility'] = 1.0 - scenario.environmental_factor * 0.6
            sensor_array.weather_conditions['wind_speed'] = 5.0 + scenario.environmental_factor * 10.0
            sensor_array.noise_multiplier = scenario.noise_level  # actually scales pixel noise

            trajectory_data = self.drone_generator.generate_swarm_trajectories(
                num_drones=scenario.num_drones, pattern=scenario.pattern,
                duration=scenario.duration, timestep=0.5, drone_type='small')
            trajectories = trajectory_data['trajectories']
            times = trajectory_data['times']
            num_frames = min(len(times), 8)

            # Observations generated once per frame and shared by all methods
            frames = []
            for f in range(num_frames):
                truth = trajectories[:, f, :]
                obs = sensor_array.observe_targets_parallel(truth, times[f])
                frames.append((times[f], truth, obs, observable_drones(sensor_array.last_ground_truth)))

            gate = max(10.0, 2.0 * scenario.voxel_resolution)
            method_results = {}
            for method in self.METHODS:
                pipeline = VolumetricDetectionPipeline(self.bounds, voxel_resolution=scenario.voxel_resolution)
                data = defaultdict(list)
                for timestamp, truth, obs, observable in frames:
                    start = time.perf_counter()
                    targets = pipeline.detect(obs, timestamp, method)
                    data['processing_times'].append((time.perf_counter() - start) * 1000)
                    data['detection_counts'].append(len(targets))
                    data['confidence_scores'].extend(float(t.confidence) for t in targets)
                    sc = score_frame(targets, truth, gate_m=gate, observable=observable)
                    for m in self.METRICS:
                        data[m].append(sc[m])
                    data['n_true'].append(sc['n_true'])
                    data['n_observable'].append(sc['n_observable'])
                    data['position_errors_m'].extend(sc['errors_m'])
                method_results[method.value] = dict(data)

            return {
                'scenario_id': scenario.scenario_id,
                'scenario_params': {
                    'num_drones': scenario.num_drones,
                    'pattern': scenario.pattern.value,
                    'duration': scenario.duration,
                    'voxel_resolution': scenario.voxel_resolution,
                    'environmental_factor': scenario.environmental_factor,
                    'visibility': 1.0 - scenario.environmental_factor * 0.6,
                    'noise_level': scenario.noise_level,
                    'match_gate_m': gate,
                },
                'method_results': method_results,
                'success': True
            }
        except Exception as e:
            import traceback
            return {'scenario_id': scenario.scenario_id, 'error': f"{e}\n{traceback.format_exc()}",
                    'success': False}

    def run_monte_carlo_validation(self, parallel: bool = True, max_workers: int = 4) -> Dict[str, Any]:
        """Run the Monte Carlo validation (scenarios in separate processes, each seeded)."""
        print(f"Starting Monte Carlo validation with {self.num_runs} runs...")
        print(f"Confidence level: {self.confidence_level * 100}%")
        print(f"Parallel processing: {'Yes' if parallel else 'No'}")
        print("=" * 60)

        scenarios = self.generate_randomized_scenarios()
        # Independent per-scenario seeds derived from the master seed, so the
        # result does not depend on worker count or execution order.
        scenario_seeds = [int(ss.generate_state(1)[0]) for ss in np.random.SeedSequence(self.seed).spawn(len(scenarios))]
        jobs = list(zip(scenarios, scenario_seeds))
        results = []
        if parallel and max_workers > 1:
            with concurrent.futures.ProcessPoolExecutor(max_workers=max_workers) as ex:
                futs = [ex.submit(_run_scenario_job, self.confidence_level, s, seed) for s, seed in jobs]
                for done, fut in enumerate(concurrent.futures.as_completed(futs), 1):
                    results.append(fut.result())
                    if done % 10 == 0 or done == len(jobs):
                        print(f"Completed {done}/{len(jobs)} scenarios...")
        else:
            for i, (s, seed) in enumerate(jobs):
                print(f"Processing scenario {i+1}/{len(jobs)}: {s.scenario_id}")
                results.append(self.run_single_validation_scenario(s, seed))
        results.sort(key=lambda r: r['scenario_id'])

        successful_results = [r for r in results if r.get('success', False)]
        failed = [r for r in results if not r.get('success', False)]
        print(f"\nSuccessful runs: {len(successful_results)}/{len(results)}")
        for r in failed[:3]:
            print(f"  {r['scenario_id']} failed: {r.get('error', '')[:300]}")
        if len(successful_results) < 10:
            print("WARNING: Too few successful runs for reliable statistics")

        method_statistics = self._calculate_method_statistics(successful_results)
        correlation_results = self._calculate_environmental_correlations(successful_results)

        validation_report = {
            'timestamp': datetime.now().isoformat(),
            'configuration': {
                'num_runs': self.num_runs,
                'successful_runs': len(successful_results),
                'confidence_level': self.confidence_level,
                'parallel_execution': parallel,
                'master_seed': self.seed,
                'ci_method': 'bootstrap over per-scenario means (scenario = independent unit)',
            },
            'method_statistics': method_statistics,
            'correlation_analysis': correlation_results,
            'raw_results': successful_results,
        }
        output_file = f'statistical_validation_results_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json'
        with open(output_file, 'w') as f:
            json.dump(validation_report, f, indent=2, default=str)
        print(f"\nValidation completed! Results saved to: {output_file}")
        self._print_validation_summary(method_statistics, correlation_results)
        return validation_report

    def _calculate_method_statistics(self, results: List[Dict]) -> Dict[str, Dict]:
        """
        Frames within a scenario are correlated, so each scenario contributes one
        mean per metric and bootstrap CIs are computed over scenarios
        (sample_size = number of scenarios). Position error is also reported
        pooled over all matched detections for its distribution.
        """
        per_method = defaultdict(lambda: defaultdict(list))
        pooled_err = defaultdict(list)
        for r in results:
            for name, d in r.get('method_results', {}).items():
                for key in ['processing_times', 'detection_counts'] + self.METRICS:
                    vals = [v for v in d.get(key, []) if v is not None and not (isinstance(v, float) and np.isnan(v))]
                    if vals:
                        per_method[name][key].append(float(np.mean(vals)))
                pooled_err[name].extend(d.get('position_errors_m', []))

        statistics = {}
        for name, d in per_method.items():
            st = {}
            if d['processing_times']:
                ci = self.calculate_confidence_interval(d['processing_times'], method='bootstrap')
                st['processing_time'] = {'mean_ms': ci.mean, 'std_ms': ci.std,
                                         'confidence_interval_ms': ci.confidence_interval,
                                         'sample_size': ci.sample_size}
            for key, out in [('detection_counts', 'detection_count')] + [(m, m) for m in self.METRICS]:
                if d[key]:
                    ci = self.calculate_confidence_interval(d[key], method='bootstrap')
                    st[out] = {'mean': ci.mean, 'std': ci.std,
                               'confidence_interval': ci.confidence_interval,
                               'sample_size': ci.sample_size}
            if pooled_err[name]:
                e = np.array(pooled_err[name])
                st['position_error_distribution_m'] = {
                    'median': float(np.median(e)), 'p90': float(np.percentile(e, 90)),
                    'rmse': float(np.sqrt(np.mean(e ** 2))), 'n_matches': int(len(e))}
            statistics[name] = st
        return statistics

    def _calculate_environmental_correlations(self, results: List[Dict]) -> Dict[str, float]:
        """Per method: correlation of scenario visibility and noise with recall and position error."""
        out = {}
        vis = np.array([r['scenario_params']['visibility'] for r in results])
        noise = np.array([r['scenario_params']['noise_level'] for r in results])
        if len(results) < 4:
            return out
        for name in results[0]['method_results']:
            for metric in ['recall', 'mean_error_m']:
                y = np.array([np.nanmean(r['method_results'][name][metric]) if r['method_results'][name][metric] else np.nan
                              for r in results])
                for xname, x in [('visibility', vis), ('noise', noise)]:
                    ok = np.isfinite(y)
                    if ok.sum() > 3 and np.std(y[ok]) > 0:
                        rho, p = stats.spearmanr(x[ok], y[ok])
                        out[f'{name}:{xname}_vs_{metric}'] = {'spearman_r': float(rho), 'p_value': float(p)}
        return out

    def _print_validation_summary(self, method_statistics: Dict, correlations: Dict):
        print("\n" + "=" * 80)
        print("STATISTICAL VALIDATION SUMMARY (mean [95% CI], CIs over scenarios)")
        print("=" * 80)
        rows = [('precision', 'Precision', '{:.3f}'), ('recall', 'Recall (all drones)', '{:.3f}'),
                ('recall_observable', 'Recall (seen by >=2 cams)', '{:.3f}'), ('f1', 'F1', '{:.3f}'),
                ('mean_error_m', 'Position error (m)', '{:.2f}'), ('fp', 'False positives/frame', '{:.2f}'),
                ('detection_count', 'Detections/frame', '{:.1f}')]
        for name, st in method_statistics.items():
            print(f"\n{name}")
            if 'processing_time' in st:
                pt = st['processing_time']; lo, hi = pt['confidence_interval_ms']
                print(f"  {'Processing time (ms)':28}: {pt['mean_ms']:.1f} [{lo:.1f}, {hi:.1f}]  n={pt['sample_size']}")
            for key, label, fmt in rows:
                if key in st:
                    m = st[key]; lo, hi = m['confidence_interval']
                    print(f"  {label:28}: {fmt.format(m['mean'])} [{fmt.format(lo)}, {fmt.format(hi)}]")
        if correlations:
            print("\nEnvironmental sensitivity (Spearman):")
            for k, v in correlations.items():
                print(f"  {k:55}: r = {v['spearman_r']:6.3f} (p = {v['p_value']:.3g})")
        print("\n" + "=" * 80)


def _run_scenario_job(confidence_level, scenario, seed):
    """Process-pool entry point (must be module level to be picklable)."""
    v = StatisticalValidator(confidence_level=confidence_level, num_monte_carlo_runs=1)
    return v.run_single_validation_scenario(scenario, seed)


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
    parser.add_argument('--seed', type=int, default=42,
                       help='Master random seed (default: 42). Use other values for robustness checks.')
    parser.add_argument('--sequential', action='store_true',
                       help='Run sequentially instead of in parallel')
    
    args = parser.parse_args()
    
    print("Statistical Validation Framework for Volumetric Detection")
    print("=" * 80)
    print(f"Configuration:")
    print(f"  Monte Carlo runs: {args.runs}")
    print(f"  Master seed: {args.seed}")
    print(f"  Confidence level: {args.confidence * 100}%")
    print(f"  Parallel workers: {args.workers if not args.sequential else 'Sequential'}")
    print("=" * 80)
    
    # Initialize validator
    validator = StatisticalValidator(
        confidence_level=args.confidence,
        num_monte_carlo_runs=args.runs,
        seed=args.seed
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
