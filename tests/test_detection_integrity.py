"""
Integrity checks for the detection pipeline.

1. Observations handed to detectors carry no ground truth.
2. Every method is scored against the true drone positions and must clear a
   basic accuracy bar on an easy scenario.
3. Position errors are non-zero: a detector that reproduces the truth exactly
   from noisy pixels is reading ground truth somewhere.

Run:  python -m pytest tests/   (or)   python tests/test_detection_integrity.py
"""

import io
import contextlib
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from drone_trajectory_generator import DroneSwarmGenerator, FlightPattern, SimulationBounds
from sensor_simulation import create_standard_sensor_array
from volumetric_detection import VolumetricDetectionPipeline, DetectionMethod
from evaluation import score_frame

FORBIDDEN_KEYS = {'world_position', 'drone_id', 'estimated_size', 'detection_quality'}


def _frame(n=10, pattern=FlightPattern.COORDINATED_ATTACK, seed=0):
    np.random.seed(seed)
    b = SimulationBounds()
    with contextlib.redirect_stdout(io.StringIO()):
        tr = DroneSwarmGenerator(b).generate_swarm_trajectories(
            num_drones=n, pattern=pattern, duration=5.0, timestep=0.5, drone_type='small')
        arr = create_standard_sensor_array(b, "perimeter")
    truth = tr['trajectories'][:, 4, :]
    return b, truth, arr.observe_targets(truth, 2.0)


def test_observations_carry_no_ground_truth():
    _, _, obs = _frame()
    for o in obs:
        for d in o.detected_objects:
            assert not FORBIDDEN_KEYS & set(d), f"ground truth leaked: {FORBIDDEN_KEYS & set(d)}"
        assert np.all(np.isnan(o.range_estimates)), "passive camera should not report range"


def test_methods_are_accurate_but_not_perfect():
    b, truth, obs = _frame()
    bars = {DetectionMethod.TRIANGULATION: 0.9, DetectionMethod.SPACE_CARVING: 0.8,
            DetectionMethod.OCCUPANCY_GRID: 0.6, DetectionMethod.HYBRID: 0.9}
    for method, min_f1 in bars.items():
        targets = VolumetricDetectionPipeline(b, voxel_resolution=5.0).detect(obs, 2.0, method)
        s = score_frame(targets, truth, gate_m=10.0)
        assert s['f1'] >= min_f1, f"{method.value}: F1 {s['f1']:.2f} < {min_f1}"
        assert s['mean_error_m'] > 0.05, f"{method.value}: zero error suggests ground-truth leakage"


def test_validation_is_reproducible():
    """Same scenario + seed -> identical observations and scores, even with threaded cameras."""
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
    from statistical_validation import StatisticalValidator
    v = StatisticalValidator(num_monte_carlo_runs=4)
    sc = v.generate_randomized_scenarios()
    for s in sc[:2]:
        s.duration = 2.0  # keep the test quick
        runs = []
        for _ in range(2):
            with contextlib.redirect_stdout(io.StringIO()):
                r = v.run_single_validation_scenario(s, seed=7)
            assert r['success'], r.get('error')
            runs.append({m: (d['detection_counts'], d['mean_error_m'], d['fp'])
                         for m, d in r['method_results'].items()})
        assert runs[0] == runs[1], f"{s.scenario_id} not reproducible"


if __name__ == '__main__':
    test_observations_carry_no_ground_truth()
    test_methods_are_accurate_but_not_perfect()
    test_validation_is_reproducible()
    print("all integrity checks passed")
