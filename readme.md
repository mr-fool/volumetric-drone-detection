# Volumetric Motion Detection for Multi-Drone Swarm Engagement

Simulation framework for evaluating multi-camera detection of drone swarms. It generates swarm
trajectories, simulates an array of passive optical cameras, runs three 3D detection methods
(multi-view triangulation, space carving, and a Bayesian occupancy grid) plus a hybrid of all
three, and scores every output against the true drone positions using Monte Carlo validation.

> ## Correction: ground-truth leakage fixed
>
> In the version of this code used for the JDMS paper, the simulated cameras passed each
> drone's **true 3D position** to the detection algorithms, and outputs were never scored
> against ground truth. As a result:
>
> - **Triangulation** averaged true positions over every cross-camera pairing, producing the
>   real drones plus phantom midpoints between them (about n + n(n-1)/2 outputs for n drones;
>   the paper's "61.8 targets").
> - **Space carving** deduplicated the true positions and returned them exactly (zero error;
>   the paper's "11.1 targets" is simply the mean number of drones in view).
> - **The occupancy grid** cast rays ending at the true positions, and was never run in the
>   reported experiments.
> - **Hybrid** contained no weighted fusion, and the reported "sensor noise 0.5-2.0x" setting did
>   not change the pixel noise.
> - The experiment was not repeatable: cameras drew noise from a shared generator across
>   threads, and the trajectory generator carried state between runs.
>
> The paper's processing-time, detection-count, confidence and environmental-correlation results
> therefore do not measure detection ability. The code now:
>
> - hands detectors only noisy pixel detections (shuffled, with nearby drones merged into one blob)
>   plus camera calibration; ground truth is kept separately for scoring only;
> - implements real cross-view association and least-squares triangulation, visual-hull space
>   carving, and a log-odds occupancy grid on shared camera geometry;
> - fuses the three in hybrid mode with explicit, unfitted default weights;
> - scores every frame against ground truth (precision, recall, F1, position error, false positives);
> - runs every method on identical observations, applies the noise multiplier to pixel noise, and
>   computes bootstrap confidence intervals over scenarios, with paired Wilcoxon tests;
> - derives all randomness from one master seed, so results are exactly reproducible;
> - includes tests that fail if ground truth reaches a detector or if results stop being reproducible.
>
> The original code is preserved in this repository's history. All results in this README come
> from the corrected code.

## Corrected Validation Results

50 Monte Carlo scenarios, master seed 42, 8-camera perimeter array. All methods run on identical
observations; outputs are matched to ground truth within max(10 m, 2 x voxel size). Values are
mean [95% bootstrap CI over scenarios].

Full data: `results/validation_50runs_corrected.json`. Report with paired Wilcoxon tests and LaTeX
tables: `results/statistical_analysis_report_corrected_50runs.txt`.

| Method | F1 | Precision | Recall | Position error (m) | False positives / frame | Time / frame (ms)* |
|---|---|---|---|---|---|---|
| Triangulation | 0.953 [0.930, 0.972] | 0.960 | 0.947 | 2.56 [2.26, 2.85] | 0.51 | 58 |
| Space carving | 0.866 [0.808, 0.913] | 0.864 | 0.869 | 3.67 [3.28, 4.11] | 1.87 | 941 |
| Occupancy grid | 0.688 [0.629, 0.745] | 0.680 | 0.699 | 6.29 [5.86, 6.71] | 4.51 | 972 |
| Hybrid | 0.933 [0.904, 0.958] | 0.922 | 0.951 | 2.57 [2.27, 2.89] | 1.21 | 1609 |

\* Processing time depends on hardware (measured on a 2-core cloud VM) and is not reproducible
by seed.

F1 by flight pattern:

| Pattern | n | Triangulation | Space carving | Occupancy grid | Hybrid |
|---|---|---|---|---|---|
| Coordinated attack | 14 | 1.00 | 0.99 | 0.86 | 1.00 |
| Random dispersal | 12 | 0.96 | 0.96 | 0.82 | 0.96 |
| Evasive maneuvers | 13 | 0.98 | 0.94 | 0.57 | 0.97 |
| Formation flying | 11 | 0.86 | 0.53 | 0.46 | 0.78 |

Robustness across master seeds (50 scenarios each; seeds chosen in advance):

| Master seed | Triangulation F1 | Space carving F1 | Occupancy grid F1 | Hybrid F1 | Triangulation error (m) | Hardest pattern | Triangulation vs hybrid F1 (Wilcoxon p) |
|---|---|---|---|---|---|---|---|
| 42 (main) | 0.953 | 0.866 | 0.688 | 0.933 | 2.56 | Formation flying | 2.0e-4 |
| 1 | 0.942 | 0.832 | 0.679 | 0.921 | 2.52 | Formation flying | 1.7e-4 |
| 2 | 0.939 | 0.830 | 0.638 | 0.912 | 2.67 | Formation flying | 3.8e-5 |
| 3 | 0.947 | 0.882 | 0.645 | 0.934 | 2.82 | Formation flying | 5.3e-4 |

The method ranking (triangulation > hybrid > space carving > occupancy grid), the hardest pattern,
and the triangulation-vs-hybrid difference are the same for every seed.

Key findings:
- Triangulation is the most accurate and the fastest. Hybrid fusion (default, unfitted weights)
  does not beat it on F1 (paired Wilcoxon p = 0.0002) because the voxel methods add false positives.
- Dense formation flying (about 25 m spacing) is the hardest case: voxel methods at 3-10 m
  resolution cannot separate ghost intersections from real drones (space carving F1 0.53).
- Pixel noise drives position error (Spearman r = 0.92 for triangulation). The visibility setting
  had no measurable effect on recall, because at the simulated visibility values (0.52-1.0) no
  drone falls below the sensor's detection threshold.
- The voxel methods take about 1-2 s per frame in Python, so they are not real-time as implemented.

## Simulation Setup

### Surveillance volume

```python
bounds = SimulationBounds(
    x_min=0.0, x_max=1000.0,   # 1 km x 1 km
    y_min=0.0, y_max=1000.0,
    z_min=50.0, z_max=500.0    # 50-500 m altitude
)
```

### Camera array (default: `"perimeter"`)

- 8 visible-spectrum optical cameras; no radar or other modalities were used in the results above
- 1920 x 1080 pixels, 60 deg (H) x 45 deg (V) field of view
- 50-2000 m detection range
- Circular perimeter at 600 m radius, 100 m elevation, 45 deg spacing, each facing the centre
- 2 px (1-sigma) base pixel noise, increased by weather and wind, times the scenario noise multiplier

Each camera reports only the pixel coordinates and a confidence score for each blob it detects.
Passive cameras do not measure range. Other array types (`"triangulation"`, `"mixed"`) exist in
`sensor_simulation.py` but were not used in the validation.

### Drones and flight patterns

- Drone types: `small` (0.8 m wingspan, 25 m/s max; used in validation), `medium` (1.5 m, 35 m/s)
- Patterns: `COORDINATED_ATTACK`, `FORMATION_FLYING`, `EVASIVE_MANEUVERS`, `RANDOM_DISPERSAL`,
  `PERIMETER_SWEEP` (the last is not included in the Monte Carlo validation)

### Monte Carlo scenarios

Each scenario randomises swarm size (5-20 drones), flight pattern, duration (3-8 s, 0.5 s
timestep, up to 8 frames scored), voxel resolution (3-10 m), visibility (0.52-1.0) and pixel-noise
multiplier (0.5-2.0x).

## Detection Methods

All methods receive the same input: noisy pixel detections plus each camera's calibration.

| Method | How it works | Strengths and weaknesses |
|---|---|---|
| `TRIANGULATION` | Intersects pixel rays for every camera pair, keeps hypotheses that reproject inside the noise gate, rejects those contradicted by other cameras, assigns each image blob to at most one target, then refines position by least squares on reprojection error | Most accurate; fastest; handles dense formations best |
| `SPACE_CARVING` | Keeps voxels whose projections land on a detection in every camera that can see them (at least 2), then extracts one target per group of image blobs | Hard consistency test: one missed detection removes a drone; limited by voxel size |
| `OCCUPANCY_GRID` | Per-voxel log-odds: detections along a ray raise occupancy, empty views lower it, evidence decays between frames | Tolerates a single missed detection, but evidence spreads along rays, so error and false positives are higher |
| `HYBRID` | Runs all three and matches their targets; confidence is a weighted sum (default weights 0.5 / 0.25 / 0.25, not tuned); position from triangulation when available | Not better than triangulation alone with the default weights |

If you tune the fusion weights, tune on one master seed and report on others, so the reported
results are not fitted to the test data.

### Voxel grid memory

The voxel methods store one float32 log-odds value per voxel (computed size, not measured process
memory):

| Resolution | Voxels | Grid memory |
|---|---|---|
| 10 m | 450,000 | 1.7 MB |
| 7 m | 1.33 M | 5.1 MB |
| 5 m | 3.6 M | 13.7 MB |
| 3 m | 16.7 M | 63.8 MB |
| 2 m | 56.3 M | 214.6 MB |

## Installation

Requires Python 3.11+ and NumPy 1.25 or newer (Windows, macOS or Linux).

```bash
git clone https://github.com/mr-fool/volumetric-drone-detection
cd volumetric-drone-detection
pip install -r requirements.txt
python tests/test_detection_integrity.py   # should print "all integrity checks passed"
```

## Usage

### Reproduce the validation results

```bash
# Main results: master seed 42 is the default (about 15 minutes with 2 workers)
python statistical_validation.py --runs 50 --workers 4

# Robustness checks: same experiment, different random sample
python statistical_validation.py --runs 50 --seed 1

# Report, significance tests and LaTeX tables
python statistical_analysis.py statistical_validation_results_<timestamp>.json -o report.txt

# Figures
python make_result_figures.py statistical_validation_results_<timestamp>.json results
```

The master seed is 42 unless you pass `--seed`; the same seed gives identical accuracy results for any `--workers` value. `run_statistical_validation.py`
is a convenience wrapper (`--quick` 10 runs, `--standard` 50, `--thorough` 100) that uses the
default seed 42.

### Quick checks

```bash
python quick_validation_test_optimized.py          # one frame through every method
python minimal_launcher.py --resolution 10 --drones 5 --duration 2
python safe_integration_test.py                    # several small configurations
python debug_test.py                               # step-by-step diagnostics
```

These print detections and timings. Use `statistical_validation.py` for accuracy, since it scores
outputs against ground truth.

### Use the pipeline in your own code

```python
import sys; sys.path.insert(0, "src")
from drone_trajectory_generator import DroneSwarmGenerator, FlightPattern, SimulationBounds
from sensor_simulation import create_standard_sensor_array
from volumetric_detection import VolumetricDetectionPipeline, DetectionMethod
from evaluation import score_frame

bounds = SimulationBounds()
traj = DroneSwarmGenerator(bounds).generate_swarm_trajectories(
    num_drones=10, pattern=FlightPattern.COORDINATED_ATTACK, duration=5.0, timestep=0.5)
cameras = create_standard_sensor_array(bounds, "perimeter")
truth = traj["trajectories"][:, 4, :]                  # true positions (for scoring only)
observations = cameras.observe_targets(truth, traj["times"][4])

pipeline = VolumetricDetectionPipeline(bounds, voxel_resolution=5.0)
targets = pipeline.detect(observations, traj["times"][4], DetectionMethod.TRIANGULATION)
print(score_frame(targets, truth, gate_m=10.0))        # precision, recall, F1, error...
```

### Visualizations

```bash
python test_visualization.py --drones 15 --pattern evasive --type small
python test_visualization.py --list-patterns
python test_visualization.py --detection-analysis individual --detection-only     # 8 camera plots + summary
python test_visualization.py --detection-analysis representative --detection-only # single explanation figure
python generate_memory_plot.py                                                    # voxel grid memory figure
```

`generate_all_figures.bat` runs the full set; images are written to `images/`. These figures show
trajectories and camera coverage; they are not detection results.

## Reproducibility

- `--seed` (default 42) sets the scenario list. Each scenario's seed is derived from it with
  `numpy.random.SeedSequence`, so results do not depend on worker count or completion order.
- Each camera draws noise from its own child generator, created in camera order before the camera
  threads start, so thread timing cannot change the noise.
- The trajectory generator resets its per-swarm state on every call.
- `tests/test_detection_integrity.py` checks that observations carry no ground truth, that every
  method clears a basic accuracy bar with non-zero error, and that the same seed gives identical results.

Processing times are the only outputs that vary between identical runs.

## Project Structure

```
volumetric-drone-detection/
├── readme.md
├── requirements.txt
├── src/
│   ├── drone_trajectory_generator.py   # Swarm trajectories and flight patterns
│   ├── sensor_simulation.py            # Camera array: noisy pixel detections only
│   ├── camera_geometry.py              # Projection, pixel rays, ray intersection, voxel tests
│   ├── multi_view_triangulator.py      # Cross-view association + least-squares triangulation
│   ├── space_carving.py                # Visual-hull space carving
│   ├── bayesian_occupancy_updater.py   # Log-odds occupancy grid
│   ├── volumetric_detection.py         # Pipeline and hybrid fusion
│   ├── detection_types.py              # Shared result types
│   ├── evaluation.py                   # Scoring against ground truth
│   ├── temporal_target_tracker.py      # Optional frame-to-frame tracking
│   └── performance_monitor.py
├── statistical_validation.py           # Monte Carlo validation (--seed, --runs, --workers)
├── statistical_analysis.py             # Report, LaTeX tables, paired Wilcoxon tests
├── run_statistical_validation.py       # Convenience wrapper
├── make_result_figures.py              # Result figures from a validation results file
├── results/                            # Corrected results, report and figures (seed 42)
├── tests/
│   └── test_detection_integrity.py
├── test_visualization.py               # Trajectory and camera-coverage figures
├── generate_memory_plot.py
├── generate_all_figures.bat
├── quick_validation_test_optimized.py
├── minimal_launcher.py
├── safe_integration_test.py
└── debug_test.py
```

## Limitations

- Idealised environment: no terrain, buildings, clutter or false-alarm sources; homogeneous
  atmosphere; visible-spectrum cameras only.
- The camera model rotates in yaw only and uses an angular projection, not a full pinhole model
  with lens distortion.
- Detection is single-frame; the optional tracker is not part of the validation.
- The visibility model never removes a detection at the simulated visibility values, so these
  results say nothing about performance in poor weather.
- Hybrid fusion weights and detector thresholds are untuned defaults.
- The voxel methods are not real-time as implemented in Python.
