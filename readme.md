# Volumetric Motion Detection for Multi-Drone Swarm Engagement

A theoretical framework and simulation system for real-time volumetric detection and kinetic engagement coordination of drone swarms. This research implements novel integration algorithms between 3D motion detection and engagement systems for counter-UAS applications.

## Overview

This project develops a comprehensive simulation framework that combines:
- Drone trajectory generation with realistic flight patterns
- Multi-camera sensor arrays with environmental limitations
- Volumetric motion detection using probabilistic occupancy grids
- Real-time 3D target detection and tracking algorithms

## Current Status

- **Completed**: Drone trajectory generator (150 drones/second performance)
- **Completed**: Multi-camera sensor simulation with realistic limitations  
- **Completed**: Volumetric detection pipeline with hybrid algorithms
- **Completed**: Integration testing framework with research validation
- **Completed**: Parallel processing optimization (10-12x speedup)
- **Completed**: Comprehensive algorithm comparison and benchmarking

## Features

### Drone Trajectory Generation
- Multiple flight patterns (coordinated attack, formation flying, evasive maneuvers)
- Realistic physics constraints (velocity, acceleration limits)
- Environmental effects (wind, sensor noise)
- Scalable from 5-50+ drones per scenario

### Virtual Sensor Array Simulation
- Multi-camera configurations (perimeter, triangulation arrays)
- Realistic sensor limitations (range, field of view, noise)
- Environmental effects (weather, visibility degradation)
- Multiple sensor types (visible spectrum, thermal infrared)

### Volumetric Detection Pipeline
- Probabilistic occupancy grids with Bayesian updates
- Space carving algorithms for 3D reconstruction
- Multi-view geometry triangulation
- Hybrid detection methods combining multiple approaches
- Temporal target tracking with velocity estimation

### Parallel Processing Framework
- Multi-core CPU utilization (up to 11 workers)
- Concurrent algorithm testing and comparison
- Parallel scenario validation
- 10-12x speedup over sequential processing

### Performance Analysis
- Real-time computational performance metrics
- Statistical validation frameworks
- Visualization tools for trajectory and sensor data
- Comprehensive testing and benchmarking

## Installation

### Prerequisites
- Python 3.11+ 
- Windows 10/11, macOS, or Linux
- 8GB+ RAM recommended (32GB for large scenarios)
- Multi-core CPU recommended for parallel processing

### Setup

1. **Clone the repository:**
```bash
git clone https://github.com/yourusername/volumetric-drone-detection.git
cd volumetric-drone-detection
```

2. **Install dependencies:**
```bash
pip install -r requirements.txt
```

3. **Verify installation:**
```bash
python quick_validation_test.py
```

## Usage Guide

### Quick Testing

#### Basic Functionality Test
```bash
# Test core volumetric detection functionality
python quick_validation_test.py
```

#### Comprehensive Validation (Recommended)
```bash
# Run parallel research validation framework (fast)
python launch_volumetric_test.py --parallel

# Run sequential validation framework (slower)
python launch_volumetric_test.py --sequential
```

### Legacy Component Tests

#### Trajectory Generation Tests
```bash
# Test all trajectory generation functionality
python test_trajectory_generator.py > test_results.txt 2>&1
```

#### Visualization and Sensor Tests
```bash
# Test visualization fixes and sensor simulation
python test_visualization_fix.py > visualization_results.txt 2>&1
```

### Performance Benchmarking

The system has been tested and validated on desktop hardware:
- **CPU**: AMD Ryzen 5 5600 (12 cores)
- **RAM**: 32GB DDR4
- **GPU**: RTX 3060 12GB (optional)
- **OS**: Windows 11

**Current Performance (Parallel Processing):**
- **Trajectory Generation**: ~150 drones/second
- **Detection Processing**: 0.4-0.6 Hz for 30-50 drones
- **Validation Time**: 3.6 minutes (vs 35-45 minutes sequential)
- **CPU Utilization**: 11 parallel workers
- **Memory Usage**: ~330MB for volumetric grids

**Detection Accuracy Results:**
- **Formation Flying**: 43% detection rate, 26m average error
- **Evasive Maneuvers**: 14% detection rate, 23m average error
- **Random Dispersal**: 1.2% detection rate, 0.9m average error
- **Triangulation Method**: 1.2% overall detection rate, 11m average error

### Example Scenarios

#### Quick Detection Test
```bash
python -c "
from src.drone_trajectory_generator import *
from src.sensor_simulation import *
from src.volumetric_detection import *

bounds = SimulationBounds()
generator = DroneSwarmGenerator(bounds)
sensor_array = create_standard_sensor_array(bounds, 'perimeter')
detection_pipeline = VolumetricDetectionPipeline(bounds)

print('Volumetric detection system ready!')
"
```

#### Coordinated Swarm Attack Simulation
```bash
python -c "
from src.drone_trajectory_generator import *
bounds = SimulationBounds()
gen = DroneSwarmGenerator(bounds)
data = gen.generate_swarm_trajectories(15, FlightPattern.COORDINATED_ATTACK, 45.0)
fig = visualize_trajectories(data)
fig.savefig('coordinated_attack_demo.png', dpi=150, bbox_inches='tight', pad_inches=0.2)
print('Demo visualization saved as coordinated_attack_demo.png')
"
```

#### Multi-Sensor Detection Analysis
```bash
python -c "
from src.sensor_simulation import *
from src.drone_trajectory_generator import *
bounds = SimulationBounds()
sensors = create_standard_sensor_array(bounds, 'triangulation')
fig = sensors.visualize_sensor_coverage()
fig.savefig('sensor_coverage_demo.png', dpi=150, bbox_inches='tight')
print('Sensor coverage saved as sensor_coverage_demo.png')
"
```

## Project Structure

```
volumetric-drone-detection/
├── README.md                          # This file
├── requirements.txt                   # Python dependencies
├── .gitignore                        # Git ignore rules
├── src/                              # Source code
│   ├── __init__.py                   # Package initialization
│   ├── drone_trajectory_generator.py # Drone movement simulation
│   ├── sensor_simulation.py          # Virtual sensor arrays
│   └── volumetric_detection.py       # 3D detection pipeline
├── launch_volumetric_test.py         # Main test launcher (parallel/sequential)
├── parallel_volumetric_test.py       # Parallel processing framework
├── quick_validation_test.py          # Quick functionality test
├── volumetric_integration_test.py    # Research validation framework
├── test_trajectory_generator.py      # Legacy trajectory tests
├── test_visualization_fix.py         # Legacy visualization tests
└── *.png                            # Generated visualizations
```

## Configuration Options

### Simulation Bounds
```python
bounds = SimulationBounds(
    x_min=0.0, x_max=1000.0,     # 1km x 1km area
    y_min=0.0, y_max=1000.0,     
    z_min=50.0, z_max=500.0      # 50m-500m altitude
)
```

### Drone Types
- **`'micro'`**: 0.3m wingspan, 15 m/s max speed
- **`'small'`**: 0.8m wingspan, 25 m/s max speed  
- **`'medium'`**: 1.5m wingspan, 35 m/s max speed

### Flight Patterns
- **`COORDINATED_ATTACK`**: Converging from perimeter
- **`FORMATION_FLYING`**: V-formation or grid patterns
- **`RANDOM_DISPERSAL`**: Chaotic movement patterns
- **`EVASIVE_MANEUVERS`**: Zigzag defensive patterns
- **`PERIMETER_SWEEP`**: Systematic area coverage

### Sensor Array Types
- **`"perimeter"`**: 8 cameras around surveillance area
- **`"triangulation"`**: 6 cameras optimized for 3D accuracy
- **`"mixed"`**: Combined visible and thermal sensors

### Detection Methods
- **`OCCUPANCY_GRID`**: Bayesian probabilistic approach
- **`SPACE_CARVING`**: 3D reconstruction via visual hulls
- **`TRIANGULATION`**: Multi-view geometry positioning
- **`HYBRID`**: Combined approach for optimal performance

### Parallel Processing Options
```bash
# Use parallel processing (recommended - 10x faster)
python launch_volumetric_test.py --parallel

# Use sequential processing (slower but deterministic)
python launch_volumetric_test.py --sequential
```

## Expected Output Files

After running tests, you'll generate:

### Research Validation
- `volumetric_detection_validation.json` - Complete research metrics
- `validation_results_parallel.txt` - Parallel processing console output
- `validation_results_sequential.txt` - Sequential processing console output

### Trajectory Visualizations
- `test_trajectory_visualization.png` - Main test output
- `trajectory_visualization_fixed.png` - Fixed axis labels
- `coordinated_attack_demo.png` - Example scenario

### Sensor Analysis
- `sensor_coverage_fixed.png` - Sensor array coverage maps
- `sensor_coverage_demo.png` - Example detection analysis

### Legacy Test Reports
- `test_results.txt` - Comprehensive trajectory test results
- `visualization_results.txt` - Visualization validation

## Performance Optimization

### For Large Scenarios (50+ drones):
```python
# Optimize for memory usage
detection_pipeline = VolumetricDetectionPipeline(
    bounds, 
    voxel_resolution=3.0,  # Lower resolution for speed
    detection_threshold=0.6
)
```

### For High-Accuracy Detection:
```python
# Optimize for accuracy
detection_pipeline = VolumetricDetectionPipeline(
    bounds, 
    voxel_resolution=1.5,  # Higher resolution
    detection_threshold=0.8
)
```

### Parallel Processing Optimization:
- **Use parallel mode** for algorithm development and testing
- **Use sequential mode** for deterministic research results
- **CPU cores**: Automatically uses available cores minus 1
- **Memory efficiency**: Shared data structures reduce overhead

## Benchmarking Results

### Validation Time Comparison:
- **Sequential Processing**: 35-45 minutes
- **Parallel Processing**: 3.6 minutes
- **Speedup Factor**: 10-12x improvement

### Detection Performance by Scenario:
- **Formation Flying**: Best performance (43% detection rate)
- **Evasive Maneuvers**: Moderate performance (14% detection rate)  
- **Random Dispersal**: High accuracy (0.9m error) but low detection rate
- **Coordinated Attack**: Requires algorithm optimization

### Computational Efficiency:
- **Low Resolution (3.0m)**: 17.6 drones/second throughput
- **Medium Resolution (2.0m)**: 7.2 drones/second throughput
- **High Resolution (1.5m)**: 3.8 drones/second throughput

## Troubleshooting

### Common Issues

#### Import Errors
```bash
# Verify Python path and test core imports
python quick_validation_test.py
```

#### Detection Pipeline Issues
```bash
# Test individual components
python -c "from src.volumetric_detection import VolumetricDetectionPipeline; print('Pipeline import successful')"
```

#### Parallel Processing Issues
```bash
# Test with sequential mode if parallel fails
python launch_volumetric_test.py --sequential
```

#### Visualization Problems
```bash
# Test matplotlib backend
python -c "import matplotlib; print(matplotlib.get_backend())"
```

#### Memory Issues for Volumetric Processing
- Increase `voxel_resolution` (2.0-4.0 meters)
- Reduce `num_drones` parameter
- Use shorter simulation durations
- Use parallel processing for better memory management

#### Performance Issues
- Use parallel processing mode
- Close other applications
- Reduce voxel grid resolution
- Use simplified detection methods

### Getting Help

1. **Run quick validation test** for immediate feedback
2. **Check console output** in validation logs
3. **Verify system requirements** (Python 3.11+, sufficient RAM)
4. **Review configuration parameters** for your hardware
5. **Try parallel processing** for better performance

## Development

### Running Research Validation
```bash
# Fast parallel validation (recommended)
python launch_volumetric_test.py --parallel

# Traditional sequential validation
python launch_volumetric_test.py --sequential

# Check results
cat validation_results_parallel.txt
cat volumetric_detection_validation.json
```

### Adding New Detection Algorithms
1. Add new method to `DetectionMethod` enum
2. Implement processing logic in `VolumetricDetectionPipeline`
3. Add test case in `volumetric_integration_test.py`
4. Add parallel worker function in `parallel_volumetric_test.py`

### Performance Profiling
```python
import time
start_time = time.time()
detected_targets = detection_pipeline.process_sensor_observations(observations, timestamp)
print(f"Detection time: {time.time() - start_time:.3f} seconds")
```

### Parallel Development Workflow
```bash
# Quick iteration during development
python quick_validation_test.py

# Algorithm comparison during optimization
python launch_volumetric_test.py --parallel

# Final validation for research
python launch_volumetric_test.py --sequential
```

## Research Applications

This framework is designed for academic research in:
- Counter-UAS system development
- Multi-target tracking algorithms
- Real-time 3D detection methods
- Swarm behavior analysis
- Sensor fusion techniques
- Parallel processing for simulation frameworks

## Research Findings

### Algorithm Performance:
- **Triangulation method** shows most promise with 1.2% detection rate
- **Formation flying scenarios** achieve best results (43% detection)
- **Space carving algorithm** requires optimization (0% detection rate)
- **Hybrid approach** currently limited by space carving performance

### Computational Performance:
- **Parallel processing** achieves 10-12x speedup
- **Desktop hardware** capable of processing 30-50 drone scenarios
- **Memory usage** scales linearly with voxel resolution
- **Real-time processing** requires further optimization (currently 0.4-0.6 Hz vs 10 Hz target)

## License

This project is developed for academic research purposes. See LICENSE file for details.

## Contributing

This is an academic research project. For questions or collaboration inquiries, please open an issue or contact the author.

---

**System Requirements**: Python 3.11+, 8GB+ RAM, Multi-core CPU, Windows/macOS/Linux  
**Performance**: Tested up to 50 simultaneous drones with parallel processing framework