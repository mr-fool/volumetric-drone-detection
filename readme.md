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
- **Completed**: Integration testing framework with academic validation
- **In Progress**: Algorithm optimization and performance validation

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
- GPU acceleration optional but recommended

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

#### Comprehensive Validation
```bash
# Run full research validation framework
python launch_volumetric_test.py
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
- **CPU**: AMD Ryzen 5 5600 
- **RAM**: 32GB DDR4
- **GPU**: RTX 3060 12GB (optional)
- **OS**: Windows 11

**Current Performance:**
- **Trajectory Generation**: ~150 drones/second
- **Detection Processing**: 1-2 Hz for 30-50 drones
- **Memory Usage**: ~330MB for volumetric grids
- **Target**: Real-time processing at 10 Hz

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
├── launch_volumetric_test.py         # Main test launcher
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

## Expected Output Files

After running tests, you'll generate:

### Research Validation
- `volumetric_detection_validation.json` - Complete research metrics
- `validation_results_fixed.txt` - Console output log

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

#### Visualization Problems
```bash
# Test matplotlib backend
python -c "import matplotlib; print(matplotlib.get_backend())"
```

#### Memory Issues for Volumetric Processing
- Increase `voxel_resolution` (2.0-4.0 meters)
- Reduce `num_drones` parameter
- Use shorter simulation durations

#### Performance Issues
- Close other applications
- Reduce voxel grid resolution
- Use simplified detection methods

### Getting Help

1. **Run quick validation test** for immediate feedback
2. **Check console output** in validation logs
3. **Verify system requirements** (Python 3.11+, sufficient RAM)
4. **Review configuration parameters** for your hardware

## Development

### Running Research Validation
```bash
# Complete validation framework
python launch_volumetric_test.py

# Check results
cat validation_results_fixed.txt
cat volumetric_detection_validation.json
```

### Adding New Detection Algorithms
1. Add new method to `DetectionMethod` enum
2. Implement processing logic in `VolumetricDetectionPipeline`
3. Add test case in `volumetric_integration_test.py`

### Performance Profiling
```python
import time
start_time = time.time()
detected_targets = detection_pipeline.process_sensor_observations(observations, timestamp)
print(f"Detection time: {time.time() - start_time:.3f} seconds")
```

## Research Applications

This framework is designed for academic research in:
- Counter-UAS system development
- Multi-target tracking algorithms
- Real-time 3D detection methods
- Swarm behavior analysis
- Sensor fusion techniques

## License

This project is developed for academic research purposes. See LICENSE file for details.

## Contributing

This is an academic research project. For questions or collaboration inquiries, please open an issue or contact the author.

---

**System Requirements**: Python 3.11+, 8GB+ RAM, Windows/macOS/Linux  
**Performance**: Tested up to 50 simultaneous drones with real-time detection processing