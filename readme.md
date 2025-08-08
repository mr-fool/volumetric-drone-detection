# Volumetric Motion Detection for Multi-Drone Swarm Engagement

A theoretical framework and simulation system for real-time volumetric detection and kinetic engagement coordination of drone swarms. This research implements novel integration algorithms between 3D motion detection and engagement systems for counter-UAS applications.

## Overview

This project develops a comprehensive simulation framework that combines:
- Drone trajectory generation with realistic flight patterns
- Multi-camera sensor arrays with environmental limitations
- Volumetric motion detection using probabilistic occupancy grids
- Engagement coordination algorithms for kinetic systems

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
python -c "from src.drone_trajectory_generator import DroneSwarmGenerator; print('Installation successful!')"
```

## Usage Guide

### Running Tests

#### Comprehensive System Test
```bash
# Test all trajectory generation functionality
python test_trajectory_generator.py > test_results.txt 2>&1

# View results
cat test_results.txt
```

#### Visualization and Sensor Tests
```bash
# Test visualization fixes and sensor simulation
python test_visualization_fix.py > visualization_results.txt 2>&1

# Check generated images
ls *.png
```

#### Import Verification
```bash
# Quick test that all components import correctly
python -c "from src.sensor_simulation import create_standard_sensor_array; print('Sensor simulation ready!')"
```

### Performance Benchmarking

The trajectory generator has been tested and validated on the following hardware:
- **CPU**: AMD Ryzen 5 5600 
- **RAM**: 32GB DDR4
- **GPU**: RTX 3060 12GB
- **OS**: Windows 11

**Benchmark Results:**
- **Generation Speed**: ~150 drones/second
- **Memory Usage**: <0.25 MB per 50-drone scenario
- **Supported Scenarios**: 5-50+ simultaneous drones
- **Processing Time**: 0.007 seconds per drone

### Example Scenarios

#### Scenario 1: Coordinated Swarm Attack
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

#### Scenario 2: Multi-Sensor Detection Analysis
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
│   └── sensor_simulation.py          # Virtual sensor arrays
├── test_trajectory_generator.py      # Comprehensive testing
├── test_visualization_fix.py         # Visualization validation
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

## Expected Output Files

After running tests, you'll generate:

### Trajectory Visualizations
- `test_trajectory_visualization.png` - Main test output
- `trajectory_visualization_fixed.png` - Fixed axis labels
- `coordinated_attack_demo.png` - Example scenario

### Sensor Analysis
- `sensor_coverage_fixed.png` - Sensor array coverage maps
- `sensor_coverage_demo.png` - Example detection analysis

### Test Reports
- `test_results.txt` - Comprehensive test results
- `visualization_results.txt` - Visualization validation

## Performance Optimization

### For Large Scenarios (50+ drones):
```python
# Optimize for memory usage
data = generator.generate_swarm_trajectories(
    num_drones=50,
    pattern=FlightPattern.COORDINATED_ATTACK,
    duration=30.0,  # Shorter duration
    timestep=0.2,   # Larger timestep
    drone_type='small'
)
```

### For High-Speed Generation:
```python
# Optimize for speed
data = generator.generate_swarm_trajectories(
    num_drones=20,
    pattern=FlightPattern.FORMATION_FLYING,
    duration=15.0,
    timestep=0.1,
    drone_type='micro'  # Lighter computation
)
```

## Troubleshooting

### Common Issues

#### Import Errors
```bash
# Verify Python path
python -c "import sys; print(sys.path)"

# Check if src directory has __init__.py
ls src/__init__.py
```

#### Visualization Problems
```bash
# Test matplotlib backend
python -c "import matplotlib; print(matplotlib.get_backend())"

# Generate test plot
python -c "import matplotlib.pyplot as plt; plt.plot([1,2,3]); plt.savefig('test.png'); print('Matplotlib working')"
```

#### Memory Issues
- Reduce `num_drones` parameter
- Increase `timestep` (0.2-0.5 seconds)
- Decrease `duration` (15-30 seconds)
- Use `'micro'` drone type for lighter computation

#### Performance Issues
- Close other applications
- Use GPU acceleration if available
- Reduce visualization complexity

### Getting Help

1. **Check test outputs** for error messages
2. **Verify system requirements** (Python 3.11+, sufficient RAM)
3. **Review configuration parameters** for your hardware
4. **Generate minimal examples** to isolate issues

## Development

### Running All Tests
```bash
# Full test suite
python test_trajectory_generator.py > full_test_results.txt 2>&1
python test_visualization_fix.py > viz_test_results.txt 2>&1

# Verify all outputs
ls *.png *.txt
```

### Adding New Flight Patterns
1. Add new enum to `FlightPattern`
2. Implement logic in `_calculate_desired_velocity()`
3. Add test case in `test_trajectory_generator.py`

### Performance Profiling
```python
import time
start_time = time.time()
# Your simulation code here
print(f"Execution time: {time.time() - start_time:.3f} seconds")
```

## License

This project is developed for academic research purposes. See LICENSE file for details.

## Contributing

This is an academic research project. For questions or collaboration inquiries, please open an issue or contact the author.

---

**System Requirements**: Python 3.11+, 8GB+ RAM, Windows/macOS/Linux  
**Performance**: Tested up to 50 simultaneous drones at 150 drones/second generation rate