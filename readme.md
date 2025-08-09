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
- **Recently Fixed**: Space carving algorithm now functional with 456x performance improvement

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
- **Optimized space carving algorithms** for 3D reconstruction (now 456x faster)
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
git clone https://github.com/mr-fool/volumetric-drone-detection
cd volumetric-drone-detection
```

2. **Install dependencies:**
```bash
pip install -r requirements.txt
```

3. **Verify installation:**
```bash
python quick_validation_test_optimized.py
```

## Usage Guide

### Quick Testing

#### Basic Functionality Test (Recommended)
```bash
# Test all three detection methods with optimized performance
python quick_validation_test_optimized.py
```

#### Comprehensive Validation
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

**Current Performance (Optimized Processing):**
- **Trajectory Generation**: ~150 drones/second
- **Triangulation Detection**: 6.2ms processing time, 15 targets detected
- **Space Carving Detection**: 0.9ms processing time, 5 high-confidence targets
- **Hybrid Detection**: 7.7ms processing time, 15 targets with enhanced confidence
- **Validation Time**: 3.6 minutes (vs 35-45 minutes sequential)
- **CPU Utilization**: 11 parallel workers
- **Memory Usage**: ~330MB for volumetric grids

**Updated Detection Performance:**
- **Triangulation Method**: Fast and comprehensive (15 targets, 6.2ms)
- **Space Carving Method**: Ultra-fast and precise (5 targets, 0.9ms, 456x speedup)
- **Hybrid Method**: Best overall performance (15 targets, enhanced confidence)
- **Overall System**: All three methods now fully functional

### Example Scenarios

#### Quick Detection Test
```bash
python quick_validation_test_optimized.py
```

Expected output:
```
Testing triangulation:
  Results: 15 targets detected (6.2ms processing)

Testing space_carving:
  Results: 5 targets detected (0.9ms processing)

Testing hybrid:
  Results: 15 targets detected (7.7ms processing)
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
├── file_tree.txt                     # Current directory structure
├── Get-FileTree.ps1                  # PowerShell script for directory listing
├── src/                              # Source code
│   ├── __init__.py                   # Package initialization
│   ├── drone_trajectory_generator.py # Drone movement simulation
│   ├── sensor_simulation.py          # Virtual sensor arrays
│   ├── volumetric_detection.py       # 3D detection pipeline (optimized)
│   └── __pycache__/                  # Python cache files
├── launch_volumetric_test.py         # Main test launcher (parallel/sequential)
├── parallel_volumetric_test.py       # Parallel processing framework
├── quick_validation_test_optimized.py # Optimized functionality test (recommended)
├── quick_validation_test.py          # Legacy functionality test
├── volumetric_integration_test.py    # Research validation framework
├── test_trajectory_generator.py      # Legacy trajectory tests
├── test_visualization_fix.py         # Legacy visualization tests
├── debug_space_carving.py            # Debug utilities for space carving
├── debug_test.py                     # General debug utilities
├── validation_results_parallel.txt   # Parallel test output log
├── quick_validation_test_optimized.txt # Quick test output log
└── __pycache__/                      # Python cache files
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
- **`SPACE_CARVING`**: Optimized 3D reconstruction (ultra-fast)
- **`TRIANGULATION`**: Multi-view geometry positioning (comprehensive)
- **`HYBRID`**: Combined approach for optimal performance

### Parallel Processing Options
```bash
# Use optimized quick test (recommended for development)
python quick_validation_test_optimized.py

# Use parallel processing for comprehensive validation
python launch_volumetric_test.py --parallel

# Use sequential processing (slower but deterministic)
python launch_volumetric_test.py --sequential
```

## Expected Output Files

After running tests, you'll generate:

### Quick Validation (Recommended)
- Console output showing all three detection methods working
- Real-time performance metrics for each algorithm

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
- **Use optimized quick test** for rapid algorithm development
- **Use parallel mode** for comprehensive testing
- **Use sequential mode** for deterministic research results
- **CPU cores**: Automatically uses available cores minus 1
- **Memory efficiency**: Shared data structures reduce overhead

## Benchmarking Results

### Algorithm Performance (Latest Results):
- **Triangulation**: 15 targets detected, 6.2ms processing, 0.34-0.47 confidence
- **Space Carving**: 5 targets detected, 0.9ms processing, 0.46-0.59 confidence  
- **Hybrid**: 15 targets detected, 7.7ms processing, enhanced confidence scores
- **Space Carving Improvement**: 456x faster than previous implementation

### Validation Time Comparison:
- **Quick Test**: < 1 second for all three methods
- **Sequential Processing**: 35-45 minutes
- **Parallel Processing**: 3.6 minutes
- **Speedup Factor**: 10-12x improvement

### Detection Performance by Method:
- **Triangulation**: Best coverage (comprehensive detection)
- **Space Carving**: Best precision (high-confidence targets only)
- **Hybrid**: Best overall (maintains coverage, enhances confidence)

### Computational Efficiency:
- **Space Carving**: 0.9ms (ultra-fast, precise filtering)
- **Triangulation**: 6.2ms (fast, comprehensive detection)
- **Hybrid**: 7.7ms (optimal balance of speed and accuracy)

## Troubleshooting

### Common Issues

#### Import Errors
```bash
# Verify Python path and test core imports
python quick_validation_test_optimized.py
```

#### Detection Pipeline Issues
```bash
# Test individual components
python -c "from src.volumetric_detection import VolumetricDetectionPipeline; print('Pipeline import successful')"
```

#### Space Carving Not Detecting Targets
This issue has been resolved. If you encounter problems:
1. Ensure you're using the latest `volumetric_detection.py`
2. Run the optimized quick test to verify functionality
3. Check that sensor observations contain detected objects

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
- Use optimized quick test for development
- Use parallel processing mode for validation
- Close other applications
- Reduce voxel grid resolution
- Use simplified detection methods

### Getting Help

1. **Run optimized quick validation test** for immediate feedback
2. **Check console output** in validation logs
3. **Verify system requirements** (Python 3.11+, sufficient RAM)
4. **Review configuration parameters** for your hardware
5. **Try parallel processing** for better performance

## Development

### Running Research Validation
```bash
# Quick development testing (recommended)
python quick_validation_test_optimized.py

# Fast parallel validation
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
detected_targets = detection_pipeline.process_sensor_observations_parallel(observations, timestamp, method)
print(f"Detection time: {time.time() - start_time:.3f} seconds")
```

### Development Workflow
```bash
# Quick iteration during development
python quick_validation_test_optimized.py

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

### Algorithm Performance (Updated):
- **All three detection methods** now fully functional and optimized
- **Triangulation method** provides comprehensive coverage (15 targets)
- **Space carving method** offers ultra-fast precision filtering (5 targets, 0.9ms)
- **Hybrid approach** combines strengths of both methods (15 targets, enhanced confidence)

### Computational Performance:
- **Space carving optimization** achieved 456x performance improvement
- **Parallel processing** achieves 10-12x speedup
- **Desktop hardware** capable of real-time processing
- **Memory usage** optimized for large-scale scenarios
- **All methods** now suitable for real-time applications

### Recent Breakthroughs:
- **Fixed space carving algorithm**: Now detects targets reliably
- **Simplified clustering approach**: Direct detection-based clustering
- **Performance optimization**: Sub-millisecond space carving processing
- **Enhanced hybrid method**: Proper fusion of all detection methods

## License

This project is developed for academic research purposes. See LICENSE file for details.

## Contributing

This is an academic research project. For questions or collaboration inquiries, please open an issue or contact the author.

---

**System Requirements**: Python 3.11+, 8GB+ RAM, Multi-core CPU, Windows/macOS/Linux  
**Performance**: All detection methods optimized for real-time processing with parallel framework  
**Status**: Fully functional volumetric detection pipeline with 456x space carving improvement