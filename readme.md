# Volumetric Motion Detection for Multi-Drone Swarm Engagement

A theoretical framework and simulation system for real-time volumetric detection and kinetic engagement coordination of drone swarms. This research implements novel integration algorithms between 3D motion detection and engagement systems for counter-UAS applications.

## Overview

This project develops a comprehensive simulation framework that combines:
- Drone trajectory generation with realistic flight patterns
- Multi-camera sensor arrays with environmental limitations
- Volumetric motion detection using probabilistic occupancy grids
- Real-time 3D target detection and tracking algorithms

## Visualization Examples

The system generates comprehensive visualizations of both sensor coverage and drone trajectory patterns:

### Sensor Array Coverage
![Sensor Coverage](sensor_coverage.png)

The sensor array visualization shows the 8-camera perimeter configuration with overlapping fields of view, providing comprehensive coverage of the surveillance area.

### Drone Flight Patterns

#### Evasive Maneuvers Pattern (15 drones)
![Evasive Maneuvers](trajectory_evasive_15drones_small.png)

Shows drones executing evasive maneuvers with rapid direction changes and altitude variations.

#### Formation Flying Pattern (20 drones)
![Formation Flying](trajectory_formation_20drones_small.png)

Demonstrates coordinated formation flying with tight grouping and synchronized movement patterns.

#### Perimeter Sweep Pattern (12 drones)
![Perimeter Sweep](trajectory_perimeter_12drones_small.png)

Illustrates systematic perimeter sweeping with drones maintaining altitude separation and coordinated coverage.

#### Random Dispersal Pattern (8 drones)
![Random Dispersal - 8 drones](trajectory_random_8drones_small.png)

Small-scale random dispersal showing individual drone trajectories with color legend for easy tracking.

#### Random Dispersal Pattern (25 drones)
![Random Dispersal - 25 drones](trajectory_swarm_25drones_small.png)

Large-scale swarm simulation demonstrating complex multi-drone interactions without cluttered legends.

## Current Status

- **Completed**: Drone trajectory generator (150 drones/second performance)
- **Completed**: Multi-camera sensor simulation with realistic limitations  
- **Completed**: Volumetric detection pipeline with hybrid algorithms
- **Completed**: Integration testing framework with research validation
- **Completed**: Parallel processing optimization (10-12x speedup)
- **Completed**: Comprehensive algorithm comparison and benchmarking
- **Completed**: Enhanced visualization system with multiple flight patterns
- **Recently Fixed**: Space carving algorithm now functional with 456x performance improvement

## Features

### Drone Trajectory Generation
- Multiple flight patterns (coordinated attack, formation flying, evasive maneuvers, perimeter sweep, random dispersal)
- Realistic physics constraints (velocity, acceleration limits)
- Environmental effects (wind, sensor noise)
- Scalable from 5-50+ drones per scenario

### Enhanced Visualization System
- Customizable drone count (1-50 drones)
- Multiple flight patterns with distinct behaviors
- 3D trajectory visualization with automatic legend management
- Clean, professional visualizations suitable for research presentation
- Organized image output to dedicated `images/` folder

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

#### Enhanced Visualization Testing
```bash
# Generate visualizations with custom parameters
python test_visualization.py --drones 15 --pattern evasive --type small

# Generate large swarm visualization
python test_visualization.py --drones 25 --pattern swarm --type small

# Generate formation flying demonstration
python test_visualization.py --drones 20 --pattern formation --type small

# Generate perimeter sweep pattern
python test_visualization.py --drones 12 --pattern perimeter --type small

# List available flight patterns
python test_visualization.py --list-patterns
```

#### Comprehensive Validation
```bash
# Run safe integration test with reasonable parameters
python safe_integration_test.py

# Run minimal launcher with custom settings
python minimal_launcher.py --resolution 10.0 --drones 8 --duration 5.0
```

### Legacy Component Tests

#### Debug and Diagnostic Tests
```bash
# Quick diagnostic test
python debug_test.py

# Basic functionality validation
python quick_validation_test_optimized.py
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
python test_visualization.py --drones 15 --pattern coordinated --duration 10.0
```

#### Multi-Sensor Detection Analysis
```bash
python test_visualization.py  # Generates sensor coverage visualization automatically
```

## Project Structure

```
volumetric-drone-detection/
├── README.md                          # This file
├── requirements.txt                   # Python dependencies
├── .gitignore                        # Git ignore rules
├── file_tree.txt                     # Current directory structure
├── Get-FileTree.ps1                  # PowerShell script for directory listing
├── images/                           # Generated visualization outputs
│   ├── sensor_coverage.png           # Sensor array coverage
│   ├── trajectory_evasive_15drones_small.png
│   ├── trajectory_formation_20drones_small.png
│   ├── trajectory_perimeter_12drones_small.png
│   ├── trajectory_random_8drones_small.png
│   └── trajectory_swarm_25drones_small.png
├── src/                              # Source code
│   ├── __init__.py                   # Package initialization
│   ├── drone_trajectory_generator.py # Drone movement simulation
│   ├── sensor_simulation.py          # Virtual sensor arrays
│   ├── volumetric_detection.py       # 3D detection pipeline (optimized)
│   └── __pycache__/                  # Python cache files
├── test_visualization.py             # Enhanced visualization testing (recommended)
├── debug_test.py                     # Quick diagnostic test
├── quick_validation_test_optimized.py # Optimized functionality test
├── safe_integration_test.py          # Safe comprehensive testing
├── minimal_launcher.py               # Configurable test launcher
├── minimal_test_results.json         # Test results data
├── safe_integration_results.json     # Integration test results
└── __pycache__/                      # Python cache files
```

## Configuration Options

### Visualization Options
```bash
# Available flight patterns
python test_visualization.py --list-patterns

# Pattern options: coordinated, evasive, formation, perimeter, random
# Drone count: 1-50 drones
# Duration: 1.0-60.0 seconds
# Drone types: small, medium, large (note: 'large' may default to 'small')
```

### Simulation Bounds
```python
bounds = SimulationBounds(
    x_min=0.0, x_max=1000.0,     # 1km x 1km area
    y_min=0.0, y_max=1000.0,     
    z_min=50.0, z_max=500.0      # 50m-500m altitude
)
```

### Drone Types
- **`'small'`**: 0.8m wingspan, 25 m/s max speed (recommended)
- **`'medium'`**: 1.5m wingspan, 35 m/s max speed
- **`'large'`**: May fallback to 'small' for compatibility

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
- **`TRIANGULATION`**: Multi-view geometry positioning (comprehensive)
- **`SPACE_CARVING`**: Optimized 3D reconstruction (ultra-fast, precise)
- **`HYBRID`**: Combined approach for optimal performance

## Expected Output Files

After running tests, you'll generate:

### Enhanced Visualization Outputs
- `images/sensor_coverage.png` - Sensor array coverage visualization
- `images/trajectory_[pattern]_[N]drones_[type].png` - Drone trajectory visualizations
- Custom trajectory files based on your parameters

### Quick Validation (Recommended)
- Console output showing all three detection methods working
- Real-time performance metrics for each algorithm

### Research Validation
- `safe_integration_results.json` - Safe research metrics
- `minimal_test_results.json` - Minimal test results
- Various console output logs

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

### Visualization Best Practices:
- **Use 8 or fewer drones** for color legend display
- **Use 12+ drones** for pattern analysis without legend clutter
- **Save images to `images/` folder** for organized output
- **Use descriptive filenames** with pattern and drone count

## Benchmarking Results

### Algorithm Performance (Latest Results):
- **Triangulation**: 15 targets detected, 6.2ms processing, 0.34-0.47 confidence
- **Space Carving**: 5 targets detected, 0.9ms processing, 0.46-0.59 confidence  
- **Hybrid**: 15 targets detected, 7.7ms processing, enhanced confidence scores
- **Space Carving Improvement**: 456x faster than previous implementation

### Validation Time Comparison:
- **Quick Test**: < 1 second for all three methods
- **Safe Integration**: 2-5 seconds for multiple configurations
- **Minimal Tests**: < 1 second for custom scenarios

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
python debug_test.py
```

#### Visualization Issues
```bash
# Test visualization system
python test_visualization.py --drones 5 --pattern coordinated
```

#### Pattern Not Found Errors
```bash
# List available patterns
python test_visualization.py --list-patterns

# Use available patterns: coordinated, evasive, formation, perimeter, random
```

#### Memory Issues for Volumetric Processing
- Use `safe_integration_test.py` for memory-safe testing
- Increase `voxel_resolution` (5.0-10.0 meters)
- Reduce `num_drones` parameter
- Use shorter simulation durations

#### Performance Issues
- Use `quick_validation_test_optimized.py` for development
- Use `minimal_launcher.py` for custom scenarios
- Close other applications
- Use appropriate voxel grid resolution

### Getting Help

1. **Run quick validation test** for immediate feedback
2. **Check visualization outputs** in `images/` folder
3. **Verify system requirements** (Python 3.11+, sufficient RAM)
4. **Use safe integration test** for comprehensive validation
5. **Try different flight patterns** for various scenarios

## Development

### Running Research Validation
```bash
# Quick development testing (recommended)
python quick_validation_test_optimized.py

# Safe comprehensive testing
python safe_integration_test.py

# Custom scenario testing
python minimal_launcher.py --resolution 10.0 --drones 8

# Visualization testing
python test_visualization.py --drones 15 --pattern formation
```

### Adding New Flight Patterns
1. Add new pattern to `FlightPattern` enum in `drone_trajectory_generator.py`
2. Update pattern mapping in `test_visualization.py`
3. Test with `python test_visualization.py --pattern new_pattern`

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

# Visualization testing during development
python test_visualization.py --drones 8 --pattern random

# Algorithm testing
python debug_test.py

# Safe comprehensive validation
python safe_integration_test.py
```

## Research Applications

This framework is designed for academic research in:
- Counter-UAS system development
- Multi-target tracking algorithms
- Real-time 3D detection methods
- Swarm behavior analysis
- Sensor fusion techniques
- Drone flight pattern analysis
- Visualization of complex multi-agent systems

## Research Findings

### Algorithm Performance (Updated):
- **All three detection methods** now fully functional and optimized
- **Triangulation method** provides comprehensive coverage (15 targets)
- **Space carving method** offers ultra-fast precision filtering (5 targets, 0.9ms)
- **Hybrid approach** combines strengths of both methods (15 targets, enhanced confidence)

### Computational Performance:
- **Space carving optimization** achieved 456x performance improvement
- **Safe parameter selection** enables reliable testing
- **Desktop hardware** capable of real-time processing
- **Memory usage** optimized for practical scenarios
- **All methods** now suitable for real-time applications

### Visualization Insights:
- **Pattern differentiation** clearly visible in 3D trajectories
- **Scalable legend management** for different swarm sizes
- **Professional quality** visualizations suitable for research publication
- **Comprehensive sensor coverage** visualization for array validation

### Recent Breakthroughs:
- **Fixed space carving algorithm**: Now detects targets reliably
- **Simplified clustering approach**: Direct detection-based clustering
- **Performance optimization**: Sub-millisecond space carving processing
- **Enhanced visualization system**: Multiple flight patterns with clean output
- **Safe testing framework**: Reliable parameter selection and validation

## License

This project is developed for academic research purposes. See LICENSE file for details.

## Contributing

This is an academic research project. For questions or collaboration inquiries, please open an issue or contact the author.

---

**System Requirements**: Python 3.11+, 8GB+ RAM, Multi-core CPU, Windows/macOS/Linux  
**Performance**: All detection methods optimized for real-time processing with safe testing framework  
**Status**: Fully functional volumetric detection pipeline with enhanced visualization capabilities