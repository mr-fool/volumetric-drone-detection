# Volumetric Motion Detection for Multi-Drone Swarm Engagement

A theoretical framework and simulation system for real-time volumetric detection and kinetic engagement coordination of drone swarms. This research implements novel integration algorithms between 3D motion detection and engagement systems for counter-UAS applications.

## Overview

This project develops a comprehensive simulation framework that combines:
- Drone trajectory generation with realistic flight patterns
- Multi-camera sensor arrays with environmental limitations
- Volumetric motion detection using probabilistic occupancy grids
- Real-time 3D target detection and tracking algorithms

## Technical Specifications

### Volumetric Detection Parameters

#### Voxel Resolution Impact
The **voxel resolution** parameter determines the size of 3D grid cubes used for volumetric analysis:

| Resolution | Voxel Size | Total Voxels | Memory Usage | Performance | Detection Precision |
|------------|------------|--------------|--------------|-------------|-------------------|
| 10.0m      | 10×10×10m  | 450,000      | ~10MB       | Fastest     | Low precision     |
| 7.0m       | 7×7×7m     | 1.3M         | ~30MB       | Fast        | Medium precision  |
| 5.0m       | 5×5×5m     | 3.6M         | ~82MB       | Moderate    | Good precision    |
| 3.0m       | 3×3×3m     | 16.7M        | ~381MB      | Slow        | High precision    |
| 2.0m       | 2×2×2m     | 56.3M        | ~1.3GB      | Very slow   | Very high precision |
| 1.5m       | 1.5×1.5×1.5m | 133M+      | ~3GB+       | Impractical | Maximum precision |

**Recommended Configurations:**
- **Development/Testing**: 10.0m resolution (fast, low memory)
- **Standard Operation**: 5.0-7.0m resolution (balanced performance)
- **High-Precision Research**: 3.0m resolution (maximum practical precision)

#### Memory Usage Formula
```
Total Voxels = (Area_X / Resolution) × (Area_Y / Resolution) × (Area_Z / Resolution)
Memory (MB) = Total_Voxels × 3_coordinates × 8_bytes / (1024²)
```

### Surveillance Coverage Specifications

#### Operational Area
```python
# Default simulation bounds
Surveillance Area: 1000m × 1000m (1 km²)
Altitude Range: 50m - 500m above ground level
Total Volume: 450,000,000 m³
```

#### Altitude Detection Characteristics

| Altitude Range | Detection Difficulty | Voxel Resolution Needed | Typical Use Cases |
|----------------|---------------------|------------------------|-------------------|
| 50-150m        | Easy                | 5.0-7.0m              | Low-altitude surveillance, landing detection |
| 150-300m       | Moderate            | 3.0-5.0m              | Standard patrol monitoring |
| 300-500m       | Challenging         | 2.0-3.0m              | High-altitude tracking, long-range detection |

#### Camera Array Specifications

**Perimeter Configuration (Default):**
- **Camera Count**: 8 cameras
- **Placement**: Distributed around surveillance perimeter
- **Detection Range**: 1-2 km (varies by sensor type)
- **Field of View**: Overlapping coverage for 3D triangulation
- **Coverage**: Complete perimeter monitoring with altitude coverage

**Triangulation Configuration:**
- **Camera Count**: 6 cameras
- **Placement**: Optimized for 3D accuracy
- **Detection Range**: 1-1.5 km
- **Precision**: Higher 3D positioning accuracy
- **Coverage**: Optimized for central area detection

#### Detection Range vs. Distance

| Distance from Cameras | Detection Confidence | Recommended Resolution | Processing Time |
|----------------------|---------------------|----------------------|----------------|
| 0-500m               | High (0.7-0.9)      | 5.0-7.0m            | Fast           |
| 500-1000m            | Medium (0.5-0.7)    | 3.0-5.0m            | Moderate       |
| 1000-1500m           | Low (0.3-0.5)       | 2.0-3.0m            | Slow           |
| 1500m+               | Very Low (<0.3)     | 1.5-2.0m            | Very slow      |

### Drone Detection Capabilities

#### Drone Size vs. Detection Range

| Drone Type | Wingspan | Max Speed | Detection Range | Min Resolution | Typical Confidence |
|------------|----------|-----------|----------------|---------------|-------------------|
| Micro      | 0.3m     | 15 m/s    | 300-800m       | 3.0m          | 0.3-0.6          |
| Small      | 0.8m     | 25 m/s    | 500-1200m      | 5.0m          | 0.4-0.7          |
| Medium     | 1.5m     | 35 m/s    | 800-1500m      | 7.0m          | 0.5-0.8          |
| Large      | 2.5m+    | 45 m/s    | 1000-2000m     | 10.0m         | 0.6-0.9          |

#### Altitude-Specific Performance

**Low Altitude (50-150m):**
- **Advantages**: High detail, strong sensor response, precise tracking
- **Challenges**: Limited coverage area per camera
- **Optimal Resolution**: 5.0-7.0m
- **Detection Confidence**: 0.6-0.9

**Medium Altitude (150-300m):**
- **Advantages**: Balanced coverage and detail, optimal sensor performance
- **Challenges**: Moderate atmospheric interference
- **Optimal Resolution**: 3.0-5.0m
- **Detection Confidence**: 0.5-0.8

**High Altitude (300-500m):**
- **Advantages**: Wide area coverage, strategic overview
- **Challenges**: Reduced sensor resolution, atmospheric effects
- **Optimal Resolution**: 2.0-3.0m
- **Detection Confidence**: 0.3-0.7

### Algorithm Performance Characteristics

#### Detection Method Comparison

| Method | Processing Time | Target Count | Precision | Coverage | Best Use Case |
|--------|----------------|--------------|-----------|----------|---------------|
| **Triangulation** | 6.2ms | 15 targets | Medium | High | Comprehensive surveillance |
| **Space Carving** | 0.9ms | 5 targets | High | Medium | Precision tracking |
| **Hybrid** | 7.7ms | 15 targets | High | High | Optimal balanced detection |

#### Performance vs. Resolution Trade-offs

| Resolution | Triangulation | Space Carving | Hybrid | Memory | Use Case |
|------------|---------------|---------------|--------|--------|----------|
| 10.0m      | 3-8ms         | 0.5-1.0ms     | 4-9ms  | 10MB   | Real-time development |
| 7.0m       | 8-15ms        | 1.0-2.0ms     | 10-18ms| 30MB   | Standard operation |
| 5.0m       | 15-25ms       | 2.0-4.0ms     | 18-30ms| 82MB   | High-quality detection |
| 3.0m       | 50-100ms      | 5.0-10ms      | 60-120ms| 381MB | Research/analysis |

### Environmental Factors

#### Weather Impact on Detection

| Condition | Visibility | Detection Range | Confidence Penalty | Recommended Settings |
|-----------|------------|----------------|-------------------|---------------------|
| Clear     | Excellent  | 100%           | None              | Standard resolution |
| Light Rain| Good       | 80%            | -10%              | Increase resolution 1 step |
| Heavy Rain| Poor       | 60%            | -25%              | Increase resolution 2 steps |
| Fog       | Very Poor  | 40%            | -40%              | Maximum resolution needed |

#### Time of Day Considerations

| Time Period | Sensor Type | Detection Quality | Recommended Configuration |
|-------------|-------------|------------------|--------------------------|
| Daylight    | Visible     | Excellent        | Standard settings        |
| Dawn/Dusk   | Mixed       | Good             | Slightly higher resolution|
| Night       | Thermal/IR  | Variable         | Higher resolution + thermal|

### System Resource Requirements

#### Minimum Requirements
- **CPU**: Quad-core 2.0GHz
- **RAM**: 8GB (for resolutions ≥5.0m)
- **Storage**: 2GB free space
- **Python**: 3.11+

#### Recommended Specifications
- **CPU**: 8+ cores, 3.0GHz+ (AMD Ryzen 5/7, Intel i5/i7)
- **RAM**: 16-32GB (for resolutions 3.0-5.0m)
- **Storage**: 5GB free space
- **GPU**: Optional (future enhancement)

#### High-Performance Research Setup
- **CPU**: 12+ cores, 3.5GHz+ (AMD Ryzen 7/9, Intel i7/i9)
- **RAM**: 32-64GB (for resolutions ≤3.0m)
- **Storage**: 10GB+ SSD
- **Network**: High-speed for distributed processing

### Configuration Guidelines

#### Performance-Optimized Settings
```bash
# Fast development testing
python minimal_launcher.py --resolution 10.0 --drones 5 --duration 2.0

# Balanced production use
python minimal_launcher.py --resolution 5.0 --drones 12 --duration 5.0
```

#### Accuracy-Optimized Settings
```bash
# High-precision research
python safe_integration_test.py  # Uses conservative 10.0m resolution

# Maximum practical accuracy (requires 32GB+ RAM)
python minimal_launcher.py --resolution 3.0 --drones 8 --duration 3.0
```

#### Resource-Constrained Settings
```bash
# Low memory systems (8GB RAM)
python minimal_launcher.py --resolution 10.0 --drones 3 --duration 1.0

# Limited processing power
python debug_test.py  # Minimal resource usage
```

## Visualization Examples

The system generates comprehensive visualizations of both sensor coverage and drone trajectory patterns:

### Sensor Array Coverage

The sensor array visualization shows the 8-camera perimeter configuration with overlapping fields of view, providing comprehensive coverage of the surveillance area.
<img width="2088" height="934" alt="sensor_coverage" src="https://github.com/user-attachments/assets/b0f8dd3d-bc42-4c91-93a2-330867af5e59" />

### Drone Flight Patterns

#### Evasive Maneuvers Pattern (15 drones)
<img width="1222" height="1275" alt="trajectory_evasive_15drones_small" src="https://github.com/user-attachments/assets/9e311993-1c53-483a-ae96-4f27a9adb863" />

Shows drones executing evasive maneuvers with rapid direction changes and altitude variations.

#### Formation Flying Pattern (20 drones)
<img width="1222" height="1275" alt="trajectory_formation_20drones_small" src="https://github.com/user-attachments/assets/efbe57f9-ddf7-4874-bf37-378395e45d1e" />

Demonstrates coordinated formation flying with tight grouping and synchronized movement patterns.

#### Perimeter Sweep Pattern (12 drones)
<img width="1222" height="1275" alt="trajectory_perimeter_12drones_small" src="https://github.com/user-attachments/assets/da55f683-5dc4-4f09-8359-cb9f35f72a86" />

Illustrates systematic perimeter sweeping with drones maintaining altitude separation and coordinated coverage.

#### Random Dispersal Pattern (8 drones)
<img width="1222" height="1275" alt="trajectory_random_8drones_small" src="https://github.com/user-attachments/assets/70e368a9-91c3-4b35-973a-e0f26f187cbc" />

Small-scale random dispersal showing individual drone trajectories with color legend for easy tracking.

#### Random Dispersal Pattern (25 drones)
<img width="1222" height="1275" alt="trajectory_swarm_25drones_small" src="https://github.com/user-attachments/assets/4e189aac-5c09-4d71-817e-37cdfeb0687d" />

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
- **RAM**: 64GB DDR4
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
