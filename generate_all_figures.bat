# Generate visualizations with custom parameters
python test_visualization.py --drones 15 --pattern evasive --type small

# Generate large swarm visualization
python test_visualization.py --drones 25 --pattern swarm --type small

# Generate formation flying demonstration
python test_visualization.py --drones 20 --pattern formation --type small

# Generate perimeter sweep pattern
python test_visualization.py --drones 12 --pattern perimeter --type small

python test_visualization.py

python test_visualization.py --detection-analysis individual