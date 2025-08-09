"""
Quick diagnostic script to identify memory spike in integration test
Run this to see exactly where the 30% -> 70% CPU/RAM jump occurs
"""

import psutil
import os
import gc
import time
import sys

def monitor_step(step_name):
    """Monitor resource usage at each step"""
    process = psutil.Process(os.getpid())
    cpu = process.cpu_percent()
    memory_mb = process.memory_info().rss / 1024 / 1024
    
    print(f"{step_name}:")
    print(f"  CPU: {cpu:.1f}%")
    print(f"  Memory: {memory_mb:.1f}MB")
    print(f"  Threads: {process.num_threads()}")
    print()
    
    return {'cpu': cpu, 'memory': memory_mb}

def main():
    print("=== MEMORY SPIKE DIAGNOSTIC ===")
    print("Tracking the 30% -> 70% CPU/RAM jump\n")
    
    # Add src path
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))
    
    baseline = monitor_step("BASELINE (startup)")
    
    try:
        # Step 1: Import modules
        from drone_trajectory_generator import DroneSwarmGenerator, FlightPattern, SimulationBounds
        step1 = monitor_step("STEP 1: Import drone_trajectory_generator")
        
        from sensor_simulation import create_standard_sensor_array
        step2 = monitor_step("STEP 2: Import sensor_simulation")
        
        from volumetric_detection import VolumetricDetectionPipeline, DetectionMethod
        step3 = monitor_step("STEP 3: Import volumetric_detection")
        
        # Step 2: Create objects
        bounds = SimulationBounds()
        step4 = monitor_step("STEP 4: Create SimulationBounds")
        
        drone_generator = DroneSwarmGenerator(bounds)
        step5 = monitor_step("STEP 5: Create DroneSwarmGenerator")
        
        sensor_array = create_standard_sensor_array(bounds, "perimeter")
        step6 = monitor_step("STEP 6: Create sensor array")
        
        # This is often where the spike happens
        detection_pipeline = VolumetricDetectionPipeline(bounds, voxel_resolution=3.0)
        step7 = monitor_step("STEP 7: Create VolumetricDetectionPipeline (LIKELY SPIKE POINT)")
        
        # Step 3: Generate data
        trajectory_data = drone_generator.generate_swarm_trajectories(
            num_drones=5, pattern=FlightPattern.COORDINATED_ATTACK, duration=2.0, timestep=0.5
        )
        step8 = monitor_step("STEP 8: Generate trajectories")
        
        trajectories = trajectory_data['trajectories']
        times = trajectory_data['times']
        drone_positions = trajectories[:, 0, :]
        timestamp = times[0]
        step9 = monitor_step("STEP 9: Extract trajectory data")
        
        # Step 4: Process observations (another likely spike point)
        print(">>> PROCESSING SENSOR OBSERVATIONS <<<")
        observations = sensor_array.observe_targets_parallel(drone_positions, timestamp)
        step10 = monitor_step("STEP 10: Process sensor observations (ANOTHER LIKELY SPIKE POINT)")
        
        # Step 5: Detection processing
        detected_targets = detection_pipeline.process_sensor_observations_parallel(
            observations, timestamp, DetectionMethod.TRIANGULATION
        )
        step11 = monitor_step("STEP 11: Detection processing")
        
        # Force garbage collection and check if memory comes down
        print(">>> FORCING GARBAGE COLLECTION <<<")
        collected = gc.collect()
        step12 = monitor_step(f"STEP 12: After garbage collection ({collected} objects)")
        
        # Summary
        print("=== ANALYSIS ===")
        steps = [baseline, step1, step2, step3, step4, step5, step6, step7, step8, step9, step10, step11, step12]
        
        print("Memory usage progression:")
        for i, step in enumerate(steps):
            if i == 0:
                print(f"  Baseline: {step['memory']:.1f}MB")
            else:
                delta = step['memory'] - baseline['memory']
                print(f"  Step {i}: {step['memory']:.1f}MB (+{delta:.1f}MB)")
        
        # Identify the biggest jumps
        print("\nBiggest memory increases:")
        for i in range(1, len(steps)):
            increase = steps[i]['memory'] - steps[i-1]['memory']
            if increase > 20:  # More than 20MB increase
                step_names = [
                    "Import drone_trajectory_generator",
                    "Import sensor_simulation", 
                    "Import volumetric_detection",
                    "Create SimulationBounds",
                    "Create DroneSwarmGenerator",
                    "Create sensor array",
                    "Create VolumetricDetectionPipeline",
                    "Generate trajectories",
                    "Extract trajectory data",
                    "Process sensor observations",
                    "Detection processing",
                    "Garbage collection"
                ]
                step_name = step_names[i-1] if i-1 < len(step_names) else f"Step {i}"
                print(f"  {step_name}: +{increase:.1f}MB")
        
        print(f"\nTotal memory increase: {steps[-1]['memory'] - baseline['memory']:.1f}MB")
        print(f"Results: {len(observations)} observations, {len(detected_targets)} targets")
        
    except Exception as e:
        print(f"Error during diagnostic: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
