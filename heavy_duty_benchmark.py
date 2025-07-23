"""
heavy_duty_benchmark.py

This script performs an extreme-load benchmark on the deconfliction engine.
It is designed to test the system's performance against a large number of
drones, each with a long and complex multi-waypoint flight path.

This specifically stresses the per-flight workload of the broad-phase filter.
"""

import cProfile
import pstats
import random
import time

# --- Import project modules ---
from src.data_models import Waypoint, PrimaryMission, SimulatedFlight
from src.deconfliction_logic import check_conflicts_hybrid

def generate_complex_flight(flight_id: int) -> SimulatedFlight:
    """
    Generates a single drone flight with a complex, 100-waypoint "random walk" path.
    """
    waypoints = []
    timestamps = []

    # Start the drone far away from the primary mission's area of interest
    current_x = random.uniform(10000, 20000)
    current_y = random.uniform(10000, 20000)
    current_z = random.uniform(100, 500)
    current_t = random.uniform(0, 100)

    # Add the starting point
    waypoints.append(Waypoint(current_x, current_y, current_z))
    timestamps.append(current_t)

    # Generate 99 more waypoints
    for _ in range(99):
        current_x += random.uniform(-200, 200)
        current_y += random.uniform(-200, 200)
        current_z += random.uniform(-50, 50)
        current_t += random.uniform(10, 30) # Time for each segment
        waypoints.append(Waypoint(current_x, current_y, current_z))
        timestamps.append(current_t)

    return SimulatedFlight(
        flight_id=f"COMPLEX_DRONE_{flight_id}",
        waypoints=waypoints,
        timestamps=timestamps
    )

def run_heavy_benchmark():
    """
    Sets up and runs the high-complexity deconfliction scenario.
    """
    print("--- Heavy-Duty Performance Benchmark ---")
    
    # --- 1. Configuration ---
    config = {
        "deconfliction_parameters": {
            "safety_buffer_m": 50.0,
            "grid_bin_size": { "x_m": 100.0, "y_m": 100.0, "z_m": 100.0, "t_s": 10.0 }
        }
    }
    primary_drone_speed = 25.0

    # --- 2. Define the Scenario ---
    primary_mission = PrimaryMission(
        waypoints=[Waypoint(0, 0, 100), Waypoint(1000, 0, 100)],
        start_time=0,
        end_time=50
    )
    
    num_complex_flights = 10000
    print(f"Generating a complex airspace with {num_complex_flights} drones, each with 100 waypoints...")
    
    start_gen_time = time.time()
    simulated_flights = [generate_complex_flight(i) for i in range(num_complex_flights)]
    gen_duration = time.time() - start_gen_time
    print(f"Airspace generation took {gen_duration:.2f} seconds.")

    # --- 3. Run the Profiler ---
    print("Running deconfliction engine with profiler...")
    
    profiler = cProfile.Profile()
    
    profiler.enable()
    result = check_conflicts_hybrid(
        primary_mission,
        simulated_flights,
        primary_drone_speed,
        config["deconfliction_parameters"]
    )
    profiler.disable()

    # --- 4. Print the Results ---
    print("\n--- Benchmark Complete ---")
    print(f"Deconfliction Result: {result['status']}")
    print("\n--- Profiler Report ---")
    
    stats = pstats.Stats(profiler).sort_stats('cumtime')
    stats.print_stats(20)

if __name__ == "__main__":
    run_heavy_benchmark()
