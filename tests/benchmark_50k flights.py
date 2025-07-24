"""
performance_benchmark.py

This script is dedicated to analyzing the performance of the deconfliction
engine under a high-density load. It uses Python's built-in cProfile module
to provide a detailed breakdown of function calls and execution time, allowing
for the identification of performance bottlenecks.
"""

import cProfile
import pstats
import random
import numpy as np
import sys
import os

# Add the project root to the Python path to allow for `src` module imports
# when running this script directly from the 'tests' directory.
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, project_root)

# --- Import project modules ---
from src.data_models import Waypoint, PrimaryMission, SimulatedFlight
from src.deconfliction_logic import check_conflicts_hybrid
from src.utils import calculate_etas

def generate_dense_airspace(num_flights: int) -> list[SimulatedFlight]:
    """Generates a large number of non-conflicting background flights."""
    flights = []
    for i in range(num_flights):
        # Place these flights far away from the primary mission's area of interest (0-1000)
        start_x = random.uniform(5000, 10000)
        start_y = random.uniform(5000, 10000)
        start_z = random.uniform(100, 500)
        
        # Give them short, simple paths
        end_x = start_x + random.uniform(-100, 100)
        end_y = start_y + random.uniform(-100, 100)
        end_z = start_z + random.uniform(-20, 20)
        
        start_t = random.uniform(0, 100)
        end_t = start_t + random.uniform(5, 20)

        flight = SimulatedFlight(
            flight_id=f"BACKGROUND_DRONE_{i}",
            waypoints=[Waypoint(start_x, start_y, start_z), Waypoint(end_x, end_y, end_z)],
            timestamps=[start_t, end_t]
        )
        flights.append(flight)
    return flights

def run_benchmark():
    """
    Sets up and runs the high-density deconfliction scenario, printing a
    performance report.
    """
    print("--- Performance Benchmark ---")
    
    # --- 1. Load Configuration ---
    config = {
        "deconfliction_parameters": {
            "safety_buffer_m": 50.0,
            "grid_bin_size": { "x_m": 100.0, "y_m": 100.0, "z_m": 100.0, "t_s": 10.0 }
        }
    }
    primary_drone_speed = 25.0 # m/s

    # --- 2. Define the Scenario ---
    primary_mission = PrimaryMission(
        waypoints=[Waypoint(0, 0, 100), Waypoint(1000, 0, 100)],
        start_time=0,
        end_time=50
    )
    
    conflicting_flight = SimulatedFlight(
        "CONFLICT_TARGET",
        [Waypoint(500, -20, 100), Waypoint(500, 20, 100)],
        [15, 25]
    )

    # *** INCREASED LOAD ***
    # Generate 50,000 background flights to truly stress the system.
    num_background_flights = 50000
    print(f"Generating a dense airspace with {num_background_flights + 1} total simulated flights...")
    background_flights = generate_dense_airspace(num_background_flights)
    all_simulated_flights = [conflicting_flight] + background_flights
    
    # --- 3. Run the Profiler ---
    print("Running deconfliction engine with profiler...")
    
    profiler = cProfile.Profile()
    
    profiler.enable()
    result = check_conflicts_hybrid(
        primary_mission,
        all_simulated_flights,
        primary_drone_speed,
        config["deconfliction_parameters"]
    )
    profiler.disable()

    # --- 4. Print the Results ---
    print("\n--- Benchmark Complete ---")
    print(f"Deconfliction Result: {result['status']} (Flight ID: {result.get('flight_id')})")
    print("\n--- Profiler Report ---")
    
    stats = pstats.Stats(profiler).sort_stats('cumtime')
    stats.print_stats(20)

if __name__ == "__main__":
    run_benchmark()
