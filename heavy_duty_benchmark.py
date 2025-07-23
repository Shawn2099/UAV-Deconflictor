"""
heavy_duty_benchmark.py

[V4 - The Final "Swarm" Test]
This script performs the ultimate stress test on the deconfliction engine.
It simulates a massive, dense swarm of 10,000 drones and forces the engine
to perform a narrow-phase check on approximately 600 potential candidates.
"""

import cProfile
import pstats
import random
import time

# --- Import project modules ---
from src.data_models import Waypoint, PrimaryMission, SimulatedFlight
from src.deconfliction_logic import check_conflicts_hybrid

def generate_swarm(num_flights: int) -> list[SimulatedFlight]:
    """
    Generates a dense, wide cluster of drones to produce a specific
    number of potential candidates.
    """
    flights = []
    
    center_x, center_y, center_z = 500, 0, 100
    center_t = 20

    for i in range(num_flights):
        # By increasing the Y-axis randomness, we can tune the candidate count
        start_x = center_x + random.uniform(-200, 200)
        start_y = center_y + random.uniform(-400, 400) # Wider cluster
        start_z = center_z + random.uniform(-50, 50)
        start_t = center_t + random.uniform(-10, 10)

        end_x = center_x + random.uniform(-200, 200)
        end_y = center_y + random.uniform(-400, 400) # Wider cluster
        end_z = center_z + random.uniform(-50, 50)
        end_t = start_t + random.uniform(5, 10)

        flight = SimulatedFlight(
            flight_id=f"SWARM_DRONE_{i}",
            waypoints=[Waypoint(start_x, start_y, start_z), Waypoint(end_x, end_y, end_z)],
            timestamps=[start_t, end_t]
        )
        flights.append(flight)
    return flights

def run_heavy_benchmark():
    """
    Sets up and runs the high-complexity deconfliction scenario.
    """
    print("--- Heavy-Duty Performance Benchmark (Swarm Test) ---")
    
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
    
    guaranteed_conflict = SimulatedFlight(
        "GUARANTEED_CONFLICT",
        [Waypoint(500, 0, 100), Waypoint(500, 0, 100)],
        [20, 21]
    )

    # *** INCREASED LOAD TO 10,000 ***
    num_swarm_drones = 10000
    print(f"Generating a 'swarm' with {num_swarm_drones + 1} total simulated flights...")
    
    start_gen_time = time.time()
    # The conflict is still at the END of the list to force a full check
    simulated_flights = generate_swarm(num_swarm_drones) + [guaranteed_conflict]
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
    print(f"Deconfliction Result: {result['status']} (Flight ID: {result.get('flight_id')})")
    print("\n--- Profiler Report ---")
    
    stats = pstats.Stats(profiler).sort_stats('cumtime')
    stats.print_stats(20)

if __name__ == "__main__":
    run_heavy_benchmark()
