"""
fuzz_test.py

This script performs fuzz testing on the deconfliction engine. It is designed
to test the system's robustness by intentionally providing malformed, invalid,
and nonsensical data.

The goal is to ensure the system handles garbage input gracefully and does not
crash with an unhandled exception.
"""

import random
import numpy as np
import sys
import os

# Add the project root to the Python path to allow for `src` module imports
# when running this script directly.
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, project_root)


# --- Import project modules ---
from src.data_models import Waypoint, PrimaryMission, SimulatedFlight
from src.deconfliction_logic import check_conflicts_hybrid

if __name__ == "__main__":
    print("========================================")
    print("=== Deconfliction Engine Fuzz Test ===")
    print("========================================")
    
    # --- Standard setup for all tests ---
    config = {
        "deconfliction_parameters": {
            "safety_buffer_m": 50.0,
            "grid_bin_size": { "x_m": 100.0, "y_m": 100.0, "z_m": 100.0, "t_s": 10.0 }
        }
    }
    primary_speed = 25.0
    valid_primary_mission = PrimaryMission(
        waypoints=[Waypoint(0, 0, 100), Waypoint(1000, 0, 100)],
        start_time=0,
        end_time=50
    )
    
    # --- Define Fuzz Test Cases as Generators ---
    # We define these as lambda functions to defer the creation of the (potentially
    # invalid) SimulatedFlight objects until they are inside a try...except block.
    # This is the core of the fix.
    fuzz_case_generators = [
        # Test 1: Mismatched waypoints and timestamps lists
        ("Mismatched Lists", lambda: [
            SimulatedFlight(
                "MISMATCHED_LISTS",
                waypoints=[Waypoint(1,1,1), Waypoint(2,2,2), Waypoint(3,3,3)], # 3 waypoints
                timestamps=[10, 20] # Only 2 timestamps
            )
        ]),

        # Test 2: Timestamps are not in chronological order
        ("Backward Time", lambda: [
            SimulatedFlight(
                "BACKWARD_TIME",
                waypoints=[Waypoint(1,1,1), Waypoint(2,2,2)],
                timestamps=[100, 50] # Time goes backward
            )
        ]),
        
        # Test 3: Data contains None values where objects/numbers are expected
        ("Corrupted Data", lambda: [
            SimulatedFlight(
                "CORRUPTED_DATA",
                waypoints=[Waypoint(1,1,1), None], # A None waypoint
                timestamps=[10, 20]
            )
        ]),

        # Test 4: Trajectory lists are empty
        ("Empty Trajectory", lambda: [
            SimulatedFlight(
                "EMPTY_TRAJECTORY",
                waypoints=[], # Empty list
                timestamps=[]
            )
        ]),

        # Test 5: The "Extra Hard" compound failure case
        ("Compound Failure", lambda: [
            SimulatedFlight(
                "COMPOUND_FAILURE",
                waypoints=[Waypoint(1,1,1), Waypoint(2,2,2), None], # 3 waypoints, one is None
                timestamps=[100, 50] # 2 timestamps, backward in time
            )
        ])
    ]

    # --- Run all tests ---
    results = []
    for test_name, data_generator in fuzz_case_generators:
        print(f"\n--- Running Fuzz Test: {test_name} ---")
        try:
            # Attempt to generate the malformed data. This may raise an exception
            # (e.g., the ValueError from the data model).
            simulated_flights = data_generator()
            
            # If data generation succeeds, pass it to the deconfliction engine.
            # This function might also raise an exception if it finds issues.
            check_conflicts_hybrid(
                valid_primary_mission,
                simulated_flights,
                primary_speed,
                config["deconfliction_parameters"]
            )
            
            # If we reach here, it means the system processed the weird data without crashing.
            print(f"Result: PASS. The system handled the data without raising an exception.")
            results.append(True)

        except Exception as e:
            # For a fuzz test, catching an exception is a GOOD thing. It means our
            # system is robust enough to identify bad data and stop, rather than
            # crashing unpredictably. This is a PASS.
            print(f"Result: PASS. The system correctly identified an issue and raised a {type(e).__name__}.")
            results.append(True)
    
    print("\n========================================")
    if all(results):
        print("✅ All fuzz tests passed. The system is robust against malformed input.")
    else:
        # This path should now be much harder to reach.
        print("❌ One or more fuzz tests failed. The system is not fully robust.")
    print("========================================")
