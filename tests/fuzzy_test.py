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

# --- Import project modules ---
from src.data_models import Waypoint, PrimaryMission, SimulatedFlight
from src.deconfliction_logic import check_conflicts_hybrid

def run_fuzz_test(test_name: str, primary_mission: PrimaryMission, simulated_flights: list, config: dict, primary_speed: float):
    """
    A helper to run a single fuzz test case and report its result.
    """
    print(f"\n--- Running Fuzz Test: {test_name} ---")
    try:
        # We expect this function to run without raising an unhandled exception
        check_conflicts_hybrid(
            primary_mission,
            simulated_flights,
            primary_speed,
            config["deconfliction_parameters"]
        )
        # If we get here, the system did not crash. This is a pass.
        print(f"Result: PASS. The system handled the malformed data gracefully.")
        return True
    except Exception as e:
        # If any unexpected exception occurs, the test fails.
        print(f"Result: FAIL! The system crashed with an unhandled exception: {type(e).__name__}: {e}")
        return False

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
    
    # --- Define Fuzz Test Cases ---

    # Test 1: Mismatched waypoints and timestamps lists
    fuzz_case_1 = [
        SimulatedFlight(
            "MISMATCHED_LISTS",
            waypoints=[Waypoint(1,1,1), Waypoint(2,2,2), Waypoint(3,3,3)], # 3 waypoints
            timestamps=[10, 20] # Only 2 timestamps
        )
    ]

    # Test 2: Timestamps are not in chronological order
    fuzz_case_2 = [
        SimulatedFlight(
            "BACKWARD_TIME",
            waypoints=[Waypoint(1,1,1), Waypoint(2,2,2)],
            timestamps=[100, 50] # Time goes backward
        )
    ]
    
    # Test 3: Data contains None values where objects/numbers are expected
    fuzz_case_3 = [
        SimulatedFlight(
            "CORRUPTED_DATA",
            waypoints=[Waypoint(1,1,1), None], # A None waypoint
            timestamps=[10, 20]
        )
    ]

    # Test 4: Trajectory lists are empty
    fuzz_case_4 = [
        SimulatedFlight(
            "EMPTY_TRAJECTORY",
            waypoints=[], # Empty list
            timestamps=[]
        )
    ]

    # Test 5: The "Extra Hard" compound failure case
    fuzz_case_5 = [
        SimulatedFlight(
            "COMPOUND_FAILURE",
            waypoints=[Waypoint(1,1,1), Waypoint(2,2,2), None], # 3 waypoints, one is None
            timestamps=[100, 50] # 2 timestamps, backward in time
        )
    ]

    # --- Run all tests ---
    results = []
    results.append(run_fuzz_test("Mismatched Lists", valid_primary_mission, fuzz_case_1, config, primary_speed))
    results.append(run_fuzz_test("Backward Time", valid_primary_mission, fuzz_case_2, config, primary_speed))
    results.append(run_fuzz_test("Corrupted Data", valid_primary_mission, fuzz_case_3, config, primary_speed))
    results.append(run_fuzz_test("Empty Trajectory", valid_primary_mission, fuzz_case_4, config, primary_speed))
    results.append(run_fuzz_test("Compound Failure", valid_primary_mission, fuzz_case_5, config, primary_speed))
    
    print("\n========================================")
    if all(results):
        print("✅ All fuzz tests passed. The system is robust against malformed input.")
    else:
        print("❌ One or more fuzz tests failed. The system is not fully robust.")
    print("========================================")

