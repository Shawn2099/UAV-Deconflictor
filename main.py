"""
main.py

This is the main entry point for the UAV Strategic Deconfliction application.
It sets up different mission scenarios, runs the deconfliction checks,
and generates visualizations of the outcomes.

[CORRECTED VERSION] - Adjusts scenarios for clarity and saves plots to files.
"""

# Import the necessary functions and classes from our 'src' package
from src.data_models import Waypoint, PrimaryMission, SimulatedFlight
from src.deconfliction_logic import check_mission_conflicts
from src.visualization import create_2d_plot

def run_scenario(scenario_name: str, primary_mission: PrimaryMission, simulated_flights: list, safety_buffer: float):
    """
    A helper function to run and report on a single deconfliction scenario.
    """
    print(f"\n--- Running Scenario: {scenario_name} ---")
    
    # Perform the conflict check
    conflict_result = check_mission_conflicts(primary_mission, simulated_flights, safety_buffer)
    
    # Print the results to the console
    if conflict_result["conflict"]:
        print(f"Conflict Detected!")
        print(f"  - With Flight ID: {conflict_result['flight_id']}")
        print(f"  - Approx. Location (x,y): ({conflict_result['location']['x']:.2f}, {conflict_result['location']['y']:.2f})")
        print(f"  - Approx. Time: {conflict_result['time']:.2f}")
    else:
        print("Mission is Clear. No conflicts detected.")
        
    # Generate and save the visualization to a file
    output_filename = f"{scenario_name.replace(' ', '_').lower()}.png"
    create_2d_plot(primary_mission, simulated_flights, conflict_result, output_filename)


if __name__ == "__main__":
    # Define a constant safety buffer for all scenarios
    SAFETY_BUFFER = 50.0  # 50 meters

    # --- SCENARIO 1: A mission that is now TRULY clear of conflicts ---
    # The simulated flight's time window is moved to be completely after the primary mission's window.
    mission_1 = PrimaryMission(
        waypoints=[Waypoint(x=0, y=0), Waypoint(x=500, y=500)],
        start_time=0,
        end_time=100
    )
    sim_flights_1 = [
        SimulatedFlight(
            flight_id="SF-01",
            waypoints=[Waypoint(x=0, y=500), Waypoint(x=500, y=0)],
            # Timestamps are now outside the primary mission's 0-100s window
            timestamps=[110, 200] 
        )
    ]
    run_scenario("Mission Clear", mission_1, sim_flights_1, SAFETY_BUFFER)


    # --- SCENARIO 2: A mission with a clear spatio-temporal conflict ---
    # This scenario remains the same, as it was already a valid conflict.
    mission_2 = PrimaryMission(
        waypoints=[Waypoint(x=100, y=100), Waypoint(x=400, y=400)],
        start_time=0,
        end_time=100
    )
    sim_flights_2 = [
        SimulatedFlight(
            flight_id="SF-02",
            waypoints=[Waypoint(x=100, y=400), Waypoint(x=400, y=100)],
            # These timings are designed to overlap with the primary mission
            timestamps=[20, 80] 
        )
    ]
    run_scenario("Conflict Detected", mission_2, sim_flights_2, SAFETY_BUFFER)
