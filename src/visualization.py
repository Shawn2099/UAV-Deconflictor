"""
visualization.py

[V2 - 3D Enabled] This module provides functions to visualize the drone mission
scenarios in 3D space.
"""
import matplotlib
# Set the backend to 'Agg' to prevent GUI-related errors.
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from typing import List, Dict, Any, Optional

# Import our custom data models
from .data_models import PrimaryMission, SimulatedFlight

def create_3d_plot(
    primary_mission: PrimaryMission, 
    simulated_flights: List[SimulatedFlight], 
    conflict_details: Optional[Dict[str, Any]] = None,
    output_filename: Optional[str] = None
):
    """
    Generates and saves a 3D plot of the airspace scenario.

    Args:
        primary_mission: The primary drone's mission plan.
        simulated_flights: A list of other drones' flight plans.
        conflict_details: An optional dictionary containing conflict information.
        output_filename: The path to save the generated plot image.
    """
    # 1. Set up the 3D plot canvas
    fig = plt.figure(figsize=(14, 14))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title("3D Airspace Visualization", fontsize=16)
    ax.set_xlabel("X Coordinate (meters)", fontsize=12)
    ax.set_ylabel("Y Coordinate (meters)", fontsize=12)
    ax.set_zlabel("Z Coordinate (Altitude in meters)", fontsize=12)

    # 2. Plot the primary mission's path
    px = [wp.x for wp in primary_mission.waypoints]
    py = [wp.y for wp in primary_mission.waypoints]
    pz = [wp.z for wp in primary_mission.waypoints]
    ax.plot(px, py, pz, 'o-', color='blue', linewidth=2, markersize=8, label="Primary Mission")
    ax.text(px[0], py[0], pz[0], ' Start', color='blue', fontsize=10)

    # 3. Plot the simulated flights' paths
    for flight in simulated_flights:
        sx = [wp.x for wp in flight.waypoints]
        sy = [wp.y for wp in flight.waypoints]
        sz = [wp.z for wp in flight.waypoints]
        ax.plot(sx, sy, sz, 's--', color='gray', linewidth=1.5, markersize=6, label=f"Sim Flight: {flight.flight_id}")
        ax.text(sx[0], sy[0], sz[0], f' {flight.flight_id}', color='black', fontsize=9)

    # 4. Highlight the conflict point if one exists
    if conflict_details and conflict_details.get("conflict"):
        loc = conflict_details["location"]
        ax.plot([loc["x"]], [loc["y"]], [loc["z"]], 'X', color='red', markersize=25, markeredgewidth=3, label="Conflict Point")
        
    # Set axis limits to make the plot more readable
    all_x = px + [wp.x for f in simulated_flights for wp in f.waypoints]
    all_y = py + [wp.y for f in simulated_flights for wp in f.waypoints]
    all_z = pz + [wp.z for f in simulated_flights for wp in f.waypoints]
    
    # Set a buffer around the min/max points for better viewing
    x_min, x_max = min(all_x) - 50, max(all_x) + 50
    y_min, y_max = min(all_y) - 50, max(all_y) + 50
    z_min, z_max = min(all_z), max(all_z) + 100 # More buffer for altitude
    
    ax.set_xlim([x_min, x_max])
    ax.set_ylim([y_min, y_max])
    ax.set_zlim([z_min, z_max])

    # 5. Save the plot to a file
    ax.legend()
    if output_filename:
        plt.savefig(output_filename)
        print(f"3D Plot saved to '{output_filename}'")
    
    plt.close(fig)


def create_2d_plot(
    primary_mission: PrimaryMission, 
    simulated_flights: List[SimulatedFlight], 
    conflict_details: Optional[Dict[str, Any]] = None,
    output_filename: Optional[str] = None
):
    """
    Generates and saves a 2D plot of the airspace scenario (X-Y plane).

    Args:
        primary_mission: The primary drone's mission plan.
        simulated_flights: A list of other drones' flight plans.
        conflict_details: An optional dictionary containing conflict information.
        output_filename: The path to save the generated plot image.
    """
    # 1. Set up the 2D plot canvas
    fig, ax = plt.subplots(figsize=(12, 12))
    ax.set_title("2D Airspace Visualization (Top-Down View)", fontsize=16)
    ax.set_xlabel("X Coordinate (meters)", fontsize=12)
    ax.set_ylabel("Y Coordinate (meters)", fontsize=12)
    ax.grid(True, alpha=0.3)

    # 2. Plot the primary mission's path
    px = [wp.x for wp in primary_mission.waypoints]
    py = [wp.y for wp in primary_mission.waypoints]
    ax.plot(px, py, 'o-', color='blue', linewidth=2, markersize=8, label="Primary Mission")
    ax.text(px[0], py[0], ' Start', color='blue', fontsize=10)

    # 3. Plot the simulated flights' paths
    for flight in simulated_flights:
        sx = [wp.x for wp in flight.waypoints]
        sy = [wp.y for wp in flight.waypoints]
        ax.plot(sx, sy, 's--', color='gray', linewidth=1.5, markersize=6, label=f"Sim Flight: {flight.flight_id}")
        ax.text(sx[0], sy[0], f' {flight.flight_id}', color='black', fontsize=9)

    # 4. Highlight the conflict point if one exists
    if conflict_details and conflict_details.get("conflict"):
        loc = conflict_details["location"]
        ax.plot([loc["x"]], [loc["y"]], 'X', color='red', markersize=25, markeredgewidth=3, label="Conflict Point")
        
    # Set axis limits to make the plot more readable
    all_x = px + [wp.x for f in simulated_flights for wp in f.waypoints]
    all_y = py + [wp.y for f in simulated_flights for wp in f.waypoints]
    
    # Set a buffer around the min/max points for better viewing
    x_min, x_max = min(all_x) - 50, max(all_x) + 50
    y_min, y_max = min(all_y) - 50, max(all_y) + 50
    
    ax.set_xlim([x_min, x_max])
    ax.set_ylim([y_min, y_max])
    ax.set_aspect('equal')

    # 5. Save the plot to a file
    ax.legend()
    if output_filename:
        plt.savefig(output_filename)
        print(f"2D Plot saved to '{output_filename}'")
    
    plt.close(fig)

