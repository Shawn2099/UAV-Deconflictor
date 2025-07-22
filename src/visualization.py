"""
visualization.py

This module provides functions to visualize the drone mission scenarios.
For the 2D MVP, it generates a Matplotlib plot showing the paths of the
primary mission and simulated flights, highlighting any detected conflicts.

[CORRECTED VERSION] - Sets a non-interactive backend and saves plot to file.
"""
import matplotlib
# Set the backend to 'Agg' to prevent GUI-related errors.
# This MUST be done before importing pyplot.
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from typing import List, Dict, Any, Optional

# Import our custom data models
from .data_models import PrimaryMission, SimulatedFlight

def create_2d_plot(
    primary_mission: PrimaryMission, 
    simulated_flights: List[SimulatedFlight], 
    conflict_details: Optional[Dict[str, Any]] = None,
    output_filename: Optional[str] = None
):
    """
    Generates and saves a 2D plot of the airspace scenario.

    Args:
        primary_mission: The primary drone's mission plan.
        simulated_flights: A list of other drones' flight plans.
        conflict_details: An optional dictionary containing conflict information.
                          If provided and a conflict exists, it will be highlighted.
        output_filename: The path to save the generated plot image. If None,
                         the plot is not saved.
    """
    # 1. Set up the plot canvas
    fig, ax = plt.subplots(figsize=(12, 12))
    ax.set_title("2D Airspace Visualization", fontsize=16)
    ax.set_xlabel("X Coordinate (meters)", fontsize=12)
    ax.set_ylabel("Y Coordinate (meters)", fontsize=12)
    ax.set_aspect('equal', adjustable='box') # Ensures correct spatial representation
    ax.grid(True, which='both', linestyle='--', linewidth=0.5)

    # 2. Plot the primary mission's path
    primary_x = [wp.x for wp in primary_mission.waypoints]
    primary_y = [wp.y for wp in primary_mission.waypoints]
    ax.plot(primary_x, primary_y, 'o-', color='blue', linewidth=2, markersize=8, label="Primary Mission")
    ax.text(primary_x[0], primary_y[0], ' Start', color='blue', fontsize=10, verticalalignment='bottom')

    # 3. Plot the simulated flights' paths
    for flight in simulated_flights:
        sim_x = [wp.x for wp in flight.waypoints]
        sim_y = [wp.y for wp in flight.waypoints]
        ax.plot(sim_x, sim_y, 's--', color='gray', linewidth=1.5, markersize=6, label=f"Sim Flight: {flight.flight_id}")
        ax.text(sim_x[0], sim_y[0], f' {flight.flight_id}', color='black', fontsize=9, verticalalignment='bottom')

    # 4. Highlight the conflict point if one exists
    if conflict_details and conflict_details.get("conflict"):
        loc = conflict_details["location"]
        ax.plot(loc["x"], loc["y"], 'X', color='red', markersize=25, markeredgewidth=3, label="Conflict Point")
        
        # Add an annotation for clarity
        conflict_text = f"Conflict with {conflict_details['flight_id']}"
        ax.annotate(
            conflict_text,
            xy=(loc["x"], loc["y"]),
            xytext=(loc["x"] + 50, loc["y"] + 50), # Offset the text
            textcoords='offset points',
            arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2", color='red'),
            bbox=dict(boxstyle="round,pad=0.5", fc="yellow", ec="black", lw=1, alpha=0.8),
            fontsize=10,
            color='black'
        )

    # 5. Save the plot to a file if a filename is provided
    ax.legend()
    if output_filename:
        plt.savefig(output_filename)
        print(f"Plot saved to '{output_filename}'")
    
    # Close the figure to free up memory and prevent it from trying to render
    plt.close(fig)
