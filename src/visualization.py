"""
visualization.py

[V8 - Final, Configurable, and Synchronized]
This module provides functions to visualize drone mission scenarios in 3D space.
It has been updated to be fully compatible with the final, configurable main.py.

Key Features:
- Accepts a 'config' dictionary to get parameters like safety_buffer.
- Draws a correctly scaled conflict sphere based on the safety_buffer.
- Retains the robust DynamicDrone class for managing drone states over time.
- Uses a fixed visualization region for consistency.
"""
import matplotlib
matplotlib.use('Agg') # Use Agg backend to prevent GUI errors in non-GUI environments
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from typing import List, Dict, Any, Optional
import numpy as np

# Import our custom data models
from .data_models import PrimaryMission, SimulatedFlight, Waypoint

class DynamicDrone:
    """A helper class to manage the state and position of a drone over time."""
    def __init__(self, waypoints: List[Waypoint], timestamps: List[float], color: str, marker: str, label: str):
        self.waypoints = waypoints
        self.timestamps = timestamps
        self.color = color
        self.marker = marker
        self.label = label
        self.path_line = None
        self.point_marker = None

    def get_position(self, time: float) -> Optional[np.ndarray]:
        """Calculates the drone's 3D position at a specific time."""
        if not self.timestamps or not self.waypoints:
            return None

        clamped_time = max(self.timestamps[0], min(self.timestamps[-1], time))

        for i in range(len(self.timestamps) - 1):
            t1, t2 = self.timestamps[i], self.timestamps[i+1]
            if t1 <= clamped_time <= t2:
                p1_wp, p2_wp = self.waypoints[i], self.waypoints[i+1]
                if t1 == t2:
                    return np.array([p1_wp.x, p1_wp.y, p1_wp.z])
                
                progress = (clamped_time - t1) / (t2 - t1)
                x = p1_wp.x + progress * (p2_wp.x - p1_wp.x)
                y = p1_wp.y + progress * (p2_wp.y - p1_wp.y)
                z = p1_wp.z + progress * (p2_wp.z - p1_wp.z)
                return np.array([x, y, z])
        
        # If time is outside the segments (but clamped), return the last waypoint's position
        return np.array([self.waypoints[-1].x, self.waypoints[-1].y, self.waypoints[-1].z])

def create_4d_animation(
    primary_mission: PrimaryMission, 
    simulated_flights: List[SimulatedFlight],
    primary_mission_etas: List[float],
    conflict_result: Dict[str, Any],
    config: Dict[str, Any], # Correctly accept the config dictionary
    output_filename: str
):
    """
    Generates and saves a 4D (3D + time) animation of the airspace scenario.
    """
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title("4D Airspace Deconfliction", fontsize=16)
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.set_zlabel("Z (Altitude in meters)")

    # Extract safety buffer from config for drawing the conflict sphere
    safety_buffer = config.get('safety_buffer_m', 50.0)

    drones = [DynamicDrone(primary_mission.waypoints, primary_mission_etas, 'blue', 'o', 'Primary Mission')]
    
    # Use a color map for simulated drones
    sim_colors = plt.cm.get_cmap('gist_rainbow', len(simulated_flights))
    for i, flight in enumerate(simulated_flights):
        drones.append(DynamicDrone(flight.waypoints, flight.timestamps, sim_colors(i), 's', flight.flight_id))

    # Set fixed axis limits for a consistent visualization region
    ax.set_xlim([0, 1000]); ax.set_ylim([0, 1000]); ax.set_zlim([0, 500])

    # Plot static paths and initialize dynamic markers
    for drone in drones:
        x_path = [wp.x for wp in drone.waypoints]
        y_path = [wp.y for wp in drone.waypoints]
        z_path = [wp.z for wp in drone.waypoints]
        drone.path_line, = ax.plot(x_path, y_path, z_path, '--', color=drone.color, linewidth=1, alpha=0.5)
        drone.point_marker, = ax.plot([], [], [], drone.marker, color=drone.color, markersize=8, label=drone.label)

    # Highlight the conflict zone with a correctly scaled transparent sphere
    if conflict_result and conflict_result.get("conflict"):
        loc = conflict_result["location"]
        u = np.linspace(0, 2 * np.pi, 100)
        v = np.linspace(0, np.pi, 100)
        x = loc['x'] + safety_buffer * np.outer(np.cos(u), np.sin(v))
        y = loc['y'] + safety_buffer * np.outer(np.sin(u), np.sin(v))
        z = loc['z'] + safety_buffer * np.outer(np.ones(np.size(u)), np.cos(v))
        ax.plot_surface(x, y, z, color='red', alpha=0.15, zorder=0)
        ax.scatter(loc['x'], loc['y'], loc['z'], color='red', s=150, marker='X', label="Conflict Point", zorder=11)

    time_text = ax.text2D(0.05, 0.95, '', transform=ax.transAxes, fontsize=14)
    ax.legend(loc='upper right', bbox_to_anchor=(1.15, 1.0))
    
    all_end_times = [d.timestamps[-1] for d in drones if d.timestamps]
    max_time = max(all_end_times) if all_end_times else 100

    def update(frame_time):
        for drone in drones:
            pos = drone.get_position(frame_time)
            if pos is not None:
                drone.point_marker.set_data_3d([pos[0]], [pos[1]], [pos[2]])
                drone.point_marker.set_visible(True)
            else:
                drone.point_marker.set_visible(False)
        time_text.set_text(f'Time: {frame_time:.2f}s')
        return [d.point_marker for d in drones] + [time_text]

    frames = np.linspace(0, max_time, num=int(max_time * 10)) # 10 FPS animation
    anim = FuncAnimation(fig, update, frames=frames, interval=100, blit=False)

    print(f"Generating animation: '{output_filename}'... This may take a moment.")
    anim.save(output_filename, writer='pillow', fps=10)
    print("Animation saved.")
    plt.close(fig)
