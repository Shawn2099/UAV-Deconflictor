"""
visualization.py

[V5 - Final Version]
This module provides functions to visualize drone mission scenarios in 3D space.
This version fixes the legend warning and improves performance by only
animating the relevant time window of the primary mission.

Key Features:
- Modern "holographic" dark theme.
- Dynamic axis limits that automatically fit any scenario.
- Enhanced drone visuals with ground-plane shadows for better 3D perception.
- Pulsating, glowing conflict sphere to clearly highlight danger zones.
- Optimized animation loop for smoother performance.
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
        self.timestamps = np.array(timestamps)
        self.color = color
        self.marker = marker
        self.label = label
        self.path_coords = np.array([[wp.x, wp.y, wp.z] for wp in waypoints]) if waypoints else np.empty((0, 3))
        self.path_line, self.point_marker, self.shadow_marker = None, None, None

    def get_position(self, time: float) -> Optional[np.ndarray]:
        """Calculates the drone's 3D position at a specific time using optimized search."""
        if self.timestamps.size == 0: return None
        clamped_time = np.clip(time, self.timestamps[0], self.timestamps[-1])
        segment_index = np.searchsorted(self.timestamps, clamped_time, side='right') - 1
        segment_index = np.clip(segment_index, 0, len(self.timestamps) - 2)
        t1, t2 = self.timestamps[segment_index], self.timestamps[segment_index + 1]
        p1, p2 = self.path_coords[segment_index], self.path_coords[segment_index + 1]
        if t1 == t2: return p1
        progress = (clamped_time - t1) / (t2 - t1)
        return p1 + progress * (p2 - p1)

def _setup_plot_aesthetics(ax, all_waypoints):
    """Applies styling and calculates dynamic axis limits."""
    plt.style.use('dark_background')
    ax.set_facecolor('#0a0a1a')
    ax.grid(True, color='w', linestyle='--', linewidth=0.5, alpha=0.1)
    ax.set_xlabel("X (meters)", fontsize=10, color='gray')
    ax.set_ylabel("Y (meters)", fontsize=10, color='gray')
    ax.set_zlabel("Z (Altitude)", fontsize=10, color='gray')
    ax.set_title("4D Airspace Deconfliction Simulation", fontsize=16, pad=20)
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    if not all_waypoints:
        ax.set_xlim([0, 1000]); ax.set_ylim([0, 1000]); ax.set_zlim([0, 500])
        return
    coords = np.array([[wp.x, wp.y, wp.z] for wp in all_waypoints])
    min_coords, max_coords = coords.min(axis=0), coords.max(axis=0)
    center, span = (min_coords + max_coords) / 2, (max_coords - min_coords) * 1.5
    span[span < 200] = 200
    ax.set_xlim(center[0] - span[0]/2, center[0] + span[0]/2)
    ax.set_ylim(center[1] - span[1]/2, center[1] + span[1]/2)
    ax.set_zlim(max(0, center[2] - span[2]/2), center[2] + span[2]/2)

def create_4d_animation(
    primary_mission: PrimaryMission, 
    simulated_flights: List[SimulatedFlight],
    primary_mission_etas: List[float],
    conflict_result: Dict[str, Any],
    config: Dict[str, Any],
    output_filename: str
):
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    all_wps = list(primary_mission.waypoints)
    for f in simulated_flights: all_wps.extend(f.waypoints)
    _setup_plot_aesthetics(ax, all_wps)

    drones = [DynamicDrone(primary_mission.waypoints, primary_mission_etas, '#00ffff', 'o', 'Primary Mission')]
    sim_colors = plt.cm.get_cmap('plasma', len(simulated_flights))
    for i, flight in enumerate(simulated_flights):
        drones.append(DynamicDrone(flight.waypoints, flight.timestamps, sim_colors(i), 's', flight.flight_id))

    for drone in drones:
        if drone.path_coords.size > 0:
            path = drone.path_coords.T
            drone.path_line, = ax.plot(path[0], path[1], path[2], '--', color=drone.color, linewidth=1, alpha=0.4)
            ax.plot(path[0], path[1], np.zeros_like(path[2]), ':', color='gray', linewidth=0.8, alpha=0.3)
        drone.point_marker, = ax.plot([], [], [], drone.marker, color=drone.color, markersize=8, markeredgecolor='white', markeredgewidth=0.5, label=drone.label)
        drone.shadow_marker, = ax.plot([], [], [], drone.marker, color='gray', markersize=4, alpha=0.5)

    conflict_sphere = None
    if conflict_result and conflict_result.get("status") == "CONFLICT":
        loc = conflict_result["location"]
        u, v = np.mgrid[0:2*np.pi:40j, 0:np.pi:20j]
        x = loc['x'] + config['deconfliction_parameters']['safety_buffer_m'] * np.cos(u) * np.sin(v)
        y = loc['y'] + config['deconfliction_parameters']['safety_buffer_m'] * np.sin(u) * np.sin(v)
        z = loc['z'] + config['deconfliction_parameters']['safety_buffer_m'] * np.cos(v)
        conflict_sphere = ax.plot_surface(x, y, z, color='#ff4444', alpha=0.1, zorder=0)
        ax.scatter(loc['x'], loc['y'], loc['z'], color='#ff0000', s=200, marker='X', label="Conflict Point", zorder=11, edgecolors='white')

    time_text = ax.text2D(0.02, 0.95, '', transform=ax.transAxes, fontsize=14, color='white', ha='left')
    status_text = ax.text2D(0.98, 0.95, '', transform=ax.transAxes, fontsize=14, color='white', ha='right')
    ax.legend(loc='upper left', bbox_to_anchor=(0.02, 0.9)) # This now works correctly
    
    # --- PERFORMANCE FIX: Animate only up to the end of the primary mission ---
    max_time = primary_mission_etas[-1] * 1.05 # Add 5% buffer

    def update(frame_time):
        # ... (update logic remains the same)
        artists = []
        for drone in drones:
            pos = drone.get_position(frame_time)
            if pos is not None:
                drone.point_marker.set_data_3d([pos[0]], [pos[1]], [pos[2]])
                drone.shadow_marker.set_data_3d([pos[0]], [pos[1]], [0])
                drone.point_marker.set_visible(True)
                drone.shadow_marker.set_visible(True)
                artists.extend([drone.point_marker, drone.shadow_marker])
            else:
                drone.point_marker.set_visible(False)
                drone.shadow_marker.set_visible(False)
        time_text.set_text(f'Time: {frame_time:.1f}s')
        artists.append(time_text)
        if conflict_sphere and conflict_result['time'] >= frame_time - 1 and conflict_result['time'] <= frame_time + 1:
            alpha_pulse = 0.3 + 0.2 * np.sin(frame_time * 5)
            conflict_sphere.set_alpha(alpha_pulse)
            status_text.set_text('CONFLICT IMMINENT')
            status_text.set_color('#ff4444')
            artists.extend([conflict_sphere, status_text])
        elif conflict_result and conflict_result.get("status") == "CONFLICT" and frame_time > conflict_result['time']:
             status_text.set_text('CONFLICT DETECTED')
             status_text.set_color('#ff4444')
        else:
            status_text.set_text('CLEAR')
            status_text.set_color('#00ff00')
        artists.append(status_text)
        return artists
    
    FPS, DPI = 12, 100
    frames = np.linspace(0, max_time, num=int(max_time * FPS))
    anim = FuncAnimation(fig, update, frames=frames, interval=1000/FPS, blit=True)
    print(f"Generating animation ({len(frames)} frames at {DPI} DPI): '{output_filename}'... This may take a moment.")
    anim.save(output_filename, writer='pillow', fps=FPS, dpi=DPI)
    print("Animation saved.")
    plt.close(fig)
