"""
test_advanced_logic.py

[V2 - Enhanced for Mathematical Rigor]
This module uses the `hypothesis` library to perform property-based testing.
It has been upgraded to move beyond simple sampling. It now verifies the engine's
output against a deterministic mathematical calculation of the true minimum
distance for each randomly generated scenario.
"""

import pytest
import numpy as np
import os
import sys
from hypothesis import given, strategies as st, settings, HealthCheck

# --- Add the project root to the Python path ---
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.data_models import Waypoint
from src.deconfliction_logic import check_spatio_temporal_conflict, get_position_on_segment_at_time

# --- Define Strategies for Hypothesis ---
coords = st.floats(min_value=-1000, max_value=1000, allow_nan=False, allow_infinity=False)
waypoints = st.builds(Waypoint, x=coords, y=coords, z=coords)
times = st.floats(min_value=0, max_value=1000, allow_nan=False, allow_infinity=False)

def _verify_true_minimum_distance(pri_p1, pri_p2, pri_t1, pri_t2, sim_p1, sim_p2, sim_t1, sim_t2) -> float:
    """
    A trusted, internal function to deterministically calculate the minimum distance
    between two moving points during their temporal overlap. This serves as the
    "ground truth" for our property-based test.
    """
    overlap_start = max(pri_t1, sim_t1)
    overlap_end = min(pri_t2, sim_t2)

    if overlap_start >= overlap_end:
        return float('inf')

    # Get positions at the start of the overlap
    pos_pri_start = get_position_on_segment_at_time(pri_p1, pri_p2, pri_t1, pri_t2, overlap_start)
    pri_pos_at_overlap_start = np.array([pos_pri_start.x, pos_pri_start.y, pos_pri_start.z])
    pos_sim_start = get_position_on_segment_at_time(sim_p1, sim_p2, sim_t1, sim_t2, overlap_start)
    sim_pos_at_overlap_start = np.array([pos_sim_start.x, pos_sim_start.y, pos_sim_start.z])

    # Calculate velocities
    pri_vel = (np.array([pri_p2.x, pri_p2.y, pri_p2.z]) - np.array([pri_p1.x, pri_p1.y, pri_p1.z])) / (pri_t2 - pri_t1) if pri_t2 > pri_t1 else np.zeros(3)
    sim_vel = (np.array([sim_p2.x, sim_p2.y, sim_p2.z]) - np.array([sim_p1.x, sim_p1.y, sim_p1.z])) / (sim_t2 - sim_t1) if sim_t2 > sim_t1 else np.zeros(3)

    relative_pos = sim_pos_at_overlap_start - pri_pos_at_overlap_start
    relative_vel = sim_vel - pri_vel

    # Find time of closest approach (TCA) from the start of the overlap
    dot_rel_vel = np.dot(relative_vel, relative_vel)
    if dot_rel_vel < 1e-9:
        tca_from_overlap_start = 0.0
    else:
        tca_from_overlap_start = -np.dot(relative_pos, relative_vel) / dot_rel_vel

    # Check distance at the boundaries of the overlap window
    min_dist_sq = np.dot(relative_pos, relative_pos) # Distance at overlap_start
    pos_at_end = relative_pos + relative_vel * (overlap_end - overlap_start)
    dist_sq_end = np.dot(pos_at_end, pos_at_end)
    if dist_sq_end < min_dist_sq:
        min_dist_sq = dist_sq_end

    # Check distance at TCA if it falls within the overlap duration
    if 0 < tca_from_overlap_start < (overlap_end - overlap_start):
        pos_at_tca = relative_pos + relative_vel * tca_from_overlap_start
        dist_sq_tca = np.dot(pos_at_tca, pos_at_tca)
        if dist_sq_tca < min_dist_sq:
            min_dist_sq = dist_sq_tca
            
    return np.sqrt(min_dist_sq)

@settings(suppress_health_check=[HealthCheck.too_slow], deadline=None, max_examples=500)
@given(
    pri_p1=waypoints, pri_p2=waypoints,
    sim_p1=waypoints, sim_p2=waypoints,
    t_start=times,
    pri_duration=st.floats(min_value=1, max_value=100),
    sim_duration=st.floats(min_value=1, max_value=100),
    time_offset=st.floats(min_value=-50, max_value=50)
)
def test_spatio_temporal_conflict_property(
    pri_p1, pri_p2, sim_p1, sim_p2, t_start, pri_duration, sim_duration, time_offset
):
    """
    This property-based test now verifies two critical properties with mathematical rigor:
    1. If a conflict is reported, it MUST be real.
    2. If no conflict is reported, a trusted verification function MUST agree.
    """
    safety_buffer = 50.0
    
    pri_t1, pri_t2 = t_start, t_start + pri_duration
    sim_t1, sim_t2 = t_start + time_offset, t_start + time_offset + sim_duration

    # Run the function we want to test
    conflict_result = check_spatio_temporal_conflict(
        pri_p1, pri_p2, pri_t1, pri_t2,
        sim_p1, sim_p2, sim_t1, sim_t2,
        safety_buffer
    )

    if conflict_result:
        # PROPERTY 1: If a conflict is reported, verify it is real.
        conflict_time = conflict_result['time']
        pos1 = get_position_on_segment_at_time(pri_p1, pri_p2, pri_t1, pri_t2, conflict_time)
        pos2 = get_position_on_segment_at_time(sim_p1, sim_p2, sim_t1, sim_t2, conflict_time)
        actual_distance = np.linalg.norm(np.array([pos1.x, pos1.y, pos1.z]) - np.array([pos2.x, pos2.y, pos2.z]))
        assert actual_distance < safety_buffer
    else:
        # PROPERTY 2: If no conflict is reported, verify with our trusted calculation.
        true_min_dist = _verify_true_minimum_distance(
            pri_p1, pri_p2, pri_t1, pri_t2,
            sim_p1, sim_p2, sim_t1, sim_t2
        )
        # The true minimum distance must be greater than or equal to the buffer.
        # We use a small tolerance for floating point comparisons.
        assert true_min_dist >= safety_buffer - 1e-9

