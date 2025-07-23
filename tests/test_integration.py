"""
test_integration.py

[V2 - Corrected]
This file contains integration tests for the full application flow.
It has been updated to work with the new argparse-based entrypoint.
"""

import pytest
import json
from argparse import Namespace

# Import the main entrypoint function
from main import main_runner

@pytest.fixture
def create_test_files(tmp_path):
    """A pytest fixture to create temporary config and scenario files for testing."""
    config_content = {
        "deconfliction_parameters": {
            "safety_buffer_m": 50.0,
            "grid_bin_size": {"x_m": 100.0, "y_m": 100.0, "z_m": 100.0, "t_s": 10.0}
        },
        "drone_performance_profiles": {
            "Test_Drone": {"speed_mps": 20.0}
        }
    }
    
    scenarios_content = {
        "scenarios": [
            {
                "name": "Test Clear Scenario",
                "primary_mission": {
                    "drone_model": "Test_Drone",
                    "start_time": 0,
                    "end_time": 100,
                    "waypoints": [{"x": 0, "y": 0, "z": 10}, {"x": 100, "y": 0, "z": 10}]
                },
                "simulated_flights": []
            },
            {
                "name": "Test Conflict Scenario",
                "primary_mission": {
                    "drone_model": "Test_Drone",
                    "start_time": 0,
                    "end_time": 100,
                    "waypoints": [{"x": 0, "y": 0, "z": 10}, {"x": 200, "y": 0, "z": 10}]
                },
                "simulated_flights": [{
                    "flight_id": "CONFLICT01",
                    "waypoints": [{"x": 100, "y": -50, "z": 10}, {"x": 100, "y": 50, "z": 10}],
                    "timestamps": [0, 10]
                }]
            }
        ]
    }

    config_file = tmp_path / "config.json"
    scenarios_file = tmp_path / "scenarios.json"

    with open(config_file, 'w') as f:
        json.dump(config_content, f)
    with open(scenarios_file, 'w') as f:
        json.dump(scenarios_content, f)
        
    return config_file, scenarios_file

def test_full_program_integration(capsys, create_test_files):
    """
    Tests the full application flow from start to finish using argparse.
    - Uses temporary files for config and scenarios.
    - Captures all console output.
    - Verifies that the output contains the expected results for key scenarios.
    """
    config_path, scenarios_path = create_test_files

    # 1. Simulate Command-Line Arguments:
    #    Create a 'Namespace' object that mimics the output of argparse.
    args = Namespace(
        config=str(config_path),
        scenarios=str(scenarios_path),
        visualize=False  # We don't need to generate GIFs during the test
    )

    # 2. Run the main application logic with the simulated arguments
    main_runner(args)

    # 3. Capture and Assert Output:
    #    Check that the console output contains the expected status for each scenario.
    captured = capsys.readouterr()
    output = captured.out

    assert "--- Running Scenario: Test Clear Scenario ---" in output
    assert "Result: Mission is CLEAR. No conflicts detected." in output
    
    assert "--- Running Scenario: Test Conflict Scenario ---" in output
    assert "Result: CONFLICT DETECTED!" in output
    assert "With Flight ID: CONFLICT01" in output

