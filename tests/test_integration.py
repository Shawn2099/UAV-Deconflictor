"""
test_integration.py

This module contains end-to-end integration tests for the entire application.
It tests the "whole program" by running the main application logic and
verifying its console output to ensure all components work together correctly.
"""

import pytest
import sys
import os

# --- Add the project root to the Python path ---
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Import the main runner function from our application
from main import main_runner

def test_full_program_integration(monkeypatch, capsys):
    """
    Tests the full application flow from start to finish.
    - Mocks user input to prevent the script from hanging.
    - Captures all console output.
    - Verifies that the output contains the expected results for key scenarios.
    """
    # 1. Simulate User Input:
    #    When `input()` is called, this will make it behave as if the user typed "n" and pressed Enter.
    monkeypatch.setattr('builtins.input', lambda _: 'n')

    # 2. Run the main application logic
    main_runner()

    # 3. Capture the Console Output
    captured = capsys.readouterr()
    output = captured.out

    # 4. Verify the Output
    # Check for the expected conflict in the showcase scenario
    assert "--- Running Scenario: Showcase Scenario with True Conflict ---" in output
    assert "Result: CONFLICT DETECTED!" in output
    assert "With Flight ID: CONFLICT-VERTICAL" in output

    # Check for the expected time violation
    assert "--- Running Scenario: Edge Case 1: Mission Time Violation ---" in output
    assert "Result: MISSION INVALID!" in output
    assert "Required: 80.00s, Allowed: 70s" in output

    # Check for a known clear scenario
    assert "--- Running Scenario: Edge Case 2: Parallel Near Miss ---" in output
    assert "Result: Mission is CLEAR. No conflicts detected." in output
    
    # Check for the stationary conflict
    assert "--- Running Scenario: Edge Case 4: Stationary Drone Conflict ---" in output
    assert "With Flight ID: HOVERING-DRONE" in output

    print("\nIntegration test passed: Verified key scenario outputs from the full application run.")

