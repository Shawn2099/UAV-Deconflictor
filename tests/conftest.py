"""
conftest.py

This file is a special configuration file for the pytest framework.
It MUST contain only valid Python code.

This configuration achieves two goals for a professional testing setup:
1.  It creates a custom command-line flag, `--runslow`, so we can choose to run
    our time-consuming performance tests.
2.  It automatically skips any test marked as 'slow' unless the `--runslow` flag
    is explicitly used. This keeps the default test runs fast.
"""

# Import the pytest library, which is necessary for configuration.
import pytest

def pytest_addoption(parser):
    """
    This is a special pytest function that adds custom command-line options.
    We are adding a boolean flag named '--runslow'.
    """
    parser.addoption(
        "--runslow",                # The name of the flag.
        action="store_true",        # Makes it a flag (it's either present or not).
        default=False,              # If the flag isn't used, its value is False.
        help="run slow tests"       # A help message for the flag.
    )

def pytest_configure(config):
    """
    This is a special pytest function for initial configuration.
    We use it to officially register our custom 'slow' marker so pytest
    doesn't produce warnings.
    """
    config.addinivalue_line(
        "markers", "slow: marks tests as slow to run"
    )

def pytest_collection_modifyitems(config, items):
    """
    This is a special pytest function that runs after tests have been collected.
    We use it to decide whether to run or skip the 'slow' tests.
    """
    # Check if the '--runslow' flag was NOT provided on the command line.
    if not config.getoption("--runslow"):
        # If the flag is missing, we need to skip the slow tests.
        skip_slow = pytest.mark.skip(reason="need --runslow option to run this test")
        for item in items:
            # Check if the test item has the 'slow' keyword (from @pytest.mark.slow).
            if "slow" in item.keywords:
                # If it does, add the 'skip' marker to it.
                item.add_marker(skip_slow)
