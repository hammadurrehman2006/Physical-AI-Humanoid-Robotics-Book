"""
Test configuration file for Isaac Robot Brain tests.
"""
import pytest
import sys
import os

# Add the src directory to the path so tests can import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))