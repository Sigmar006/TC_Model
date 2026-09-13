#!/usr/bin/env bash
set -e

echo "Setting up Python virtual environment..."
VENV_DIR="tools/.venv"
if [ ! -d "$VENV_DIR" ]; then
    python3 -m venv "$VENV_DIR"
fi
source "$VENV_DIR/bin/activate"
pip install -q pandas matplotlib

echo "Cleaning up old trajectories..."
rm -rf /tmp/1q_trajectories
mkdir -p /tmp/1q_trajectories

echo "Running AircraftManeuverTest to generate trajectories..."
export DUMP_MANEUVER_TRAJECTORY=1

./build/llvm-ninja-debug/bin/1q_flight_dynamic_known_limit_tests \
  --gtest_filter="*AircraftManeuverTest.*"

echo "Plotting trajectories..."
python tools/plot_maneuvers.py

echo "Done! Check docs/maneuvers/ for outputs."
