#!/usr/bin/env bash
# Ask the camera once: is there ink on the board, and where?
# Usage: scripts/lab_check.sh        (camera running, see scripts/lab_start.sh)
cd "$(dirname "$0")/.." || exit 1
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
python ros/client_example.py
