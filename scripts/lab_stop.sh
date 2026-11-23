#!/usr/bin/env bash
# Stop the programs started by scripts/lab_start.sh.
pkill -INT -f "[r]s_launch.py" 2>/dev/null
pkill -INT -f "[r]ealsense2_camera_node" 2>/dev/null
pkill -INT -f "[r]os/camera.py" 2>/dev/null
pkill -f "[r]qt_image_view" 2>/dev/null
sleep 2
pkill -KILL -f "[r]ealsense2_camera_node" 2>/dev/null
[ "$1" = quiet ] || echo "stopped"
