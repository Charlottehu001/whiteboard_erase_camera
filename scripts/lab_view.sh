#!/usr/bin/env bash
# Open the live result window: top-down view of the drawing area with detected ink.
# Usage: scripts/lab_view.sh [camera]     "camera" shows the camera image with the markers instead
source /opt/ros/jazzy/setup.bash
topic=/whiteboard_camera/debug_image
[ "$1" = camera ] && topic=/whiteboard_camera/camera_view
setsid nohup ros2 run rqt_image_view rqt_image_view "$topic" > /dev/null 2>&1 &
echo "opened a window on $topic"
