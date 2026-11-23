#!/usr/bin/env bash
# Bring up everything for a lab session: RealSense driver, detection node, live view.
# Safe to run again at any time: a running camera driver is kept, the detection
# node is restarted (so it picks up config changes), the view window is opened
# if it is not open. Never start the driver by hand as well: two drivers fight
# over the camera and the image rate collapses.
# Usage:  scripts/lab_start.sh            (extra arguments go to the node, e.g. -p reference_path:=none)
# Stop:   scripts/lab_stop.sh
# Logs:   output/driver.log, output/node.log
cd "$(dirname "$0")/.." || exit 1
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
mkdir -p output

if pgrep -f "[r]ealsense-viewer" > /dev/null; then
    echo "realsense-viewer is open: close it first, it blocks the camera."; exit 1
fi
if ! lsusb | grep -qi realsense; then
    echo "No RealSense camera on USB: check the cable."; exit 1
fi

drivers=$(pgrep -c -f "[r]ealsense2_camera_node")
if [ "$drivers" -gt 1 ]; then
    echo "$drivers camera drivers are running, stopping all of them"
    pkill -INT -f "[r]s_launch.py"; pkill -INT -f "[r]ealsense2_camera_node"; sleep 3
    pkill -KILL -f "[r]ealsense2_camera_node"; drivers=0
fi
if [ "$drivers" -eq 0 ]; then
    setsid nohup ros2 launch realsense2_camera rs_launch.py \
        rgb_camera.color_profile:=1280x720x30 enable_depth:=false > output/driver.log 2>&1 &
    echo -n "starting the camera driver "
else
    echo -n "camera driver already running, keeping it "
fi
ok=0
for _ in $(seq 30); do
    if timeout 3 ros2 topic echo --once --no-arr /camera/camera/color/camera_info > /dev/null 2>&1; then ok=1; break; fi
    echo -n "."; sleep 1
done
echo
if [ "$ok" != 1 ]; then echo "no images from the camera, see output/driver.log"; tail -5 output/driver.log; exit 1; fi

pkill -INT -f "[r]os/camera.py"; sleep 1.5
setsid nohup python ros/camera.py --ros-args -p detect_rate_hz:=2.0 "$@" > output/node.log 2>&1 &
sleep 2
if ! pgrep -f "[r]qt_image_view" > /dev/null; then
    setsid nohup ros2 run rqt_image_view rqt_image_view /whiteboard_camera/camera_view > /dev/null 2>&1 &
fi
echo "running: 1 camera driver, detection node (2 detections per second), live view window"
echo "stop with: scripts/lab_stop.sh"
