"""Save one image from the camera topic to a file.

Writes data/real/NAME.png (the raw camera image) and data/real/NAME.json (the
lens data of the camera), which scripts/inspect_image.py picks up by itself.
Use it to collect real photos for tuning and as test data.

Usage (ROS 2 sourced, virtual environment active, camera driver running):
  python ros/capture.py clean_board
  python ros/capture.py red_circle --image-topic /camera/color/image_raw
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from camera import image_msg_to_bgr  # ros/camera.py, same folder
from whiteboard_camera import save_camera_info

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("name", help="file name without extension, e.g. clean_board")
    parser.add_argument("--image-topic", default="/camera/camera/color/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/camera/color/camera_info")
    parser.add_argument("--out", default=ROOT / "data" / "real")
    parser.add_argument("--timeout", type=float, default=5.0, help="seconds to wait for an image")
    args = parser.parse_args()

    rclpy.init()
    node = Node("whiteboard_capture")
    received = {}
    node.create_subscription(Image, args.image_topic, lambda m: received.__setitem__("image", m), qos_profile_sensor_data)
    node.create_subscription(
        CameraInfo, args.camera_info_topic, lambda m: received.setdefault("info", m), qos_profile_sensor_data
    )
    # The first frames after subscribing can still be adjusting exposure:
    # wait for the image and the lens data, then take the newest image.
    deadline = node.get_clock().now().nanoseconds + int(args.timeout * 1e9)
    while rclpy.ok() and node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if "image" in received and "info" in received:
            break
    for _ in range(5):
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()

    if "image" not in received:
        sys.exit(f"no image on {args.image_topic} within {args.timeout:g} s. Check `ros2 topic list`.")
    image = image_msg_to_bgr(received["image"])
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    image_path = out / f"{args.name}.png"
    cv2.imwrite(str(image_path), image)
    print(f"wrote {image_path}  ({image.shape[1]} x {image.shape[0]})")
    if "info" in received:
        info = received["info"]
        save_camera_info(image_path.with_suffix(".json"), np.array(info.k).reshape(3, 3), info.d, info.width, info.height)
        print(f"wrote {image_path.with_suffix('.json')}  (lens data)")
    else:
        print(f"warning: no lens data on {args.camera_info_topic}; lens distortion will not be corrected")


if __name__ == "__main__":
    main()
