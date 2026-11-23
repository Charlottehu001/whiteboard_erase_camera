"""ROS 2 node of the camera module.

Subscribes to the RealSense colour image and answers "is there ink on the
whiteboard, and where".

Interface (node name: whiteboard_camera)
  service  ~/detect        std_srvs/Trigger
      Inspect the most recent image. response.success is False when the board
      could not be inspected (no fresh image, markers not visible).
      response.message is JSON:
        {"ink_present": bool,
         "regions": [[x_mm, y_mm, r_mm], ...],   # whiteboard frame
         "ink_area_mm2": float, "unobserved_fraction": float, ...}
  topic    ~/ink_present   std_msgs/Bool               published after each detection
  topic    ~/ink_regions   std_msgs/Float32MultiArray  N x 3 array of [x_mm, y_mm, r_mm]
  topic    ~/debug_image   sensor_msgs/Image           rectified view with the result drawn in
  topic    ~/camera_view   sensor_msgs/Image           camera image with markers and drawing area drawn in
  service  ~/save_reference  std_srvs/Trigger
      Store the current view of the CLEAN board as the reference image and use
      it from now on. Call it at the photo pose with a clean board.

Parameters
  config_path        path to board.yaml
  reference_path     rectified photo of the clean board, default data/reference.png.
                     Used if the file exists; ~/save_reference creates it.
                     "none" = never use a reference
  image_topic        default /camera/camera/color/image_raw
  camera_info_topic  default /camera/camera/color/camera_info
  max_image_age_s    refuse to use an image older than this (default 1.0)
  detect_rate_hz     > 0: also detect continuously at this rate (default 0 = on request only)

Run (ROS 2 sourced and the project's virtual environment active):
  python ros/camera.py
"""
import json
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32MultiArray, MultiArrayDimension
from std_srvs.srv import Trigger

from whiteboard_camera import RectificationError, WhiteboardInspector, draw_camera_view, draw_debug, undistort

ROOT = Path(__file__).resolve().parents[1]

# sensor_msgs/Image encoding -> (channels, indices that give B, G, R)
_ENCODINGS = {
    "bgr8": (3, [0, 1, 2]),
    "rgb8": (3, [2, 1, 0]),
    "bgra8": (4, [0, 1, 2]),
    "rgba8": (4, [2, 1, 0]),
    "mono8": (1, [0, 0, 0]),
}


def image_msg_to_bgr(msg: Image) -> np.ndarray:
    """Convert a sensor_msgs/Image to an OpenCV BGR array (no cv_bridge needed)."""
    if msg.encoding not in _ENCODINGS:
        raise ValueError(f"unsupported image encoding: {msg.encoding}")
    channels, order = _ENCODINGS[msg.encoding]
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    # A row can be padded: only the first width*channels bytes are pixels.
    pixels = rows[:, : msg.width * channels].reshape(msg.height, msg.width, channels)
    return np.ascontiguousarray(pixels[:, :, order])


def bgr_to_image_msg(image: np.ndarray, header) -> Image:
    msg = Image()
    msg.header = header
    msg.height, msg.width = image.shape[:2]
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = image.shape[1] * 3
    msg.data = np.ascontiguousarray(image).tobytes()
    return msg


class WhiteboardCameraNode(Node):
    def __init__(self):
        super().__init__("whiteboard_camera")
        self.declare_parameter("config_path", str(ROOT / "config" / "board.yaml"))
        self.declare_parameter("reference_path", str(ROOT / "data" / "reference.png"))
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("max_image_age_s", 1.0)
        self.declare_parameter("detect_rate_hz", 0.0)

        config_path = self.get_parameter("config_path").value
        reference_path = self.get_parameter("reference_path").value
        if reference_path.strip().lower() == "none":
            reference_path = ""
        self.reference_path = Path(reference_path) if reference_path else ROOT / "data" / "reference.png"
        use_reference = bool(reference_path) and self.reference_path.exists()
        self.inspector = WhiteboardInspector.from_files(
            config_path, self.reference_path if use_reference else None
        )
        self.max_age = float(self.get_parameter("max_image_age_s").value)

        self.latest_image = None       # sensor_msgs/Image
        self.latest_received = None    # node time when it arrived
        self.camera_matrix = None
        self.dist_coeffs = None

        image_topic = self.get_parameter("image_topic").value
        info_topic = self.get_parameter("camera_info_topic").value
        self.create_subscription(Image, image_topic, self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, info_topic, self.on_camera_info, qos_profile_sensor_data)

        self.pub_present = self.create_publisher(Bool, "~/ink_present", 10)
        self.pub_regions = self.create_publisher(Float32MultiArray, "~/ink_regions", 10)
        self.pub_debug = self.create_publisher(Image, "~/debug_image", 1)
        self.pub_view = self.create_publisher(Image, "~/camera_view", 1)
        self.create_service(Trigger, "~/detect", self.on_detect)
        self.create_service(Trigger, "~/save_reference", self.on_save_reference)

        rate = float(self.get_parameter("detect_rate_hz").value)
        if rate > 0.0:
            self.create_timer(1.0 / rate, self.on_timer)

        self.get_logger().info(f"config: {config_path} | image topic: {image_topic}")
        if use_reference:
            self.get_logger().info(
                f"using reference image {self.reference_path}: only valid for the pose and lighting it was taken with"
            )
        else:
            self.get_logger().warn(
                "no reference image: broad faint residue will not be detected. "
                "With a clean board at the photo pose, call ~/save_reference"
            )

    def on_image(self, msg: Image):
        self.latest_image = msg
        self.latest_received = self.get_clock().now()

    def on_camera_info(self, msg: CameraInfo):
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)

    def fresh_image(self):
        """Latest camera image as BGR. Returns (image, "") or (None, error text)."""
        if self.latest_image is None:
            return None, "no image received yet"
        age = (self.get_clock().now() - self.latest_received).nanoseconds * 1e-9
        if age > self.max_age:
            return None, f"latest image is {age:.1f} s old (limit {self.max_age:.1f} s)"
        try:
            return image_msg_to_bgr(self.latest_image), ""
        except ValueError as error:
            return None, str(error)

    def publish_camera_view(self, image):
        if self.pub_view.get_subscription_count() == 0:
            return
        view = draw_camera_view(undistort(image, self.camera_matrix, self.dist_coeffs), self.inspector)
        self.pub_view.publish(bgr_to_image_msg(view, self.latest_image.header))

    def detect(self):
        """Inspect the latest image. Returns (ok, result or error text)."""
        image, error = self.fresh_image()
        if image is None:
            return False, error
        try:
            result = self.inspector.inspect(image, self.camera_matrix, self.dist_coeffs)
        except (RectificationError, ValueError) as error:
            self.publish_camera_view(image)
            return False, str(error)
        self.publish_camera_view(image)

        if result.homography_source == "cached":
            self.get_logger().warn(
                "markers not visible, reusing the last homography: only valid at the same camera pose"
            )
        if result.unobserved_fraction > 0.05:
            self.get_logger().warn(
                f"{100 * result.unobserved_fraction:.0f} % of the drawing area is hidden or in glare"
            )

        self.pub_present.publish(Bool(data=result.ink_present))
        regions = Float32MultiArray()
        regions.layout.dim = [
            MultiArrayDimension(label="regions", size=len(result.regions), stride=3 * len(result.regions)),
            MultiArrayDimension(label="x_mm,y_mm,r_mm", size=3, stride=3),
        ]
        regions.data = [float(v) for circle in result.regions for v in circle]
        self.pub_regions.publish(regions)
        if self.pub_debug.get_subscription_count() > 0:
            debug = draw_debug(result, self.inspector.cfg)
            self.pub_debug.publish(bgr_to_image_msg(debug, self.latest_image.header))
        return True, result

    def on_detect(self, request, response):
        ok, outcome = self.detect()
        response.success = ok
        if ok:
            response.message = json.dumps(outcome.to_dict())
        else:
            response.message = json.dumps({"error": outcome})
            self.get_logger().error(f"detect failed: {outcome}")
        return response

    def on_save_reference(self, request, response):
        image, error = self.fresh_image()
        if image is None:
            response.success, response.message = False, error
            return response
        # Judge the photo without any reference: is the board really clean?
        previous, self.inspector.reference = self.inspector.reference, None
        try:
            result = self.inspector.inspect(image, self.camera_matrix, self.dist_coeffs)
        except (RectificationError, ValueError) as error:
            response.success, response.message = False, str(error)
            return response
        finally:
            self.inspector.reference = previous
        if result.homography_source != "markers":
            response.success = False
            response.message = "markers not visible: the reference must be taken with the board located"
            return response
        self.reference_path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(self.reference_path), result.rectified)
        self.inspector.reference = result.rectified
        response.success = True
        response.message = f"saved {self.reference_path} and using it now"
        if result.ink_present:
            response.message += (
                f". WARNING: {result.ink_area_mm2:.0f} mm2 of ink seen in this image; "
                "a reference with ink on it hides that ink later"
            )
            self.get_logger().warn(response.message)
        else:
            self.get_logger().info(response.message)
        return response

    def on_timer(self):
        ok, outcome = self.detect()
        if not ok:
            self.get_logger().warn(f"detect failed: {outcome}", throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = WhiteboardCameraNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
