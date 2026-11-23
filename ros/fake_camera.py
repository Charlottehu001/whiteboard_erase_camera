"""Fake RealSense: publishes synthetic whiteboard images on the camera topics.

Lets the camera node (and the modules that call it) be tested without the
camera or the robot.

Usage:
  python ros/fake_camera.py                               # board with the demo drawing
  python ros/fake_camera.py --ros-args -p scene:=blank    # blank | drawing | residue | drawing_glare
"""
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from whiteboard_camera import load_config
from whiteboard_camera.synthetic import Smear, demo_strokes, look_at_camera, render_scene

ROOT = Path(__file__).resolve().parents[1]
GLARE = {"center": (700, 330), "sigma": (500.0, 35.0), "angle_deg": 70.0, "amplitude": 170.0}


class FakeCamera(Node):
    def __init__(self):
        super().__init__("fake_camera")
        self.declare_parameter("config_path", str(ROOT / "config" / "board.yaml"))
        self.declare_parameter("scene", "drawing")
        self.declare_parameter("rate_hz", 5.0)
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")

        self.cfg = load_config(self.get_parameter("config_path").value)
        self.camera = look_at_camera(self.cfg, tilt_x_deg=12.0, tilt_y_deg=-18.0)
        scene = self.get_parameter("scene").value
        centre = ((self.cfg.x_min + self.cfg.x_max) / 2, (self.cfg.y_min + self.cfg.y_max) / 2)
        scenes = {
            "blank": dict(),
            "drawing": dict(strokes=demo_strokes(self.cfg)),
            "drawing_glare": dict(strokes=demo_strokes(self.cfg), glare=GLARE),
            "residue": dict(smears=[Smear(centre, 45.0, "red", 0.3)]),
        }
        if scene not in scenes:
            raise ValueError(f"unknown scene '{scene}', choose from {sorted(scenes)}")
        self.scene = scenes[scene]
        self.frame = 0

        self.pub_image = self.create_publisher(Image, self.get_parameter("image_topic").value, 10)
        self.pub_info = self.create_publisher(CameraInfo, self.get_parameter("camera_info_topic").value, 10)
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self.publish)
        self.get_logger().info(f"publishing scene '{scene}'")

    def publish(self):
        # A new noise seed per frame, like a real sensor.
        bgr = render_scene(self.cfg, self.camera, seed=self.frame, **self.scene)
        self.frame += 1
        if not rclpy.ok():  # shut down while rendering
            return
        stamp = self.get_clock().now().to_msg()

        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = "camera_color_optical_frame"
        image.height, image.width = bgr.shape[:2]
        image.encoding = "rgb8"  # the RealSense driver publishes rgb8
        image.step = image.width * 3
        image.data = np.ascontiguousarray(bgr[:, :, ::-1]).tobytes()
        self.pub_image.publish(image)

        info = CameraInfo()
        info.header = image.header
        info.height, info.width = image.height, image.width
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = self.camera.K.ravel().tolist()
        self.pub_info.publish(info)


def main():
    rclpy.init()
    node = FakeCamera()
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
