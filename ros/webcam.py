"""Publish a webcam on the camera topics, as a stand-in for the RealSense.

For rehearsing the whole workflow with real images (printed markers, real pen
ink) before the lab camera is available.

Usage:
  python ros/webcam.py                              # /dev/video0
  python ros/webcam.py --ros-args -p device:=2 -p width:=1280 -p height:=720
"""
import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class Webcam(Node):
    def __init__(self):
        super().__init__("webcam")
        self.declare_parameter("device", 0)
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 720)
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")

        device = int(self.get_parameter("device").value)
        self.capture = cv2.VideoCapture(device)
        if not self.capture.isOpened():
            raise RuntimeError(f"cannot open webcam /dev/video{device}")
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.get_parameter("width").value))
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.get_parameter("height").value))
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # always the newest frame

        self.pub_image = self.create_publisher(Image, self.get_parameter("image_topic").value, 10)
        self.pub_info = self.create_publisher(CameraInfo, self.get_parameter("camera_info_topic").value, 10)
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self.publish)
        self.get_logger().info(f"publishing /dev/video{device}")

    def publish(self):
        ok, bgr = self.capture.read()
        if not ok:
            self.get_logger().warn("no frame from the webcam", throttle_duration_sec=5.0)
            return
        image = Image()
        image.header.stamp = self.get_clock().now().to_msg()
        image.header.frame_id = "camera_color_optical_frame"
        image.height, image.width = bgr.shape[:2]
        image.encoding = "rgb8"
        image.step = image.width * 3
        image.data = np.ascontiguousarray(bgr[:, :, ::-1]).tobytes()
        self.pub_image.publish(image)

        # The webcam is not calibrated: a plausible camera matrix and zero
        # distortion, so the node applies no lens correction.
        info = CameraInfo()
        info.header = image.header
        info.height, info.width = image.height, image.width
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        focal = float(image.width)
        info.k = [focal, 0.0, image.width / 2.0, 0.0, focal, image.height / 2.0, 0.0, 0.0, 1.0]
        self.pub_info.publish(info)


def main():
    rclpy.init()
    node = Webcam()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.capture.release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
