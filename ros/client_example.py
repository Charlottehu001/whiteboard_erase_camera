"""How another module asks the camera for the ink on the board.

Copy the `detect_ink` function into the erase routine. It needs only rclpy and
std_srvs, not this repository.

Usage (camera node running):  python ros/client_example.py
"""
import json

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def detect_ink(node: Node, client, timeout_s: float = 5.0) -> dict:
    """Call the camera and return its answer.

    Move the arm to the photo pose and let it come to rest before calling.
    Returns e.g. {"ink_present": True, "regions": [[x_mm, y_mm, r_mm], ...],
    "unobserved_fraction": 0.0, ...}. Raises RuntimeError if the camera could
    not inspect the board (no image, markers hidden).
    """
    if not client.wait_for_service(timeout_sec=timeout_s):
        raise RuntimeError("camera service not available: is ros/camera.py running?")
    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
    if future.result() is None:
        raise RuntimeError("camera service did not answer in time")
    answer = json.loads(future.result().message)
    if not future.result().success:
        raise RuntimeError(f"camera could not inspect the board: {answer['error']}")
    return answer


def main():
    rclpy.init()
    node = Node("camera_client_example")
    client = node.create_client(Trigger, "/whiteboard_camera/detect")
    try:
        answer = detect_ink(node, client)
        print("ink present:", answer["ink_present"])
        for x, y, r in answer["regions"]:
            # The eraser must cover a circle of radius r around (x, y).
            print(f"  erase around x={x:.1f} mm  y={y:.1f} mm  ink radius={r:.1f} mm")
        if answer["unobserved_fraction"] > 0.05:
            print(f"  note: {100 * answer['unobserved_fraction']:.0f} % of the board could not be checked")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
