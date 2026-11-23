"""Click the outline of the gripper in a camera image to get an ignore polygon.

Left click adds a corner, right click removes the last one, Enter finishes.
The script prints the line to paste into config/board.yaml under
`image_ignore_polygons:`. Run it once per object that hides the board.

Usage:  python scripts/pick_polygon.py data/real/clean_board.png
"""
import argparse
import sys

import cv2
import matplotlib.pyplot as plt

from whiteboard_camera import camera_info_for_image, undistort


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("image", help="raw camera image, e.g. saved with ros/capture.py")
    args = parser.parse_args()

    image = cv2.imread(args.image, cv2.IMREAD_COLOR)
    if image is None:
        sys.exit(f"cannot read image: {args.image}")
    # The polygons are applied to the undistorted image, so click on that one.
    image = undistort(image, *camera_info_for_image(args.image))

    plt.figure(figsize=(12, 7))
    plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    plt.title("left click: add corner   right click: undo   Enter: finish")
    points = plt.ginput(n=-1, timeout=0)
    plt.close()
    if len(points) < 3:
        sys.exit("a polygon needs at least 3 corners")
    polygon = [[int(round(x)), int(round(y))] for x, y in points]
    print("Paste into config/board.yaml (one list entry per polygon):")
    print("image_ignore_polygons:")
    print(f"  - {polygon}")


if __name__ == "__main__":
    main()
