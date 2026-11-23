"""Estimate where the markers are on the board from camera images, without a ruler.

Known: every marker is a square of markers.size_mm, and the camera intrinsics
(the .json saved next to the image by ros/capture.py). Unknown: the position and
rotation of each marker on the board and the pose of the camera. Both are fitted
together so that the predicted marker corners match the detected ones.

The result is expressed in the same frame as the config: origin at the top-left
corner of the black square of the first marker id, x towards the second id.

Accuracy is limited by how precisely a small marker fixes the scale; expect a
few millimetres over half a metre. Several images from different camera
positions are averaged. A ruler is more accurate; use this as a cross-check or
when no ruler is at hand.

Usage:  python scripts/estimate_layout.py data/real/pose_v3.png [more images ...]
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

from whiteboard_camera import camera_info_for_image, load_config, undistort
from whiteboard_camera.rectify import detect_markers

ROOT = Path(__file__).resolve().parents[1]


def square(size):
    """Corners of a marker in its own frame, ArUco order, origin at its top-left corner."""
    return np.array([[0.0, 0.0], [size, 0.0], [size, size], [0.0, size]])


def board_points(layout, ids, size):
    """layout: flat (x, y, theta) for every id except the first, which is at the origin."""
    pts = [square(size)]
    for k in range(len(ids) - 1):
        x, y, theta = layout[3 * k : 3 * k + 3]
        c, s = np.cos(theta), np.sin(theta)
        pts.append(square(size) @ np.array([[c, s], [-s, c]]) + [x, y])
    return np.vstack(pts)


def project(points_2d, rvec, tvec, K):
    R = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))[0]
    cam = np.hstack([points_2d, np.zeros((len(points_2d), 1))]) @ R.T + tvec
    pix = cam @ K.T
    return pix[:, :2] / pix[:, 2:3]


def fit(detections, ids, size, K):
    """Joint least-squares fit of marker layout and camera pose for one image."""
    image_pts = np.vstack([detections[i] for i in ids])
    # Start: assume a fronto-parallel view and read positions off the pixel scale.
    scale = np.mean([np.linalg.norm(detections[i][1] - detections[i][0]) for i in ids]) / size
    origin = detections[ids[0]][0]
    top = detections[ids[0]][1] - origin
    angle0 = np.arctan2(top[1], top[0])
    rot = np.array([[np.cos(angle0), np.sin(angle0)], [-np.sin(angle0), np.cos(angle0)]])
    layout = []
    for i in ids[1:]:
        p = rot @ (detections[i][0] - origin) / scale
        edge = rot @ (detections[i][1] - detections[i][0])
        layout += [p[0], p[1], np.arctan2(edge[1], edge[0])]
    layout = np.array(layout)
    obj = np.hstack([board_points(layout, ids, size), np.zeros((4 * len(ids), 1))])
    ok, rvec, tvec = cv2.solvePnP(obj, image_pts, K, None)
    if not ok:
        raise RuntimeError("could not initialise the camera pose")
    params = np.concatenate([layout, np.ravel(rvec), np.ravel(tvec)])
    n_layout = len(layout)

    def residual(p):
        pts = board_points(p[:n_layout], ids, size)
        return (project(pts, p[n_layout : n_layout + 3], p[n_layout + 3 :], K) - image_pts).ravel()

    damping = 1e-3
    cost = np.sum(residual(params) ** 2)
    for _ in range(200):
        r = residual(params)
        J = np.empty((len(r), len(params)))
        for k in range(len(params)):
            step = np.zeros(len(params))
            step[k] = 1e-5 * max(1.0, abs(params[k]))
            J[:, k] = (residual(params + step) - r) / step[k]
        A = J.T @ J
        delta = np.linalg.solve(A + damping * np.diag(np.diag(A)), -J.T @ r)
        new_cost = np.sum(residual(params + delta) ** 2)
        if new_cost < cost:
            params, cost, damping = params + delta, new_cost, damping / 3
            if np.abs(delta).max() < 1e-7:
                break
        else:
            damping *= 5
    rms_px = np.sqrt(cost / (len(image_pts)))
    corners = board_points(params[:n_layout], ids, size).reshape(len(ids), 4, 2)
    # Express in the config's frame: x axis from the first marker's top-left corner to the second's.
    direction = corners[1][0] - corners[0][0]
    a = np.arctan2(direction[1], direction[0])
    R = np.array([[np.cos(a), np.sin(a)], [-np.sin(a), np.cos(a)]])
    corners = corners @ R.T
    distance = float(np.linalg.norm(params[n_layout + 3 :]))
    return corners, rms_px, distance


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("images", nargs="+")
    parser.add_argument("--config", default=ROOT / "config" / "board.yaml")
    args = parser.parse_args()
    cfg = load_config(args.config)
    ids = sorted(cfg.marker_centers_mm)
    size = cfg.marker_size_mm

    results = []
    for path in args.images:
        image = cv2.imread(path, cv2.IMREAD_COLOR)
        if image is None:
            sys.exit(f"cannot read image: {path}")
        K, dist = camera_info_for_image(path)
        if K is None:
            sys.exit(f"no lens data next to {path} (expected a .json written by ros/capture.py)")
        image = undistort(image, K, dist)
        detections = detect_markers(image, cfg)
        if sorted(detections) != ids:
            print(f"{path}: skipped, markers visible {sorted(detections)}, need {ids}")
            continue
        corners, rms_px, distance = fit(detections, ids, size, K)
        results.append(corners)
        print(f"{path}: fit error {rms_px:.2f} px, camera {distance:.0f} mm from marker {ids[0]}")
    if not results:
        sys.exit("no usable image")

    stack = np.array(results)
    mean = stack.mean(axis=0)
    print(f"\nlayout from {len(results)} image(s), frame: origin = top-left corner of marker {ids[0]}, x towards marker {ids[1]}")
    for k, marker_id in enumerate(ids):
        centre = mean[k].mean(axis=0)
        edge = mean[k][1] - mean[k][0]
        spread = "" if len(results) == 1 else f"   spread between images {np.abs(stack[:, k] - mean[k]).max():.1f} mm"
        print(
            f"  marker {marker_id}: top-left corner ({mean[k][0][0]:7.1f}, {mean[k][0][1]:7.1f})  "
            f"centre ({centre[0]:7.1f}, {centre[1]:7.1f})  rotated {np.degrees(np.arctan2(edge[1], edge[0])):+.1f} deg{spread}"
        )
    print("\ndistances between top-left corners (compare with a ruler):")
    for a in range(len(ids)):
        for b in range(a + 1, len(ids)):
            print(f"  marker {ids[a]} -> {ids[b]}: {np.linalg.norm(mean[a][0] - mean[b][0]):6.1f} mm")


if __name__ == "__main__":
    main()
