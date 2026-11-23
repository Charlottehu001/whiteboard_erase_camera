"""The complete camera pipeline: camera image -> "is there ink, and where".

    image --(markers, homography)--> rectified top-down view
          --(compare with clean board)--> ink mask
          --(grid + enclosing circles)--> ink_present, [(x, y, r), ...] in mm
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

from .config import BoardConfig, load_config
from .rectify import apply_homography, detect_markers, estimate_homography, observed_mask, rectify
from .regions import Circle, mask_to_circles
from .segment import segment_ink


class RectificationError(RuntimeError):
    """The whiteboard could not be located in the image."""


@dataclass
class InspectionResult:
    ink_present: bool
    regions: list[Circle]            # (x_mm, y_mm, r_mm) in the whiteboard frame
    ink_area_mm2: float
    unobserved_fraction: float       # share of the drawing area that could not be checked
    markers_found: list[int]
    homography_source: str           # "markers" or "cached"
    reprojection_rms_mm: float       # marker fit error, nan when cached
    # Images for debugging, not part of the interface to other modules.
    rectified: np.ndarray = field(repr=False, default=None)
    ink_mask: np.ndarray = field(repr=False, default=None)
    unobserved_mask: np.ndarray = field(repr=False, default=None)

    def to_dict(self) -> dict:
        rms = self.reprojection_rms_mm
        return {
            "ink_present": self.ink_present,
            "regions": [list(c) for c in self.regions],
            "ink_area_mm2": round(self.ink_area_mm2, 1),
            "unobserved_fraction": round(self.unobserved_fraction, 4),
            "markers_found": self.markers_found,
            "homography_source": self.homography_source,
            "reprojection_rms_mm": None if np.isnan(rms) else round(rms, 3),
        }


class WhiteboardInspector:
    """Holds the configuration, the clean-board reference and the last homography."""

    def __init__(self, cfg: BoardConfig, reference_bgr: np.ndarray | None = None):
        self.cfg = cfg
        self.reference = reference_bgr
        # Last homography found from markers. Reused when markers are hidden,
        # which is only correct if the camera is at the same pose as before.
        self.homography: np.ndarray | None = None

    @classmethod
    def from_files(cls, config_path: str | Path, reference_path: str | Path | None = None):
        cfg = load_config(config_path)
        reference = None
        if reference_path:
            reference = cv2.imread(str(reference_path), cv2.IMREAD_COLOR)
            if reference is None:
                raise FileNotFoundError(f"cannot read reference image: {reference_path}")
        return cls(cfg, reference)

    def locate_board(self, image_bgr: np.ndarray):
        """Return (H image->board, source, rms_mm, marker ids)."""
        detections = detect_markers(image_bgr, self.cfg)
        ids = sorted(detections)
        H, rms = estimate_homography(detections, self.cfg)
        if H is not None:
            self.homography = H
            return H, "markers", rms, ids
        if self.homography is not None:
            return self.homography, "cached", float("nan"), ids
        raise RectificationError(
            f"only {len(ids)} of the configured markers visible (ids {ids}), "
            f"need {self.cfg.min_markers}, and no earlier homography to fall back on"
        )

    def rectify(self, image_bgr, camera_matrix=None, dist_coeffs=None):
        """Undistort (if lens data is given) and warp to the top-down view."""
        image_bgr = undistort(image_bgr, camera_matrix, dist_coeffs)
        H, source, rms, ids = self.locate_board(image_bgr)
        return rectify(image_bgr, H, self.cfg), image_bgr, H, source, rms, ids

    def inspect(self, image_bgr, camera_matrix=None, dist_coeffs=None) -> InspectionResult:
        if image_bgr is None or image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
            raise ValueError("inspect() needs a BGR colour image")
        rect, undistorted, H, source, rms, ids = self.rectify(image_bgr, camera_matrix, dist_coeffs)
        observed = observed_mask(undistorted.shape, H, self.cfg)
        mask, unobserved = segment_ink(rect, self.cfg, self.reference, observed)
        circles = mask_to_circles(mask, self.cfg)
        return InspectionResult(
            ink_present=bool(mask.any()),
            regions=circles,
            ink_area_mm2=float(mask.sum()) / self.cfg.px_per_mm**2,
            unobserved_fraction=float(unobserved.mean()),
            markers_found=ids,
            homography_source=source,
            reprojection_rms_mm=rms,
            rectified=rect,
            ink_mask=mask,
            unobserved_mask=unobserved,
        )


def undistort(image, camera_matrix, dist_coeffs):
    """Remove lens distortion. Returns the image unchanged without lens data or with all-zero coefficients."""
    if camera_matrix is None or dist_coeffs is None:
        return image
    dist = np.asarray(dist_coeffs, dtype=np.float64).ravel()
    if dist.size == 0 or not np.any(dist):
        return image
    return cv2.undistort(image, np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3), dist)


def draw_debug(result: InspectionResult, cfg: BoardConfig) -> np.ndarray:
    """Rectified image with ink (red), unobserved areas (yellow) and circles (green)."""
    scale = 2
    view = result.rectified.copy()
    view[result.unobserved_mask] = (0.5 * view[result.unobserved_mask] + (0, 110, 127)).astype(np.uint8)
    view[result.ink_mask] = (0, 0, 255)
    view = cv2.resize(view, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    for x, y, r in result.regions:
        u, v = apply_homography(cfg.board_to_canvas, [[x, y]])[0]
        center = (int(round((u + 0.5) * scale)), int(round((v + 0.5) * scale)))
        cv2.circle(view, center, int(round(r * cfg.px_per_mm * scale)), (0, 160, 0), 2, cv2.LINE_AA)
    label = (
        f"ink_present={result.ink_present}  regions={len(result.regions)}  "
        f"area={result.ink_area_mm2:.0f}mm2  unobserved={100 * result.unobserved_fraction:.1f}%  "
        f"H={result.homography_source}"
    )
    bar = np.full((28, view.shape[1], 3), 30, np.uint8)
    cv2.putText(bar, label, (6, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
    return np.vstack([bar, view])


def draw_camera_view(image_bgr: np.ndarray, inspector: WhiteboardInspector) -> np.ndarray:
    """Camera image with what the module sees in it, for setting up the photo pose.

    Green: detected markers with their ids. Cyan: outline of the drawing area,
    from the current markers or, if too few are visible, from the last good
    homography. Magenta: the ``image_ignore_polygons``. Works without a located
    board too, which is exactly when it is needed.
    """
    cfg = inspector.cfg
    view = image_bgr.copy()
    detections = detect_markers(image_bgr, cfg)
    for marker_id, corners in detections.items():
        pts = np.round(corners).astype(np.int32)
        cv2.polylines(view, [pts], True, (0, 200, 0), 2, cv2.LINE_AA)
        cv2.putText(view, str(marker_id), tuple(int(v) for v in pts[0]), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (0, 200, 0), 2, cv2.LINE_AA)
    for polygon in cfg.image_ignore_polygons:
        cv2.polylines(view, [np.round(np.asarray(polygon)).astype(np.int32)], True, (255, 0, 255), 2)

    H, rms = estimate_homography(detections, cfg)
    source = "markers"
    if H is None and inspector.homography is not None:
        H, source = inspector.homography, "cached"
    if H is not None:
        area = [[cfg.x_min, cfg.y_min], [cfg.x_max, cfg.y_min], [cfg.x_max, cfg.y_max], [cfg.x_min, cfg.y_max]]
        outline = apply_homography(np.linalg.inv(H), area)
        cv2.polylines(view, [np.round(outline).astype(np.int32)], True, (255, 200, 0), 2, cv2.LINE_AA)
        origin = tuple(int(round(v)) for v in outline[0])
        cv2.circle(view, origin, 6, (255, 200, 0), -1)
        cv2.putText(view, "(x_min, y_min)", (origin[0] + 8, origin[1] - 8), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 200, 0), 1, cv2.LINE_AA)

    missing = sorted(set(cfg.marker_centers_mm) - set(detections))
    label = f"markers found {sorted(detections)}  missing {missing}"
    if H is None:
        label += f"  BOARD NOT LOCATED (need {cfg.min_markers})"
    elif source == "markers":
        label += f"  fit error {rms:.2f} mm"
    else:
        label += "  using CACHED homography"
    bar = np.full((28, view.shape[1], 3), 30, np.uint8)
    cv2.putText(bar, label, (6, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return np.vstack([bar, view])


# --- lens data next to saved images ----------------------------------------
# ros/capture.py stores the camera matrix and distortion coefficients of a
# saved photo in a JSON file of the same name, so the command line scripts
# treat the photo exactly as the ROS node treats the live image.


def save_camera_info(path: str | Path, camera_matrix, dist_coeffs, width: int, height: int) -> None:
    data = {
        "width": int(width),
        "height": int(height),
        "camera_matrix": np.asarray(camera_matrix, dtype=float).reshape(3, 3).tolist(),
        "dist_coeffs": np.asarray(dist_coeffs, dtype=float).ravel().tolist(),
    }
    Path(path).write_text(json.dumps(data, indent=2), encoding="utf-8")


def load_camera_info(path: str | Path):
    """Return (camera_matrix, dist_coeffs) from a file written by save_camera_info."""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    return np.array(data["camera_matrix"], dtype=np.float64), np.array(data["dist_coeffs"], dtype=np.float64)


def camera_info_for_image(image_path: str | Path):
    """Lens data stored next to ``image_path`` (same name, .json), or (None, None)."""
    sidecar = Path(image_path).with_suffix(".json")
    if sidecar.exists():
        return load_camera_info(sidecar)
    return None, None
