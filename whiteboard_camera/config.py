"""Configuration of the whiteboard camera module.

Everything that depends on the physical setup (board size, marker positions,
thresholds) lives in ``config/board.yaml`` and is loaded into ``BoardConfig``.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml


@dataclass(frozen=True)
class BoardConfig:
    # Drawing area in the whiteboard frame, millimetres.
    x_min: float = 0.0
    x_max: float = 300.0
    y_min: float = 0.0
    y_max: float = 200.0
    y_axis_up: bool = False
    px_per_mm: float = 2.0

    marker_dictionary: str = "DICT_4X4_50"
    marker_size_mm: float = 40.0
    marker_centers_mm: dict = field(
        default_factory=lambda: {
            0: (-30.0, -30.0),
            1: (330.0, -30.0),
            2: (330.0, 230.0),
            3: (-30.0, 230.0),
        }
    )
    min_markers: int = 3

    min_darkness: float = 0.12
    background_kernel_mm: float = 25.0
    min_component_area_mm2: float = 4.0
    saturation_level: int = 250

    cell_mm: float = 40.0

    image_ignore_polygons: tuple = ()

    def __post_init__(self):
        if self.x_max <= self.x_min or self.y_max <= self.y_min:
            raise ValueError("board: x_max/y_max must be larger than x_min/y_min")
        if self.px_per_mm <= 0 or self.marker_size_mm <= 0 or self.cell_mm <= 0:
            raise ValueError("px_per_mm, markers.size_mm and regions.cell_mm must be positive")
        if not 0.0 < self.min_darkness < 1.0:
            raise ValueError("segmentation.min_darkness must be between 0 and 1")
        if len(self.marker_centers_mm) < 2:
            raise ValueError("at least 2 markers must be configured")

    # --- rectified image ("canvas") geometry -------------------------------

    @property
    def canvas_size(self) -> tuple[int, int]:
        """(width, height) of the rectified image in pixels."""
        w = int(round((self.x_max - self.x_min) * self.px_per_mm))
        h = int(round((self.y_max - self.y_min) * self.px_per_mm))
        return w, h

    @property
    def board_to_canvas(self) -> np.ndarray:
        """3x3 matrix mapping board millimetres to rectified pixel coordinates.

        Pixel (u, v) of the rectified image is centred on the board point
        ``x = x_min + (u + 0.5) / px_per_mm`` (and the same for y, flipped if the
        board's y axis points up), hence the -0.5 offsets.
        """
        s = self.px_per_mm
        if self.y_axis_up:
            return np.array(
                [[s, 0.0, -self.x_min * s - 0.5], [0.0, -s, self.y_max * s - 0.5], [0.0, 0.0, 1.0]]
            )
        return np.array(
            [[s, 0.0, -self.x_min * s - 0.5], [0.0, s, -self.y_min * s - 0.5], [0.0, 0.0, 1.0]]
        )

    @property
    def canvas_to_board(self) -> np.ndarray:
        return np.linalg.inv(self.board_to_canvas)

    def marker_corners_board(self, marker_id: int) -> np.ndarray:
        """Board coordinates of a marker's corners, in ArUco order.

        ArUco reports corners as top-left, top-right, bottom-right, bottom-left
        of the upright marker. "Top" is +y if the board's y axis points up and
        -y otherwise.
        """
        cx, cy = self.marker_centers_mm[marker_id]
        h = self.marker_size_mm / 2.0
        up = 1.0 if self.y_axis_up else -1.0
        return np.array(
            [
                [cx - h, cy + up * h],
                [cx + h, cy + up * h],
                [cx + h, cy - up * h],
                [cx - h, cy - up * h],
            ]
        )


def load_config(path: str | Path) -> BoardConfig:
    """Read ``board.yaml`` into a BoardConfig."""
    with open(path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f) or {}
    board = raw.get("board", {})
    markers = raw.get("markers", {})
    seg = raw.get("segmentation", {})
    regions = raw.get("regions", {})

    kwargs = {}
    for key in ("x_min", "x_max", "y_min", "y_max", "px_per_mm"):
        if key in board:
            kwargs[key] = float(board[key])
    if "y_axis_up" in board:
        kwargs["y_axis_up"] = bool(board["y_axis_up"])
    if "dictionary" in markers:
        kwargs["marker_dictionary"] = str(markers["dictionary"])
    if "size_mm" in markers:
        kwargs["marker_size_mm"] = float(markers["size_mm"])
    if "centers_mm" in markers:
        kwargs["marker_centers_mm"] = {
            int(k): (float(v[0]), float(v[1])) for k, v in markers["centers_mm"].items()
        }
    if "min_markers" in markers:
        kwargs["min_markers"] = int(markers["min_markers"])
    for key in ("min_darkness", "background_kernel_mm", "min_component_area_mm2"):
        if key in seg:
            kwargs[key] = float(seg[key])
    if "saturation_level" in seg:
        kwargs["saturation_level"] = int(seg["saturation_level"])
    if "cell_mm" in regions:
        kwargs["cell_mm"] = float(regions["cell_mm"])
    polygons = raw.get("image_ignore_polygons") or []
    kwargs["image_ignore_polygons"] = tuple(
        tuple((float(p[0]), float(p[1])) for p in poly) for poly in polygons
    )
    return BoardConfig(**kwargs)
