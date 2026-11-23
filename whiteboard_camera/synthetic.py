"""Synthetic camera images of the whiteboard, with known ground truth.

Used to develop and test the pipeline before real RealSense images exist, and
by the fake camera ROS node. A scene is built in three steps:

1. Paint the board (markers, pen strokes, faint residue) as a flat texture.
2. Project it into a virtual pinhole camera. For a plane this projection is
   the homography H = K [r1 r2 t], the same model the pipeline inverts.
3. Add what makes real images hard: uneven lighting, a glare stripe, noise.
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .config import BoardConfig
from .rectify import _aruco_dictionary, apply_homography

INK_BGR = {
    "red": (45, 45, 200),
    "green": (70, 140, 40),
    "blue": (170, 80, 30),
    "black": (45, 45, 45),
}
BOARD_BGR = (236, 238, 238)
WALL_BGR = (95, 100, 105)
_TEXTURE_PX_PER_MM = 4.0
_TEXTURE_MARGIN_MM = 90.0


@dataclass
class Stroke:
    """A pen stroke: polyline through ``points`` (board mm)."""
    points: list
    color: str = "red"
    width_mm: float = 3.0
    closed: bool = False


@dataclass
class Smear:
    """Broad faint residue left after erasing: a soft-edged disc."""
    center: tuple
    radius_mm: float
    color: str = "red"
    alpha: float = 0.25


def circle_stroke(cx, cy, r, color="red", width_mm=3.0, n=90) -> Stroke:
    angles = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
    pts = [(cx + r * np.cos(a), cy + r * np.sin(a)) for a in angles]
    return Stroke(pts, color, width_mm, closed=True)


def rectangle_stroke(x0, y0, x1, y1, color="blue", width_mm=3.0) -> Stroke:
    return Stroke([(x0, y0), (x1, y0), (x1, y1), (x0, y1)], color, width_mm, closed=True)


def demo_strokes(cfg: BoardConfig) -> list[Stroke]:
    """A circle, a rectangle, a triangle and a line in four colours."""
    w, h = cfg.x_max - cfg.x_min, cfg.y_max - cfg.y_min
    x0, y0 = cfg.x_min, cfg.y_min
    return [
        circle_stroke(x0 + 0.22 * w, y0 + 0.32 * h, 0.14 * h * 1.3, "red"),
        rectangle_stroke(x0 + 0.48 * w, y0 + 0.14 * h, x0 + 0.86 * w, y0 + 0.46 * h, "blue"),
        Stroke(
            [(x0 + 0.12 * w, y0 + 0.88 * h), (x0 + 0.40 * w, y0 + 0.88 * h), (x0 + 0.26 * w, y0 + 0.60 * h)],
            "green",
            closed=True,
        ),
        Stroke([(x0 + 0.55 * w, y0 + 0.66 * h), (x0 + 0.90 * w, y0 + 0.86 * h)], "black"),
    ]


def _visual_flip(cfg: BoardConfig) -> np.ndarray:
    """Board frame -> "visual" frame (x right, y down as seen facing the board)."""
    return np.diag([1.0, -1.0 if cfg.y_axis_up else 1.0, 1.0])


@dataclass
class Camera:
    """Pinhole camera. R, t map visual board coordinates (mm, z=0 on the board) to the camera frame."""
    K: np.ndarray
    R: np.ndarray
    t: np.ndarray
    size: tuple  # (width, height)

    def board_to_image(self, cfg: BoardConfig) -> np.ndarray:
        """True homography from board millimetres to image pixels."""
        H_visual = self.K @ np.column_stack([self.R[:, 0], self.R[:, 1], self.t])
        return H_visual @ _visual_flip(cfg)


def look_at_camera(
    cfg: BoardConfig,
    distance_mm: float = 520.0,
    tilt_x_deg: float = 0.0,
    tilt_y_deg: float = 0.0,
    size: tuple = (1280, 720),
    focal_px: float = 920.0,
) -> Camera:
    """Camera looking at the centre of the drawing area.

    ``tilt_y_deg`` moves the camera sideways around the board centre (view from
    the right for positive values), ``tilt_x_deg`` moves it up/down. The default
    intrinsics are close to a RealSense D435 colour stream at 1280x720.
    """
    centre_board = np.array([(cfg.x_min + cfg.x_max) / 2.0, (cfg.y_min + cfg.y_max) / 2.0, 1.0])
    target = _visual_flip(cfg) @ centre_board
    target[2] = 0.0
    ax, ay = np.radians(tilt_x_deg), np.radians(tilt_y_deg)
    # The visual frame is right-handed with z pointing INTO the board, so the
    # camera sits at negative z.
    offset = distance_mm * np.array([np.sin(ay) * np.cos(ax), np.sin(ax), -np.cos(ay) * np.cos(ax)])
    position = target + offset
    z_axis = (target - position) / np.linalg.norm(target - position)
    x_axis = np.cross([0.0, 1.0, 0.0], z_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    R = np.vstack([x_axis, y_axis, z_axis])
    t = -R @ position
    K = np.array([[focal_px, 0.0, size[0] / 2.0], [0.0, focal_px, size[1] / 2.0], [0.0, 0.0, 1.0]])
    return Camera(K, R, t, size)


def _draw_strokes(canvas, strokes, to_pixels: np.ndarray, px_per_mm: float, color_of):
    for stroke in strokes:
        pts = np.round(apply_homography(to_pixels, stroke.points)).astype(np.int32)
        thickness = max(1, int(round(stroke.width_mm * px_per_mm)))
        cv2.polylines(canvas, [pts], stroke.closed, color_of(stroke), thickness, cv2.LINE_AA)


def _smear_alpha(shape, smear: Smear, to_pixels: np.ndarray, px_per_mm: float) -> np.ndarray:
    """Per-pixel opacity of a smear: flat in the middle, soft at the edge."""
    u, v = apply_homography(to_pixels, [smear.center])[0]
    yy, xx = np.mgrid[: shape[0], : shape[1]]
    dist_mm = np.hypot(xx - u, yy - v) / px_per_mm
    edge = max(2.0, 0.2 * smear.radius_mm)
    return smear.alpha * np.clip((smear.radius_mm - dist_mm) / edge, 0.0, 1.0)


def ground_truth_mask(cfg: BoardConfig, strokes=(), smears=()) -> np.ndarray:
    """Where ink really is, on the rectified image grid."""
    w, h = cfg.canvas_size
    canvas = np.zeros((h, w), np.uint8)
    _draw_strokes(canvas, strokes, cfg.board_to_canvas, cfg.px_per_mm, lambda s: 255)
    mask = canvas > 127
    for smear in smears:
        mask |= _smear_alpha((h, w), smear, cfg.board_to_canvas, cfg.px_per_mm) > 0.5 * smear.alpha
    return mask


def render_scene(
    cfg: BoardConfig,
    camera: Camera,
    strokes=(),
    smears=(),
    hide_markers=(),
    lighting=(0.80, 1.0),
    glare: dict | None = None,
    noise_sigma: float = 2.0,
    seed: int = 0,
) -> np.ndarray:
    """Render the camera image (BGR, uint8) of the board.

    ``glare``: dict with ``center`` (u, v pixels), ``sigma`` (along, across)
    in pixels, ``angle_deg`` and ``amplitude`` (grey levels added at the peak).
    """
    rng = np.random.default_rng(seed)
    tpm = _TEXTURE_PX_PER_MM

    # 1. Flat texture of the board in the visual frame.
    flip = _visual_flip(cfg)
    corners = apply_homography(
        flip, [[cfg.x_min, cfg.y_min], [cfg.x_max, cfg.y_max]] + list(cfg.marker_centers_mm.values())
    )
    v_min = corners.min(axis=0) - _TEXTURE_MARGIN_MM
    v_max = corners.max(axis=0) + _TEXTURE_MARGIN_MM
    tex_w, tex_h = np.ceil((v_max - v_min) * tpm).astype(int)
    visual_to_tex = np.array([[tpm, 0.0, -v_min[0] * tpm - 0.5], [0.0, tpm, -v_min[1] * tpm - 0.5], [0.0, 0.0, 1.0]])
    board_to_tex = visual_to_tex @ flip
    texture = np.full((tex_h, tex_w, 3), BOARD_BGR, np.uint8)

    dictionary = _aruco_dictionary(cfg.marker_dictionary)
    side = int(round(cfg.marker_size_mm * tpm))
    for marker_id, centre in cfg.marker_centers_mm.items():
        if marker_id in hide_markers:
            continue
        u, v = apply_homography(board_to_tex, [centre])[0]
        x0, y0 = int(round(u + 0.5 - side / 2.0)), int(round(v + 0.5 - side / 2.0))
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, side)
        texture[y0 : y0 + side, x0 : x0 + side] = marker[..., None]

    for smear in smears:
        alpha = _smear_alpha(texture.shape[:2], smear, board_to_tex, tpm)[..., None]
        texture = (texture * (1.0 - alpha) + np.array(INK_BGR[smear.color]) * alpha).astype(np.uint8)
    _draw_strokes(texture, strokes, board_to_tex, tpm, lambda s: INK_BGR[s.color])

    # 2. Project into the camera: texture px -> visual mm -> image px.
    texture = cv2.GaussianBlur(texture, (0, 0), 1.0)  # avoid aliasing when shrinking
    visual_to_image = camera.K @ np.column_stack([camera.R[:, 0], camera.R[:, 1], camera.t])
    tex_to_image = visual_to_image @ np.linalg.inv(visual_to_tex)
    w, h = camera.size
    image = cv2.warpPerspective(
        texture, tex_to_image, (w, h), flags=cv2.INTER_LINEAR, borderValue=WALL_BGR
    ).astype(np.float32)

    # 3. Lighting, glare, sensor noise.
    yy, xx = np.mgrid[:h, :w].astype(np.float32)
    ramp = (0.7 * xx / w + 0.3 * yy / h)
    image *= (lighting[0] + (lighting[1] - lighting[0]) * ramp)[..., None]
    if glare:
        angle = np.radians(glare.get("angle_deg", 0.0))
        dx, dy = xx - glare["center"][0], yy - glare["center"][1]
        along = dx * np.cos(angle) + dy * np.sin(angle)
        across = -dx * np.sin(angle) + dy * np.cos(angle)
        s_along, s_across = glare.get("sigma", (400.0, 40.0))
        bump = np.exp(-0.5 * ((along / s_along) ** 2 + (across / s_across) ** 2))
        image += glare.get("amplitude", 160.0) * bump[..., None]
    if noise_sigma > 0:
        image += rng.normal(0.0, noise_sigma, image.shape)
    return np.clip(image, 0, 255).astype(np.uint8)
