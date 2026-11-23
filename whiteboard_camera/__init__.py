"""Whiteboard camera module: find leftover ink on the whiteboard."""
from .config import BoardConfig, load_config
from .pipeline import (
    InspectionResult,
    RectificationError,
    WhiteboardInspector,
    camera_info_for_image,
    draw_camera_view,
    draw_debug,
    load_camera_info,
    save_camera_info,
    undistort,
)

__all__ = [
    "BoardConfig",
    "load_config",
    "InspectionResult",
    "RectificationError",
    "WhiteboardInspector",
    "camera_info_for_image",
    "draw_camera_view",
    "draw_debug",
    "load_camera_info",
    "save_camera_info",
    "undistort",
]
