"""OpenMV V1 pure logic: config, states, ROI and 8-byte UART frames.

The blob-selection approach is adapted from Shenzhen University RobotPilots
2026 main_final.py (RoboMaster BBS article 1938958, CC BY-NC-SA 4.0).
This module intentionally does not import sensor/pyb so host tests can run.
"""

try:
    import json
except ImportError:  # older MicroPython builds
    import ujson as json


IDLE = 0
WAIT = 1
SEARCH = 2
DONE = 3
READY_BANNER = bytes((0xFF, 0x52, 0x50, 0x44, 0x41, 0x52, 0x54))
START_COMMAND = 0xAB
FRAME_HEADER = bytes((0xFF, 0x01))


def load_config(path="config.json"):
    """Read required JSON once at boot; reject malformed configurations."""
    with open(path, "r") as config_file:
        cfg = json.load(config_file)
    validate_config(cfg)
    return cfg


def validate_config(cfg):
    """Fail fast; do not silently substitute defaults for wrong V1 settings."""
    timing = cfg["timing"]
    vision = cfg["vision"]
    camera = cfg["camera"]
    recording = cfg["recording"]

    if not isinstance(timing["delay_ms"], int) or timing["delay_ms"] < 0:
        raise ValueError("timing.delay_ms must be a nonnegative integer")
    if not isinstance(timing["search_ms"], int) or timing["search_ms"] <= 0:
        raise ValueError("timing.search_ms must be a positive integer")
    if not isinstance(timing["status_interval_ms"], int) or timing["status_interval_ms"] <= 0:
        raise ValueError("timing.status_interval_ms must be positive")
    if len(vision["lab_threshold"]) != 6:
        raise ValueError("vision.lab_threshold must have 6 LAB values")
    if not all(isinstance(v, int) for v in vision["lab_threshold"]):
        raise ValueError("vision.lab_threshold values must be integers")
    if vision["roi_margin_x"] < 0 or vision["roi_margin_y"] < 0:
        raise ValueError("ROI margins cannot be negative")
    if len(camera["readout_window"]) != 4:
        raise ValueError("camera.readout_window requires x,y,w,h")
    if camera["readout_window"][2] <= 0 or camera["readout_window"][3] <= 0:
        raise ValueError("camera.readout_window has invalid size")
    if camera["exposure_us"] <= 0 or camera["warmup_ms"] < 0:
        raise ValueError("camera exposure/warmup invalid")
    if not isinstance(recording["enabled"], bool):
        raise ValueError("recording.enabled must be boolean")
    if not isinstance(recording["every_n_frames"], int) or recording["every_n_frames"] < 1:
        raise ValueError("recording.every_n_frames must be >= 1")
    if not isinstance(recording["basename"], str) or not recording["basename"]:
        raise ValueError("recording.basename must be nonempty")
    if "/" in recording["basename"] or "\\" in recording["basename"]:
        raise ValueError("recording.basename cannot contain a directory separator")


class VisualSession:
    """One-shot IDLE -> WAIT -> SEARCH -> DONE state machine."""

    def __init__(self, delay_ms, search_ms):
        self.delay_ms = delay_ms
        self.search_ms = search_ms
        self.state = IDLE
        self.trigger_ms = None
        self.search_started_ms = None

    def trigger(self, now_ms):
        if self.state != IDLE:
            return False
        self.trigger_ms = now_ms
        self.state = WAIT
        return True

    def ready_to_search(self, now_ms, ticks_diff):
        return (
            self.state == WAIT
            and ticks_diff(now_ms, self.trigger_ms) >= self.delay_ms
        )

    def advance(self, now_ms, ticks_diff):
        if self.ready_to_search(now_ms, ticks_diff):
            self.state = SEARCH
            self.search_started_ms = now_ms
        elif self.state == SEARCH and ticks_diff(now_ms, self.search_started_ms) >= self.search_ms:
            self.state = DONE
        return self.state


def pack_frame(state, valid=0, dx=0, dy=0):
    """FF 01 state valid dx_H dx_L dy_H dy_L; big-endian signed int16."""
    if state not in (IDLE, WAIT, SEARCH, DONE):
        raise ValueError("invalid visual state")
    if valid not in (0, 1):
        raise ValueError("valid must be 0 or 1")
    if valid:
        if state != SEARCH:
            raise ValueError("valid=1 requires SEARCH")
        if not (-160 <= dx <= 159 and -120 <= dy <= 119):
            raise ValueError("pixel offset outside QVGA")
    elif dx != 0 or dy != 0:
        raise ValueError("invalid observation must have zero offsets")
    dx_u16 = dx & 0xFFFF
    dy_u16 = dy & 0xFFFF
    return bytes((
        0xFF, 0x01, state, valid,
        (dx_u16 >> 8) & 0xFF, dx_u16 & 0xFF,
        (dy_u16 >> 8) & 0xFF, dy_u16 & 0xFF
    ))


def choose_largest_rect(rects):
    """Select largest bounding rectangle by w*h, as in the SZU baseline."""
    largest = None
    max_area = -1
    for rect in rects:
        area = rect[2] * rect[3]
        if area > max_area:
            largest = rect
            max_area = area
    return largest


def rect_offset(rect, width=320, height=240):
    """Center of bounding rectangle relative to image center; right/down positive."""
    x, y, w, h = rect
    return x + w // 2 - width // 2, y + h // 2 - height // 2


def clamp_roi(rect, margin_x, margin_y, width=320, height=240):
    """Clip expanded software ROI to image; never return an out-of-bounds ROI."""
    x, y, w, h = rect
    x0 = max(0, x - margin_x)
    y0 = max(0, y - margin_y)
    x1 = min(width, x + w + margin_x)
    y1 = min(height, y + h + margin_y)
    if x1 <= x0 or y1 <= y0:
        return (0, 0, width, height)
    return (x0, y0, x1 - x0, y1 - y0)


def find_target(img, cfg, roi):
    """Use original LAB find_blobs/max bbox area; return rect or None."""
    threshold = tuple(cfg["vision"]["lab_threshold"])
    blobs = img.find_blobs([threshold], roi=roi)
    return choose_largest_rect(blob.rect() for blob in blobs)
