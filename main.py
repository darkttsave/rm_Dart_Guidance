"""RoboMaster dart OpenMV H750 visual V1 (one-shot, no flight control).

Adapted from Shenzhen University RobotPilots 2026 main_final.py:
https://bbs.robomaster.com/article/1938958 (article marks CC BY-NC-SA 4.0).
No real H750/F411 hardware validation has been performed for this adaptation.
"""

import time
import sensor
from pyb import UART

from vision_v1 import (
    IDLE, WAIT, SEARCH, DONE, START_COMMAND, READY_BANNER,
    VisualSession, find_target, rect_offset, clamp_roi, load_config, pack_frame
)
from recorder import VideoRecorder


def init_camera(cfg):
    """Keep SZU QVGA/LAB camera baseline with one fixed READOUT window."""
    camera = cfg["camera"]
    sensor.reset()
    sensor.set_framesize(sensor.QVGA)
    sensor.set_pixformat(sensor.RGB565)
    sensor.ioctl(sensor.IOCTL_SET_READOUT_WINDOW, tuple(camera["readout_window"]))
    sensor.set_windowing((0, 0, 320, 240))
    sensor.set_auto_gain(False, gain_db=camera["gain_db"])
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(False, exposure_us=camera["exposure_us"])
    sensor.skip_frames(time=camera["warmup_ms"])
    # Retain the original pre-trigger snapshot and optional camera sleep.
    sensor.snapshot()
    if camera["sleep_until_trigger"]:
        sensor.sleep(True)


def draw_overlay(img, rect, roi):
    if rect is not None:
        x, y, w, h = rect
        img.draw_rectangle(x, y, w, h, color=(255, 0, 0))
    img.draw_rectangle(roi[0], roi[1], roi[2], roi[3], color=(0, 0, 255))


def read_start(uart):
    """Ignore old 11/22/55/FF commands; V1 only reacts to AB."""
    count = uart.any()
    if count <= 0:
        return False
    incoming = uart.read(count)
    return incoming is not None and START_COMMAND in incoming


def send_observation(uart, state, valid=0, dx=0, dy=0):
    uart.write(pack_frame(state, valid, dx, dy))


def run():
    cfg = load_config()
    # Wire contract: UART3, 115200, 8N1. Do not casually change unilaterally.
    uart = UART(3, 115200)
    init_camera(cfg)

    # Ready banner is not an eight-byte visual frame.
    uart.write(READY_BANNER)
    session = VisualSession(cfg["timing"]["delay_ms"], cfg["timing"]["search_ms"])
    recorder = VideoRecorder(cfg["recording"])
    status_interval = cfg["timing"]["status_interval_ms"]

    send_observation(uart, IDLE)
    last_status_ms = time.ticks_ms()
    last_warmup_ms = None
    current_roi = (0, 0, 320, 240)

    while True:
        now = time.ticks_ms()
        state = session.state

        if state == IDLE:
            if read_start(uart) and session.trigger(time.ticks_ms()):
                send_observation(uart, WAIT)
                last_status_ms = time.ticks_ms()
                if cfg["camera"]["sleep_until_trigger"]:
                    sensor.sleep(False)
                last_warmup_ms = None
                continue

        elif state == WAIT:
            if session.ready_to_search(now, time.ticks_diff):
                # Opening the optional recording file is outside SEARCH's 2s timer.
                recorder.start()
                session.advance(time.ticks_ms(), time.ticks_diff)
                current_roi = (0, 0, 320, 240)
                continue
            # Capture throwaway images to allow camera/exposure to settle.
            # This can block; 20Hz status in WAIT is a target, not a hard guarantee.
            if last_warmup_ms is None or time.ticks_diff(now, last_warmup_ms) >= status_interval:
                sensor.snapshot()
                last_warmup_ms = time.ticks_ms()

        elif state == SEARCH:
            if session.advance(now, time.ticks_diff) == DONE:
                recorder.close()
                send_observation(uart, DONE)
                last_status_ms = time.ticks_ms()
                continue

            # Exactly one observation per new snapshot: never replay old valid=1.
            img = sensor.snapshot()
            if session.advance(time.ticks_ms(), time.ticks_diff) == DONE:
                recorder.close()
                send_observation(uart, DONE)
                last_status_ms = time.ticks_ms()
                continue

            rect = find_target(img, cfg, current_roi)
            if rect is None:
                current_roi = (0, 0, img.width(), img.height())
                send_observation(uart, SEARCH, valid=0)
            else:
                dx, dy = rect_offset(rect, img.width(), img.height())
                send_observation(uart, SEARCH, valid=1, dx=dx, dy=dy)
                current_roi = clamp_roi(
                    rect,
                    cfg["vision"]["roi_margin_x"],
                    cfg["vision"]["roi_margin_y"],
                    img.width(), img.height()
                )

            if cfg["vision"]["draw_overlay"]:
                draw_overlay(img, rect, current_roi)
            recorder.capture(img)
            # No separate periodic SEARCH state frame: every frame is a new image.
            continue

        else:  # DONE: ignore start commands, keep sending inactive status.
            count = uart.any()
            if count:
                uart.read(count)

        now = time.ticks_ms()
        if time.ticks_diff(now, last_status_ms) >= status_interval:
            send_observation(uart, session.state)
            last_status_ms = now
        time.sleep_ms(1)


if __name__ == "__main__":
    run()
