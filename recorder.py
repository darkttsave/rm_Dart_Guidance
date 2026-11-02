"""Optional development-only MJPEG/AVI recording, isolated from UART.

Adapted from the recording functionality of Shenzhen University RobotPilots
2026 main_final.py (RoboMaster BBS article 1938958).
"""

import os


def next_record_name(basename):
    """Preserve SZU-style record01.avi naming; reuse highest empty file."""
    max_number = 0
    max_path = None
    for name in os.listdir():
        if name.startswith(basename) and name.endswith(".avi"):
            middle = name[len(basename):-4]
            try:
                number = int(middle)
            except ValueError:
                continue
            if number > max_number:
                max_number = number
                max_path = name

    if max_path is not None:
        try:
            if os.stat(max_path)[6] == 0:
                return max_path
        except OSError:
            pass
    return "%s%02d.avi" % (basename, max_number + 1)


class VideoRecorder:
    """Recorder failures are reported but never stop the vision/UART loop."""

    def __init__(self, recording_cfg):
        self.enabled = recording_cfg["enabled"]
        self.every_n_frames = recording_cfg["every_n_frames"]
        self.basename = recording_cfg["basename"]
        self.writer = None
        self.frame_count = 0

    def start(self):
        self.frame_count = 0
        if not self.enabled:
            return
        try:
            import mjpeg
            filename = next_record_name(self.basename)
            self.writer = mjpeg.Mjpeg(filename)
            print("Recording:", filename)
        except Exception as exc:
            self.writer = None
            print("Recording unavailable:", exc)

    def capture(self, img):
        if self.writer is None:
            return
        try:
            if self.frame_count % self.every_n_frames == 0:
                self.writer.write(img)
        except Exception as exc:
            print("Recording write failed:", exc)
            self.close()
        finally:
            self.frame_count += 1

    def close(self):
        if self.writer is None:
            return
        writer = self.writer
        self.writer = None
        try:
            writer.close()
        except Exception as exc:
            print("Recording close failed:", exc)
