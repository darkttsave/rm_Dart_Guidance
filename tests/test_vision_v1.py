"""CPython host-side contract tests; not an OpenMV or F411 hardware test."""

import unittest

from vision_v1 import (
    IDLE, WAIT, SEARCH, DONE, READY_BANNER, START_COMMAND, FRAME_HEADER,
    VisualSession, choose_largest_rect, clamp_roi, rect_offset,
    pack_frame, find_target, load_config
)
from recorder import VideoRecorder


class FakeBlob:
    def __init__(self, rect):
        self._rect = rect

    def rect(self):
        return self._rect


class FakeImage:
    def __init__(self, rects):
        self.rects = rects
        self.last_roi = None

    def find_blobs(self, thresholds, roi=None):
        self.last_roi = roi
        return [FakeBlob(rect) for rect in self.rects]


class TestProtocol(unittest.TestCase):
    def test_banner_is_distinct(self):
        self.assertEqual(READY_BANNER, bytes.fromhex("FF 52 50 44 41 52 54"))
        self.assertEqual(START_COMMAND, 0xAB)
        self.assertEqual(FRAME_HEADER, bytes.fromhex("FF 01"))

    def test_signed_big_endian(self):
        self.assertEqual(
            pack_frame(SEARCH, 1, 15, -10),
            bytes.fromhex("FF 01 02 01 00 0F FF F6")
        )
        self.assertEqual(pack_frame(SEARCH, 1, -160, -120),
                         bytes.fromhex("FF 01 02 01 FF 60 FF 88"))
        self.assertEqual(pack_frame(SEARCH, 1, 159, 119),
                         bytes.fromhex("FF 01 02 01 00 9F 00 77"))

    def test_all_inactive_states(self):
        for state in (IDLE, WAIT, SEARCH, DONE):
            self.assertEqual(pack_frame(state), bytes((255, 1, state, 0, 0, 0, 0, 0)))

    def test_illegal_observations(self):
        for args in ((4,), (IDLE, 1), (DONE, 1), (SEARCH, 2),
                     (SEARCH, 0, 1, 0), (SEARCH, 1, 160, 0),
                     (SEARCH, 1, 0, -121)):
            with self.subTest(args=args):
                with self.assertRaises(ValueError):
                    pack_frame(*args)


class TestSession(unittest.TestCase):
    def test_one_shot_delay_and_search_window(self):
        s = VisualSession(1000, 2000)
        diff = lambda a, b: a - b
        self.assertEqual(s.state, IDLE)
        self.assertTrue(s.trigger(100))
        self.assertFalse(s.trigger(101))
        self.assertEqual(s.advance(1099, diff), WAIT)
        self.assertTrue(s.ready_to_search(1100, diff))
        self.assertEqual(s.advance(1100, diff), SEARCH)
        self.assertEqual(s.search_started_ms, 1100)
        self.assertEqual(s.advance(3099, diff), SEARCH)
        self.assertEqual(s.advance(3100, diff), DONE)
        self.assertFalse(s.trigger(3200))
        self.assertEqual(s.advance(4000, diff), DONE)

    def test_zero_delay(self):
        s = VisualSession(0, 2000)
        s.trigger(23)
        self.assertEqual(s.advance(23, lambda a, b: a - b), SEARCH)


class TestVision(unittest.TestCase):
    def test_bbox_area_matches_szu_selection(self):
        self.assertEqual(choose_largest_rect(((0, 0, 10, 20), (2, 4, 11, 20))),
                         (2, 4, 11, 20))
        self.assertIsNone(choose_largest_rect(()))

    def test_center_and_offsets(self):
        self.assertEqual(rect_offset((150, 110, 20, 20)), (0, 0))
        self.assertEqual(rect_offset((160, 100, 30, 20)), (15, -10))

    def test_roi_clamp(self):
        self.assertEqual(clamp_roi((0, 0, 10, 15), 40, 40), (0, 0, 50, 55))
        self.assertEqual(clamp_roi((300, 230, 20, 10), 40, 40), (260, 190, 60, 50))
        for rect in ((0, 0, 10, 15), (300, 230, 20, 10), (150, 110, 20, 20)):
            x, y, w, h = clamp_roi(rect, 40, 40)
            self.assertGreater(w, 0)
            self.assertGreater(h, 0)
            self.assertLessEqual(x + w, 320)
            self.assertLessEqual(y + h, 240)

    def test_find_target_and_missing_target(self):
        cfg = {"vision": {"lab_threshold": [9, 100, -102, -25, 0, 127]}}
        img = FakeImage([(0, 0, 10, 10), (7, 9, 20, 20)])
        self.assertEqual(find_target(img, cfg, (0, 0, 320, 240)), (7, 9, 20, 20))
        self.assertEqual(img.last_roi, (0, 0, 320, 240))
        self.assertIsNone(find_target(FakeImage([]), cfg, (0, 0, 320, 240)))


class TestConfig(unittest.TestCase):
    def test_json_and_disabled_recorder(self):
        cfg = load_config("config.json")
        self.assertFalse(cfg["recording"]["enabled"])
        self.assertEqual(cfg["timing"]["search_ms"], 2000)
        recorder = VideoRecorder(cfg["recording"])
        recorder.start()
        recorder.capture(object())
        recorder.close()
        self.assertIsNone(recorder.writer)


if __name__ == "__main__":
    unittest.main()
