#!/usr/bin/env python3
"""Live target-floor probe: wrong floors, replay, loss and HTTP errors never pass."""
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import threading
import time
import unittest
from unittest.mock import MagicMock, patch

from floor_arrival_probe import FloorArrivalProbe


def state(t, seq, floor="4", stream="camera-a", **changes):
    return dict({"floor": floor, "score": .9, "margin": .2, "error": None,
                 "roi": [[0, 0], [1, 0], [1, 1], [0, 1]], "updated": 1000 + t,
                 "frame_seq": seq, "stream_id": stream}, **changes)


class Tests(unittest.TestCase):
    def probe(self):
        return FloorArrivalProbe("4", 1000)

    def arrive(self, probe):
        for i in range(5):
            t = 1 + i * .1
            result = probe.check(state(t, i + 1), t, 1000 + t)
        self.assertTrue(result["exit_condition_met"])
        self.assertFalse(result["motion_authorized"])
        return result

    def test_four_new_frames_are_insufficient_five_confirm_floor_four(self):
        probe = self.probe()
        for i in range(5):
            t = 1 + i * .1
            result = probe.check(state(t, i + 1), t, 1000 + t)
            self.assertEqual(result["exit_condition_met"], i == 4)
            self.assertEqual(result["confirmed_frames"], i + 1)
            self.assertFalse(result["motion_authorized"])
        self.assertEqual(result["target_floor"], "F4")
        self.assertEqual(result["current_floor"], "F4")

    def test_other_floor_and_unknown_never_confirm(self):
        for floor in ("1", "2", "3", "5", "UNKNOWN"):
            with self.subTest(floor=floor):
                probe = self.probe()
                for i in range(10):
                    t = 1 + i * .1
                    result = probe.check(state(t, i + 1, floor), t, 1000 + t)
                    self.assertFalse(result["exit_condition_met"])

    def test_leaving_floor_four_revokes_even_with_old_target_event(self):
        probe = self.probe()
        self.arrive(probe)
        snapshot = state(1.5, 6, "3", event={"event": "TARGET_FLOOR_DETECTED", "floor": "4"})
        result = probe.check(snapshot, 1.5, 1001.5)
        self.assertFalse(result["exit_condition_met"])
        self.assertEqual(result["confirmed_frames"], 0)

    def test_wrong_floor_between_matches_resets_consecutive_count(self):
        probe = self.probe()
        for i in range(4):
            t = 1 + i * .1
            probe.check(state(t, i + 1), t, 1000 + t)
        probe.check(state(1.4, 5, "3"), 1.4, 1001.4)
        result = probe.check(state(1.5, 6), 1.5, 1001.5)
        self.assertEqual(result["confirmed_frames"], 1)
        self.assertFalse(result["exit_condition_met"])

    def test_repeated_http_snapshot_counts_as_one_frame(self):
        probe = self.probe()
        for i in range(7):
            t = 1 + i * .1
            result = probe.check(state(1, 1), t, 1000 + t)
        self.assertEqual(result["confirmed_frames"], 1)
        self.assertFalse(result["exit_condition_met"])

    def test_old_snapshot_and_missing_reader_revoke_confirmation(self):
        for snapshot in (state(1.4, 5), None):
            with self.subTest(snapshot=snapshot):
                probe = self.probe()
                self.arrive(probe)
                result = probe.check(snapshot, 3, 1003)
                self.assertFalse(result["exit_condition_met"])
                self.assertEqual(result["current_floor"], "UNKNOWN")

    def test_reader_restart_requires_five_new_frames(self):
        probe = self.probe()
        self.arrive(probe)
        for i in range(5):
            t = 1.5 + i * .1
            result = probe.check(state(t, i + 1, stream="camera-b"), t, 1000 + t)
            self.assertEqual(result["exit_condition_met"], i == 4)

    def test_bad_confidence_missing_roi_and_camera_error_revoke(self):
        for change in ({"score": .6}, {"margin": .01}, {"roi": None},
                       {"error": "camera disconnected"}):
            with self.subTest(change=change):
                probe = self.probe()
                self.arrive(probe)
                result = probe.check(state(1.5, 6, **change), 1.5, 1001.5)
                self.assertFalse(result["exit_condition_met"])

    def test_frames_before_pose_settles_do_not_confirm(self):
        probe = self.probe()
        for i in range(5):
            t = i * .1
            result = probe.check(state(t, i + 1), t, 1000 + t)
        self.assertEqual(result["confirmed_frames"], 0)

    def test_http_poll_reads_live_state(self):
        response = MagicMock()
        response.__enter__.return_value.read.return_value = json.dumps(state(1, 1)).encode()
        with patch("floor_arrival_probe.urlopen", return_value=response), \
                patch("floor_arrival_probe.time.monotonic", return_value=1), \
                patch("floor_arrival_probe.time.time", return_value=1001):
            result = self.probe().poll("http://127.0.0.1:8765/api/state")
        self.assertEqual(result["current_floor"], "F4")
        self.assertEqual(result["confirmed_frames"], 1)

    def test_live_http_endpoint_confirms_four_and_revokes_on_three(self):
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                self.server.frame_seq += 1
                seq = self.server.frame_seq
                snapshot = state(time.time() - 1000, seq, "4" if seq <= 5 else "3",
                                 event={"floor": "4", "event": "TARGET_FLOOR_DETECTED"})
                body = json.dumps(snapshot).encode()
                self.send_response(200)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, *args):
                pass

        server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        server.frame_seq = 0
        thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": .01}, daemon=True)
        thread.start()
        probe = FloorArrivalProbe("4", time.time() - 1)
        try:
            url = "http://127.0.0.1:%s/api/state" % server.server_port
            for i in range(6):
                result = probe.poll(url)
                self.assertEqual(result["exit_condition_met"], i == 4)
                self.assertFalse(result["motion_authorized"])
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=1)

    def test_http_failure_and_invalid_json_revoke(self):
        probe = self.probe()
        self.arrive(probe)
        with patch("floor_arrival_probe.urlopen", side_effect=OSError("offline")):
            result = probe.poll("http://127.0.0.1:8765/api/state")
        self.assertFalse(result["exit_condition_met"])
        self.assertFalse(result["motion_authorized"])
        self.assertIn("OSError", result["reader_error"])
        probe = self.probe()
        self.arrive(probe)
        response = MagicMock()
        response.__enter__.return_value.read.return_value = b"invalid JSON"
        with patch("floor_arrival_probe.urlopen", return_value=response):
            result = probe.poll("http://127.0.0.1:8765/api/state")
        self.assertFalse(result["exit_condition_met"])


if __name__ == "__main__":
    unittest.main()
