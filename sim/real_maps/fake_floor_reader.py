#!/usr/bin/env python3
"""Simulation-only floor reader: serves the sim elevator's current floor over HTTP.

Same GET /api/state keys as tools/floor_reader/app.py (floor, score, margin, error, updated,
frame_seq, stream_id, target, roi, event, labels) and the same TARGET_FLOOR_DETECTED gate
(10 recent labels within 1.5 s, last == target, >= 8 matches). POST /target {"floor":"2"} sets
the target like the real app. No camera, no image recognition: the label is copied from
/elevator/state (F2 -> "2"). Binds 127.0.0.1 inside the isolated simulation container only.
"""
import collections
import json
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

LABELS = ['1', '2', '3', '4', '5', '6']


class Gate:
    """Copy of tools/floor_reader/app.py Gate semantics."""
    def __init__(self):
        self.reset()

    def reset(self):
        self.history = collections.deque(maxlen=10)
        self.sent = False

    def update(self, label, target, now):
        self.history.append((now, label))
        recent = [v for t, v in self.history if now-t <= 1.5]
        ok = len(recent) == 10 and recent[-1] == target and recent.count(target) >= 8
        if target and ok and not self.sent:
            self.sent = True
            return True
        return False


class FakeReader(Node):
    def __init__(self):
        super().__init__('sim_fake_floor_reader')
        self.declare_parameter('port', 8765)
        self.declare_parameter('target', '')
        self.lock = threading.RLock()
        self.target = str(self.get_parameter('target').value) or None
        self.stream_id = 'sim-'+uuid.uuid4().hex[:8]
        self.status = {'floor': 'UNKNOWN', 'error': '시뮬 엘리베이터 상태 대기', 'score': 0}
        self.frame_seq = self.seq = 0
        self.last_event = None
        self.gate = Gate()
        self.create_subscription(String, '/elevator/state', self.on_state, 10)
        self.create_timer(.1, self.tick)   # same 10 Hz sampling as the camera loop
        self.label = None

    def on_state(self, msg):
        floor = json.loads(msg.data).get('current_floor', '')
        self.label = floor[1:] if floor.upper().startswith('F') and floor[1:] in LABELS else None

    def tick(self):
        now = time.monotonic()
        with self.lock:
            self.frame_seq += 1
            if self.label is None:
                self.status = {'floor': 'UNKNOWN', 'score': 0, 'updated': time.time(),
                               'error': '시뮬 엘리베이터 상태 없음'}
                self.gate.reset()
                return
            self.status = {'floor': self.label, 'score': 1.0, 'margin': 1.0, 'error': None,
                           'updated': time.time(), 'frame_seq': self.frame_seq, 'stream_id': self.stream_id}
            if self.gate.update(self.label, self.target, now):
                self.seq += 1
                self.last_event = {'event': 'TARGET_FLOOR_DETECTED', 'floor': self.label, 'seq': self.seq,
                                   'timestamp': time.time(), 'motion_authorized': False}
                self.get_logger().info(json.dumps(self.last_event))

    def state(self):
        with self.lock:
            status = dict(self.status)
            if time.time()-status.get('updated', 0) > 3 and not status.get('error'):
                status.update(floor='UNKNOWN', score=0, error='카메라 영상 갱신이 멈췄습니다')
            return {**status, 'target': self.target, 'roi': None, 'event': self.last_event,
                    'labels': LABELS, 'source': 'simulation'}


def handler(reader):
    class Handler(BaseHTTPRequestHandler):
        def send(self, body, code=200):
            data = json.dumps(body, ensure_ascii=False).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if urlparse(self.path).path == '/api/state':
                return self.send(reader.state())
            return self.send({'error': 'not found'}, 404)

        def do_POST(self):
            if self.path != '/target':
                return self.send({'error': 'not found'}, 404)
            try:
                body = json.loads(self.rfile.read(int(self.headers.get('Content-Length', 0))) or b'{}')
                label = str(body.get('floor', '')).upper()
            except (ValueError, json.JSONDecodeError):
                return self.send({'error': 'bad request'}, 400)
            if label not in LABELS:
                return self.send({'error': '잘못된 층수'}, 400)
            with reader.lock:
                reader.target = label
                reader.gate.reset()
            return self.send({'ok': True, 'target': label})

        def log_message(self, *args):
            pass
    return Handler


def main():
    rclpy.init()
    node = FakeReader()
    server = ThreadingHTTPServer(('127.0.0.1', int(node.get_parameter('port').value)), handler(node))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
