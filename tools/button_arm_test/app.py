#!/usr/bin/env python3
"""Our round buttons (1-4, UP/DOWN) -> pixel -> calibrated arm press. Default: no serial."""
import argparse
import json
import re
import secrets
import threading
import time
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import cv2
import numpy as np
import arm
import calib
import synth
import vision
from vision import ARROWS, DIGITS, Gate

ROOT = Path(__file__).resolve().parent
cv2.setNumThreads(1)
MAX_SAMPLES = 3
TEMPLATE = re.compile(r'template_([1-4])_([0-2])')
JOG_STEPS = (-50, -10, 10, 50)
# Fixed rule: hall call by arrow; inside the car always floor 4 going up, floor 1 going down.
MISSIONS = {('hall', 'up'): 'UP', ('hall', 'down'): 'DOWN', ('car', 'up'): '4', ('car', 'down'): '1'}
# Field rule: button 1 is pressed with servo_test menu 6 (press), button 4 with menu 7 (press2),
# wherever it appears on screen. Other labels use the pixel calibration points.
LABEL_MENU = {'1': '6', '4': '7'}
ARROW_SYMBOL = {'UP': '▲', 'DOWN': '▼'}


def open_camera(source):
    device = int(source) if source.isdigit() else source
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2) if source.startswith('/dev/') else cv2.VideoCapture(device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 10)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def synthetic_source(name):
    labels = ('UP', 'DOWN') if name == 'synthetic-arrows' else DIGITS
    count = 0
    def frame():
        nonlocal count
        count += 1
        return synth.scene(labels, seed=count//30)[0]  # layout jitters every ~3 s
    return frame


class App:
    def __init__(self, args, driver_factory=arm.default_driver):
        self.args = args
        self.data = Path(args.data)
        self.data.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.shutdown = threading.Event()
        self.cancel = threading.Event()
        self.worker = None
        self.templates, self.prepared = {}, {}
        self.stacked = vision.stack({})
        self.points, self.model = [], None
        self.label_menu = dict(LABEL_MENU)
        self.arrow_poses = {}  # ▲/▼ fixed press poses taught by jog, kept in data/config.json
        self.target = 'UP'
        self.gate, self.last_center = Gate(), None
        self.display, self.jpeg, self.jpeg_frame = None, b'', -1
        self.detected, self.observations = [], []
        self.selected = None
        self.updated, self.ready, self.token = 0., None, None
        self.busy, self.message = False, '카메라 연결 중'
        self.error = '카메라 연결 중'
        self.processing_ms = 0.
        self.frames = 0
        self.source = arm.load_source(args.servo_source)
        self.session = arm.Session(self.source, args.port, driver_factory) if args.execute else None
        # Commanded pose is unknown on a real arm until read/home; the preview arm starts at home.
        self.commanded = None if args.execute else dict(self.source['home'])
        self.restore()

    # ---- persistence -------------------------------------------------
    def restore(self):
        file = self.data/'config.json'
        if file.exists():
            cfg = json.loads(file.read_text(encoding='utf-8'))
            self.points = cfg.get('points', [])
            self.model = calib.fit(self.points)
            self.arrow_poses = {k: v for k, v in cfg.get('arrow_poses', {}).items() if k in ARROWS}
            if cfg.get('target') in MISSIONS.values():
                self.target = cfg['target']
        for file in sorted(self.data.glob('template_*.png')):
            match = TEMPLATE.fullmatch(file.stem)
            mask = cv2.imread(str(file), 0)
            if match and mask is not None and mask.shape == (vision.GLYPH_H, vision.GLYPH_W):
                self.add_template(match.group(1), mask, int(match.group(2)))

    def save_config(self):
        text = json.dumps({'points': self.points, 'target': self.target, 'arrow_poses': self.arrow_poses},
                          ensure_ascii=False, indent=2)
        temp = self.data/'config.tmp'
        temp.write_text(text, encoding='utf-8')
        temp.replace(self.data/'config.json')

    def add_template(self, label, mask, index):
        refs = self.templates.setdefault(label, [None]*MAX_SAMPLES)
        refs[index] = mask
        self.prepared[label] = [vision.prepare(m) for m in refs if m is not None]
        self.stacked = vision.stack(self.prepared)

    # ---- configuration -----------------------------------------------
    def invalidate(self):
        self.gate.reset()
        self.ready, self.token = None, None

    def fresh(self):
        return not self.error and time.monotonic()-self.updated < .8

    def set_mission(self, stage, direction):
        if (stage, direction) not in MISSIONS:
            raise ValueError('단계는 hall/car, 방향은 up/down')
        self.set_target(MISSIONS[stage, direction])

    def set_target(self, label):
        if label not in MISSIONS.values():
            raise ValueError('목표는 UP, DOWN, 4(내부 올라감), 1(내부 내려감) 중 하나')
        self.target = label
        self.invalidate()
        self.save_config()

    def select(self, point):
        """Pick the detected button under a click (normalized coords)."""
        if (not isinstance(point, list) or len(point) != 2 or
                any(type(v) not in (int, float) or not 0 <= v <= 1 for v in point)):
            raise ValueError('잘못된 좌표')
        if not self.fresh():
            raise ValueError('카메라 영상이 없습니다')
        x, y = point[0]*640, point[1]*480
        hits = [o for o in self.observations
                if np.hypot(x-o['center_px'][0], y-o['center_px'][1]) <= o['radius']]
        if not hits:
            raise ValueError('검출된 버튼(원) 안을 클릭하세요')
        self.selected = {'px': list(hits[0]['center_px']), 'radius': hits[0]['radius'],
                         'label': hits[0]['label']}
        return self.selected

    def add_point(self, pose_name):
        """Teach: selected button pixel <-> press pose (servo_test menu 6/7 or current jog pose)."""
        if self.selected is None:
            raise ValueError('먼저 영상에서 버튼을 선택하세요')
        if pose_name in self.source['menu_press']:
            press = dict(self.source['menu_press'][pose_name])
        elif pose_name == 'current':
            if self.commanded is None:
                raise ValueError('현재 자세를 모릅니다. 위치 읽기 또는 home 이동 먼저')
            press = dict(self.commanded)
        else:
            raise ValueError('자세는 6, 7, current 중 하나')
        arm.plan(self.source, press)  # rejects poses whose approach/press leave PWM limits
        for p in self.points:
            if np.hypot(p['px'][0]-self.selected['px'][0], p['px'][1]-self.selected['px'][1]) < self.selected['radius']:
                raise ValueError('같은 버튼에 이미 보정점이 있습니다. 삭제 후 다시 추가하세요')
            if p['press'] == press:
                raise ValueError('다른 버튼에 같은 자세가 이미 있습니다. 이 버튼에 닿는 자세를 조그로 찾아 저장하세요')
        self.points.append({'px': self.selected['px'], 'radius': self.selected['radius'],
                            'press': press, 'from': pose_name})
        self.model = calib.fit(self.points)
        self.invalidate()
        self.save_config()

    def save_menu(self, menu):
        """Current jogged pose -> servo_test.py menu 6 (press) / 7 (press2). Points taught from that
        menu follow the new value."""
        if self.commanded is None:
            raise ValueError('현재 자세를 모릅니다. 위치 읽기 또는 home 이동 먼저')
        source = arm.save_menu_press(self.args.servo_source, arm.protocol.__file__,
                                     self.source, menu, self.commanded)
        self.source = source
        if self.session:
            self.session.source = source
        for p in self.points:
            if p['from'] == menu:
                p['press'] = dict(source['menu_press'][menu])
        self.model = calib.fit(self.points)
        self.invalidate()
        self.save_config()
        self.message = '메뉴 %s 자세 저장: %s' % (menu, ' / '.join(str(self.commanded[i]) for i in arm.IDS))

    def save_arrow(self, label):
        """Current jogged pose -> fixed press pose for ▲ (UP) or ▼ (DOWN), wherever it appears."""
        if label not in ARROWS:
            raise ValueError('UP 또는 DOWN')
        if self.commanded is None:
            raise ValueError('현재 자세를 모릅니다. 위치 읽기 또는 home 이동 먼저')
        arm.plan(self.source, self.commanded)  # press and approach poses inside PWM limits
        self.arrow_poses[label] = dict(self.commanded)
        self.invalidate()
        self.save_config()
        self.message = '%s 자세 저장: %s' % (ARROW_SYMBOL[label],
                                         ' / '.join(str(self.commanded[i]) for i in arm.IDS))

    def delete_point(self, index):
        if type(index) is not int or not 0 <= index < len(self.points):
            raise ValueError('없는 보정점')
        del self.points[index]
        self.model = calib.fit(self.points)
        self.invalidate()
        self.save_config()

    def enroll(self, text):
        """Label every visible non-arrow button at once, in reading order."""
        if not isinstance(text, str) or not re.fullmatch('[1-4]{1,4}', text) or len(set(text)) != len(text):
            raise ValueError('보이는 숫자를 왼쪽 위부터 읽는 순서로 입력하세요. 예: 1234')
        if not self.fresh():
            raise ValueError('카메라 영상이 없습니다')
        digits = [ink for label, ink in self.detected if label not in ARROWS]
        if len(digits) != len(text):
            raise ValueError('숫자 버튼 %d개를 찾았는데 입력은 %d개입니다' % (len(digits), len(text)))
        if any(ink is None for ink in digits):
            raise ValueError('글자를 읽지 못한 버튼이 있습니다. 조명·초점을 확인하세요')
        for label, ink in zip(text, digits):
            mask = vision.normalize(ink)
            refs = self.templates.get(label, [])
            filled = [i for i, m in enumerate(refs) if m is not None]
            # Keep newest three samples per digit: fill gaps, then overwrite oldest file.
            index = next((i for i in range(MAX_SAMPLES) if i not in filled), None)
            if index is None:
                index = min(range(MAX_SAMPLES),
                            key=lambda i: (self.data/('template_%s_%d.png' % (label, i))).stat().st_mtime)
            if not cv2.imwrite(str(self.data/('template_%s_%d.png' % (label, index))), mask):
                raise IOError('등록 영상 저장 실패')
            self.add_template(label, mask, index)
        self.invalidate()
        return {label: len(self.prepared[label]) for label in sorted(self.prepared)}

    def clear_templates(self):
        for file in self.data.glob('template_*.png'):
            file.unlink()
        self.templates, self.prepared = {}, {}
        self.stacked = vision.stack({})
        self.invalidate()

    # ---- recognition -------------------------------------------------
    def process(self, frame):
        start = time.monotonic()
        if frame.shape[:2] != (480, 640):
            frame = cv2.resize(frame, (640, 480))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        buttons = vision.reading_order(vision.find_buttons(gray))
        with self.lock:
            self.updated, self.error = start, None
            if self.frames == 0:
                self.message = '버튼을 화면에 보이게 하고 숫자 등록·좌표 보정을 진행하세요'
            self.frames += 1
            found, detected = [], []
            display = frame.copy()
            for button in buttons:
                label, dist, ratio, ink = vision.read_button(gray, button, self.stacked)
                cx, cy = button['center']
                if label in self.label_menu:  # fixed button: always its servo_test menu pose
                    press, reason = dict(self.source['menu_press'][self.label_menu[label]]), None
                elif label in self.arrow_poses:
                    press, reason = dict(self.arrow_poses[label]), None
                elif label in ARROWS:
                    press, reason = None, '%s 자세 미저장 (③ 조그 후 "현재 자세 → %s 저장")' % (
                        ARROW_SYMBOL[label], ARROW_SYMBOL[label])
                else:
                    press, reason = calib.predict(self.model, cx, cy, button['radius'])
                found.append({'label': label, 'distance': round(dist, 2), 'ratio': round(min(ratio, 99), 2),
                              'center_px': [round(cx), round(cy)], 'radius': round(button['radius'], 1),
                              'press': press, 'reason': reason})
                detected.append((label, ink))
                color = ((0, 200, 0) if label == self.target else
                         (0, 0, 230) if label == 'UNKNOWN' else (0, 190, 240))
                cv2.ellipse(display, (round(cx), round(cy)),
                            tuple(round(v) for v in button['axes']), button['angle'], 0, 360, color, 3)
                cv2.putText(display, label, (round(cx-button['radius']), round(cy-button['radius'])-6),
                            cv2.FONT_HERSHEY_SIMPLEX, .7, color, 2)
            for i, p in enumerate(self.points):
                cv2.drawMarker(display, tuple(p['px']), (255, 80, 255), cv2.MARKER_CROSS, 22, 2)
                cv2.putText(display, 'P%d' % (i+1), (p['px'][0]+8, p['px'][1]-8),
                            cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 80, 255), 2)
            if self.selected:
                cv2.circle(display, tuple(self.selected['px']), round(self.selected['radius'])+6, (255, 255, 0), 2)
            self.observations, self.detected = found, detected
            self.display = display  # JPEG is encoded only when the page asks for it
            candidates = [v for v in found if v['label'] == self.target]
            ok = len(candidates) == 1 and not self.busy
            if ok and self.last_center is not None:
                moved = np.hypot(*np.subtract(candidates[0]['center_px'], self.last_center))
                if moved >= .5*candidates[0]['radius']:
                    self.gate.reset()  # target jumped: restart the stability count at this frame
            self.last_center = candidates[0]['center_px'] if len(candidates) == 1 else None
            if self.gate.update((self.target, bool(ok and candidates[0]['press'])) if ok else None, start):
                self.ready = dict(candidates[0])
                if self.token is None and self.ready['press']:
                    self.token = secrets.token_hex(12)
                elif not self.ready['press']:
                    self.token = None
            else:
                self.ready, self.token = None, None
            self.processing_ms = round((time.monotonic()-start)*1000, 2)

    def capture(self):
        cap, fake = None, None
        try:
            if self.args.source.startswith('synthetic'):
                fake = synthetic_source(self.args.source)
            else:
                cap = open_camera(self.args.source)
                if not cap.isOpened():
                    raise RuntimeError('카메라 열기 실패: ' + self.args.source)
            last = 0.
            while not self.shutdown.is_set():
                if fake:
                    if self.shutdown.wait(.1):
                        break
                    frame = fake()
                else:
                    ok, frame = cap.read()
                    if not ok:
                        raise RuntimeError('카메라 프레임 읽기 실패 (USB 재연결 시 앱 재시작)')
                now = time.monotonic()
                if now-last < .095:
                    continue  # Drain camera buffers even when recognition is throttled.
                last = now
                self.process(frame)
        except Exception as exc:
            with self.lock:
                self.error = str(exc)
                self.invalidate()
                self.cancel.set()
        finally:
            if cap is not None:
                cap.release()

    def watchdog(self):
        while not self.shutdown.wait(.2):
            with self.lock:
                if not self.fresh():
                    self.invalidate()
                    if self.busy:
                        self.cancel.set()

    def snapshot(self):
        if not self.fresh() or self.display is None:
            return b''
        if self.jpeg_frame != self.frames:
            ok, jpeg = cv2.imencode('.jpg', self.display, [cv2.IMWRITE_JPEG_QUALITY, 65])
            self.jpeg, self.jpeg_frame = (jpeg.tobytes() if ok else b''), self.frames
        return self.jpeg

    def state(self):
        with self.lock:
            fresh = self.fresh()
            return {'mode': '실제 팔 모드' if self.args.execute else '미리보기 (팔 안 움직임)',
                    'execute': bool(self.args.execute),
                    'error': self.error or (None if fresh else '영상 갱신 대기 / 멈춤'),
                    'target': self.target, 'points': self.points, 'label_menu': self.label_menu, 'arrow_poses': self.arrow_poses,
                    'mission': next('%s-%s' % k for k, v in MISSIONS.items() if v == self.target),
                    'model': self.model['kind'] if self.model else None,
                    'selected': self.selected, 'commanded': self.commanded, 'jog_steps': JOG_STEPS,
                    'samples': {label: len(self.prepared.get(label, [])) for label in DIGITS},
                    'observations': self.observations if fresh else [],
                    'ready': self.ready if fresh else None, 'token': self.token if fresh else None,
                    'busy': self.busy, 'message': self.message, 'frames': self.frames,
                    'processing_ms': self.processing_ms,
                    'servo_sha256': self.source['sha256'], 'source': self.source['path']}

    # ---- arm jobs ----------------------------------------------------
    def run_job(self, name, physical, after=None):
        """Caller holds the lock. Dry-run applies `after` at once; real mode runs in a worker."""
        if self.busy:
            raise ValueError('팔 동작 중')
        if not self.args.execute:
            if after:
                after(None)
            self.message = name + ' · 미리보기 (시리얼 열지 않음)'
            return
        self.busy = True
        self.cancel.clear()
        self.invalidate()
        self.message = name + ' 시작'
        def report(message):
            with self.lock:
                self.message = message
        def run():
            try:
                result = physical(report)
                with self.lock:
                    if after:
                        after(result)
                    if self.message.endswith('시작'):
                        self.message = name + ' 완료'
            except Exception as exc:
                with self.lock:
                    self.commanded = None  # stopped somewhere unknown; read positions again
                report('중지 / 오류: ' + str(exc))
            finally:
                with self.lock:
                    self.busy = False
                    self.invalidate()
        self.worker = threading.Thread(target=run, daemon=True)
        self.worker.start()

    def set_commanded(self, pose):
        def after(_):
            self.commanded = dict(pose)
        return after

    def jog(self, joint, delta):
        if joint not in arm.IDS or delta not in JOG_STEPS:
            raise ValueError('잘못된 조그 값')
        if self.commanded is None:
            raise ValueError('현재 자세를 모릅니다. 위치 읽기 또는 home 이동 먼저')
        pose = dict(self.commanded)
        pose[joint] += delta
        arm.protocol.check_pwm(joint, pose[joint])
        self.run_job('조그 %s %+d' % (joint, delta),
                     lambda report: self.session.move(pose, arm.JOG_MS, self.cancel), self.set_commanded(pose))

    def goto(self, name):
        if name not in arm.GOTO_MS:
            raise ValueError('home 또는 press_ready')
        pose = self.source[name]
        self.run_job(name + ' 이동', lambda report: self.session.move(pose, arm.GOTO_MS[name], self.cancel),
                     self.set_commanded(pose))

    def read_positions(self):
        if not self.args.execute:
            raise ValueError('미리보기 모드에는 실제 위치가 없습니다')
        def after(positions):
            self.commanded = dict(positions)
        self.run_job('위치 읽기', lambda report: self.session.read(), after)

    def press(self, token):
        if self.busy or not self.fresh() or not self.ready or token != self.token or not token:
            raise ValueError('인식이 변경됐거나 준비되지 않았습니다. 다시 확인하세요')
        ready = dict(self.ready)
        steps = arm.plan(self.source, ready['press'])
        self.invalidate()
        if not self.args.execute:
            self.message = '누르기 명령 미리보기 완료 · 팔 포트 열지 않음'
            return {'dry_run': True, 'button': ready, 'steps': steps}
        self.run_job('버튼 %s 누르기' % ready['label'],
                     lambda report: self.session.press(ready['press'], self.cancel, report),
                     self.set_commanded(self.source['home']))
        return {'started': True, 'button': ready, 'physical_contact_verified': False}

    def stop(self):
        self.cancel.set()
        self.invalidate()
        if self.session and not self.busy:
            threading.Thread(target=self.session.stop_idle, daemon=True).start()
        self.message = '중지 요청 · 실제 정지는 현장에서 확인하세요'


def handler(app):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass
        def send(self, value, kind='application/json', code=200):
            body = json.dumps(value, ensure_ascii=False).encode() if isinstance(value, (dict, list)) else value
            self.send_response(code)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)
        def do_GET(self):
            path = urlparse(self.path).path
            if path == '/':
                return self.send((ROOT/'index.html').read_bytes(), 'text/html; charset=utf-8')
            if path == '/api/state':
                return self.send(app.state())
            if path == '/frame.jpg':
                with app.lock:
                    jpeg = app.snapshot()
                return self.send(jpeg, 'image/jpeg', 200 if jpeg else 503)
            return self.send({'error': '없는 경로'}, code=404)
        def do_POST(self):
            try:
                origin = self.headers.get('Origin')
                if origin and urlparse(origin).netloc != self.headers.get('Host'):
                    raise ValueError('다른 사이트의 요청은 허용하지 않습니다')
                if self.headers.get('Content-Type', '').split(';')[0] != 'application/json':
                    raise ValueError('JSON 요청 필요')
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 4096:
                    raise ValueError('요청 크기 오류')
                data = json.loads(self.rfile.read(length))
                path = urlparse(self.path).path
                result = {'ok': True}
                with app.lock:
                    if path == '/api/stop':
                        app.stop()
                    elif app.busy:
                        raise ValueError('팔 동작 중에는 다른 명령을 받을 수 없습니다')
                    elif path == '/api/mission':
                        app.set_mission(data['stage'], data['direction'])
                    elif path == '/api/enroll':
                        result['samples'] = app.enroll(data['labels'])
                    elif path == '/api/clear':
                        app.clear_templates()
                    elif path == '/api/select':
                        result['selected'] = app.select(data['point'])
                    elif path == '/api/point':
                        app.add_point(data['pose'])
                    elif path == '/api/save_menu':
                        app.save_menu(data['menu'])
                    elif path == '/api/save_arrow':
                        app.save_arrow(data['label'])
                    elif path == '/api/point/delete':
                        app.delete_point(data['index'])
                    elif path == '/api/jog':
                        app.jog(data['joint'], data['delta'])
                    elif path == '/api/goto':
                        app.goto(data['pose'])
                    elif path == '/api/read':
                        app.read_positions()
                    elif path == '/api/press':
                        result = app.press(data.get('token'))
                    else:
                        raise ValueError('없는 명령')
                self.send(result)
            except (ValueError, KeyError, TypeError, OSError) as exc:
                self.send({'error': str(exc)}, code=400)
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', default='/dev/v4l/by-id/usb-Generic_USB2.0_PC_CAMERA-video-index0',
                        help='카메라 장치, 또는 synthetic / synthetic-arrows')
    parser.add_argument('--data', default=str(ROOT/'data'))
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--http-port', type=int, default=8091)
    parser.add_argument('--servo-source', default=str(ROOT/'servo_test.py'),
                        help='servo_test.py 경로. import 없이 AST로 값만 읽는다')
    parser.add_argument('--execute', action='store_true', help='실제 팔 시리얼 명령 허용')
    parser.add_argument('--field-approved', action='store_true', help='현장 안전 확인 후에만 지정')
    parser.add_argument('--port', default='/dev/arm_servo')
    args = parser.parse_args()
    if args.execute:
        if not args.field_approved:
            parser.error('실제 모드는 --field-approved 필요')
        if args.host != '127.0.0.1' or args.source.startswith('synthetic'):
            parser.error('실제 모드는 localhost + 실제 카메라만 허용; 원격은 SSH 터널 사용')
        if args.port != '/dev/arm_servo' or not Path(args.port).is_char_device():
            parser.error('/dev/arm_servo 실제 장치 필요')
    app = App(args)
    server = ThreadingHTTPServer((args.host, args.http_port), handler(app))  # fail on busy port before moving
    if args.execute:
        # Same startup as servo_test.py: read all joints, wait for Enter, home over 3000 ms.
        try:
            print('현재 모터 위치:', app.session.read(), flush=True)
            input('주변을 확인하고 Enter를 누르면 home으로 이동합니다.')
            app.session.move(app.source['home'], arm.GOTO_MS['home'], app.cancel)
        except BaseException as exc:
            print('시작 home 실패, 정지:', exc, flush=True)
            app.session.close()
            server.server_close()
            raise SystemExit(1)
        app.commanded = dict(app.source['home'])
        app.message = '시작 home 이동 완료'
    threads = [threading.Thread(target=app.capture, daemon=True),
               threading.Thread(target=app.watchdog, daemon=True)]
    for thread in threads:
        thread.start()
    print('http://%s:%s · %s' % (args.host, args.http_port,
          'PHYSICAL (home 완료)' if args.execute else 'DRY-RUN · 팔 포트를 열지 않음'), flush=True)
    try:
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    finally:
        app.cancel.set()
        app.shutdown.set()
        if app.worker:
            app.worker.join(timeout=8)
        if app.session:
            app.session.close()
        server.server_close()


if __name__ == '__main__':
    main()
