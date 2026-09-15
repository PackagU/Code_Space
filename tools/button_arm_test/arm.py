"""servo_test.py values (read by AST, never imported) + one serial session for jog/press."""
import ast
import hashlib
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import servo_reference as protocol

IDS = protocol.SERVO_IDS
TOLERANCE_PWM = 30        # [제안값] PRAD 도달 판정
FEEDBACK_TIMEOUT = 1.5    # [제안값] s
JOG_MS = 400              # [제안값] 조그 1회 이동 시간
GOTO_MS = {'home': 3000, 'press_ready': 2000}  # servo_test.py 메뉴 1/2 시간


def load_source(path):
    raw = Path(path).read_bytes()
    tree = ast.parse(raw.decode('utf-8-sig'))
    values = {}
    # Resolve literal constants and references only. Never execute source code.
    def literal(node):
        if isinstance(node, ast.Name):
            return values[node.id]
        if isinstance(node, ast.Dict):
            return {literal(k): literal(v) for k, v in zip(node.keys, node.values)}
        if isinstance(node, (ast.Tuple, ast.List)):
            return tuple(literal(v) for v in node.elts)
        return ast.literal_eval(node)
    for node in tree.body:
        if isinstance(node, ast.Assign):
            try:
                value = literal(node.value)
            except (ValueError, KeyError, TypeError):
                continue
            for target in node.targets:
                if isinstance(target, ast.Name):
                    values[target.id] = value
    if values.get('SERVO_IDS') != protocol.SERVO_IDS or values.get('PWM_LIMITS') != protocol.PWM_LIMITS:
        raise ValueError('원본 ID/PWM_LIMITS가 기준본과 다릅니다. 임의 변환하지 않습니다')
    poses = values.get('POSES', {})
    for cycle in (1, 2):
        expected = protocol.get_poses(cycle)
        if {name: poses.get(name) for name in expected} != expected:
            raise ValueError('원본 포즈가 기준본과 다릅니다. 원본에 맞춘 구현 확인 필요')
    # servo_test.py backs off from the button by the same joint offset in both cycles.
    approach = {i: poses['pre_press'][i] - poses['press'][i] for i in IDS}
    if approach != {i: poses['pre_press2'][i] - poses['press2'][i] for i in IDS}:
        raise ValueError('메뉴 6/7의 접근 오프셋이 달라 일반화할 수 없습니다')
    return {'path': str(Path(path).resolve()), 'sha256': hashlib.sha256(raw).hexdigest(),
            'baud': values.get('BAUD', 115200), 'home': poses['home'],
            'press_ready': poses['press_ready'], 'approach': approach,
            'menu_press': {'6': poses['press'], '7': poses['press2']}}


MENU_POSES = {'6': (1, 'pre_press', 'press', 'retreat'), '7': (2, 'pre_press2', 'press2', 'retreat2')}


def _dict_nodes(tree, variable):
    """{pose name: ast.Dict} for `variable = {...}` at module level."""
    for node in tree.body:
        if (isinstance(node, ast.Assign) and isinstance(node.value, ast.Dict) and
                any(isinstance(t, ast.Name) and t.id == variable for t in node.targets)):
            return {ast.literal_eval(k): v for k, v in zip(node.value.keys, node.value.values)}
    raise ValueError('%s 표를 찾지 못했습니다' % variable)


def _splice(text, edits):
    """Replace (lineno, col, end_lineno, end_col) spans; columns are UTF-8 byte offsets."""
    lines = text.split('\n')
    for (l1, c1, l2, c2), new in sorted(edits, reverse=True):
        head, tail = lines[l1-1].encode()[:c1].decode(), lines[l2-1].encode()[c2:].decode()
        lines[l1-1:l2] = (head + new + tail).split('\n')
    return '\n'.join(lines)


def save_menu_press(servo_path, reference_path, source, menu, press):
    """Write a jogged press pose back to servo_test.py menu 6/7 (and the matching reference table).
    pre_press/retreat keep servo_test's approach offset. Only numeric literals are rewritten."""
    if menu not in MENU_POSES:
        raise ValueError('메뉴는 6 또는 7')
    press = {i: int(press[i]) for i in IDS}
    plan(source, press)  # limits for press and its approach pose, before touching any file
    cycle, pre_name, press_name, retreat_name = MENU_POSES[menu]
    pre = {i: press[i] + source['approach'][i] for i in IDS}
    new = {pre_name: pre, press_name: press, retreat_name: pre}
    servo_path, reference_path = Path(servo_path), Path(reference_path)
    servo_text = servo_path.read_text(encoding='utf-8')
    tables = _dict_nodes(ast.parse(servo_text), 'POSES')
    edits = []
    for name, pose in new.items():
        node = tables[name]
        for key, value in zip(node.keys, node.values):
            edits.append(((value.lineno, value.col_offset, value.end_lineno, value.end_col_offset),
                          str(pose[ast.literal_eval(key)])))
    ref_text = reference_path.read_text(encoding='utf-8')
    ref_tables = _dict_nodes(ast.parse(ref_text), 'POSES_%d' % cycle)
    ref_edits = [((n.lineno, n.col_offset, n.end_lineno, n.end_col_offset),
                  '{%s}' % ', '.join('"%s": %d' % (i, pose[i]) for i in IDS))
                 for name, pose in new.items() for n in [ref_tables[name]]]
    stamp = time.strftime('%Y%m%d_%H%M%S')
    written = []
    old_tables = {name: protocol.POSE_TABLES[cycle][name] for name in new}
    try:
        for path, text, changes in ((servo_path, servo_text, edits), (reference_path, ref_text, ref_edits)):
            path.with_name('%s.bak_%s' % (path.name, stamp)).write_bytes(text.encode('utf-8'))
            temp = path.with_name(path.name + '.tmp')
            temp.write_bytes(_splice(text, changes).encode('utf-8'))
            temp.replace(path)
            written.append((path, text))
        protocol.POSE_TABLES[cycle].update(new)  # keep the loaded reference equal to the file
        return load_source(servo_path)
    except BaseException:
        protocol.POSE_TABLES[cycle].update(old_tables)
        for path, text in written:
            path.write_bytes(text.encode('utf-8'))
        raise


def cycle_poses(source, press):
    pre = {i: press[i] + source['approach'][i] for i in IDS}
    return {'home': source['home'], 'press_ready': source['press_ready'],
            'pre_press': pre, 'press': dict(press), 'retreat': pre}


def plan(source, press):
    """Same steps and times as servo_test.py run_press_cycle; raises if any PWM is out of limits."""
    poses = cycle_poses(source, press)
    return [{'pose': name, 'duration_ms': ms, 'pwm': poses[name],
             'payload': protocol.pose_command(name, ms, poses) if send else None}
            for name, ms, send in protocol.PRESS_CYCLE_1]


def default_driver(port, baud):
    import serial
    # Refuse a second cooperating serial owner; do not reset/reopen other tools.
    def connection(*args, **kwargs):
        return serial.Serial(*args, exclusive=True, **kwargs)
    driver = protocol.SerialPoseDriver(port, baud, timeout=.5,
                                       serial_module=SimpleNamespace(Serial=connection))
    time.sleep(.5)  # servo_test.py waits after opening
    return driver


class Session:
    """Opens the port on the first physical command and keeps it (no reopen per jog)."""
    def __init__(self, source, port, driver_factory=default_driver):
        self.source, self.port, self.factory = source, port, driver_factory
        self.driver, self.io = None, threading.Lock()

    def _open(self):
        if self.driver is None:
            self.driver = self.factory(self.port, self.source['baud'])
        return self.driver

    def _fault(self):
        try:
            if self.driver is not None:
                self.driver.stop_all()
        finally:
            self.close()

    def read(self):
        with self.io:
            try:
                return self._open().read_positions()
            except BaseException:
                self._fault()
                raise

    def move(self, pose, duration_ms, cancel):
        with self.io:
            try:
                self._send_and_verify('target', duration_ms, {'target': pose}, cancel)
            except BaseException:
                self._fault()
                raise

    def press(self, press_pose, cancel, report):
        """Home feedback first; every pose verified; fault/cancel -> stop, no auto retreat."""
        poses = cycle_poses(self.source, press_pose)
        steps = plan(self.source, press_pose)  # validates limits before any byte is sent
        with self.io:
            try:
                driver = self._open()
                if cancel.is_set():
                    raise RuntimeError('중지 요청')
                if not protocol.positions_reached(driver.read_positions(), self.source['home'], TOLERANCE_PWM):
                    raise RuntimeError('home 응답 불일치: 먼저 home으로 이동하세요')
                for step in steps:
                    report('동작 중: ' + step['pose'])
                    if step['payload']:
                        self._send_and_verify(step['pose'], step['duration_ms'], poses, cancel)
                    elif cancel.wait(step['duration_ms']/1000.):
                        raise RuntimeError('중지 요청')
                report('사이클 응답 확인 완료 · 실제 눌림은 현장에서 확인하세요')
            except BaseException:
                self._fault()
                raise

    def _send_and_verify(self, name, duration_ms, poses, cancel):
        driver = self._open()
        if cancel.is_set():
            raise RuntimeError('중지 요청')
        driver.send_pose(name, duration_ms, poses)
        if cancel.wait(duration_ms/1000.):
            raise RuntimeError('중지 요청')
        deadline = time.monotonic() + FEEDBACK_TIMEOUT
        while not protocol.positions_reached(driver.read_positions(), poses[name], TOLERANCE_PWM):
            if cancel.is_set():
                raise RuntimeError('중지 요청')
            if time.monotonic() >= deadline:
                raise TimeoutError('관절 응답 불일치: ' + name)

    def stop_idle(self):
        """Stop request while no job runs. A running job sees `cancel` and stops itself."""
        if self.io.acquire(timeout=.2):
            try:
                if self.driver is not None:
                    self.driver.stop_all()
            finally:
                self.io.release()

    def close(self):
        driver, self.driver = self.driver, None
        if driver is not None:
            driver.close()
