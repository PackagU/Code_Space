"""신공학관 E2E 촬영 리허설 진행기 본체.

원칙
- 로봇이 움직이는 단계는 사람이 Enter 로 시작 신호를 준다. 소프트웨어만 하는 단계(위치 지정·지도 전환)는
  앞 단계가 끝나면 바로 이어서 한다.
- Nav2 는 처음부터 끝까지 켜 둔다. 층 전환은 지도만 바꾼다(재시작 없음). 수동 구간은 goal 이 없을 때
  같은 터미널에서 텔레옵을 연다.
- 움직이기 전에는 resume, 끝나면 stop. 실패·Ctrl+C 는 즉시 stop(+지게팔 '!', 팔 중지) 후 메뉴를 띄운다.
- 팔 자세·누르기는 버튼 웹사이트 값 그대로. 이 파일은 팔 값을 만들거나 바꾸지 않는다.
"""

import csv
import datetime as dt
import json
import time
from pathlib import Path

from .backends import BackendError

KST = dt.timezone(dt.timedelta(hours=9))
STEP_TYPES = ("pose", "goal", "teleop", "lift", "move", "approach", "button", "wait", "map")
ROBOT_TYPES = ("goal", "lift", "move", "approach", "button")
NAV_TYPES = ("pose", "goal", "teleop", "move", "approach", "map")
BUTTON_LABELS = {("hall", "up"): "UP", ("hall", "down"): "DOWN", ("car", "up"): "4", ("car", "down"): "1"}
BUTTON_SHOWN = {"UP": "▲", "DOWN": "▼", "4": "4", "1": "1"}


class Abort(Exception):
    """운영자가 중단을 골랐다."""


class StepFailed(Exception):
    """이 단계가 실패했다. 메시지는 운영자에게 보인다."""


def load_scenario(path):
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    seen = set()
    for step in data["steps"]:
        if step["type"] not in STEP_TYPES:
            raise ValueError(f"{step['id']}: unknown step type {step['type']}")
        if step["id"] in seen:
            raise ValueError(f"duplicate step id {step['id']}")
        seen.add(step["id"])
        if step["type"] in ("pose", "goal", "map") and not step.get("waypoint"):
            raise ValueError(f"{step['id']}: waypoint required")
        if step["type"] == "button" and (step.get("stage"), step.get("direction")) not in BUTTON_LABELS:
            raise ValueError(f"{step['id']}: button needs stage hall/car and direction up/down")
        if step["type"] == "lift" and step.get("action") not in ("up", "down"):
            raise ValueError(f"{step['id']}: lift action must be up or down")
        if step["type"] == "map" and step.get("floor") not in ("F1", "F2", "F3"):
            raise ValueError(f"{step['id']}: map floor must be F1/F2/F3")
        if step["type"] == "move" and not 0.01 <= abs(float(step.get("distance_m", 0))) <= 1.0:
            raise ValueError(f"{step['id']}: move distance must be 0.01..1.0 m")
        if step["type"] == "approach":
            if not 0.01 <= float(step.get("max_distance_m", 0)) <= 1.0:
                raise ValueError(f"{step['id']}: approach max_distance_m must be 0.01..1.0 m (forward only)")
            if not 1.0 <= float(step.get("stop_cm", 0)) <= 200.0:
                raise ValueError(f"{step['id']}: approach stop_cm must be 1..200")
    return data


def select_steps(steps, start=None, end=None, only=None):
    ids = [s["id"] for s in steps]
    for name in [start, end, *(only or [])]:
        if name and name not in ids:
            raise ValueError(f"unknown step id {name} (choose from {', '.join(ids)})")
    if only:
        return [s for s in steps if s["id"] in only]
    first = ids.index(start) if start else 0
    last = ids.index(end) if end else len(ids) - 1
    if last < first:
        raise ValueError("--to step comes before --from step")
    return steps[first:last + 1]


def state_before(steps, step_id, floor="F1", lift="bottom"):
    """step_id 직전까지 시나리오가 끝났다고 볼 때의 층·포크 위치·마지막 위치."""
    waypoint = None
    for step in steps:
        if step["id"] == step_id:
            break
        if step["type"] == "map":
            floor, waypoint = step["floor"], step["waypoint"]
        elif step["type"] in ("pose", "goal"):
            waypoint = step["waypoint"]
        elif step["type"] == "lift":
            lift = "top" if step["action"] == "up" else "bottom"
    return {"floor": floor, "lift": lift, "waypoint": waypoint}


def clock(seconds):
    seconds = int(max(0, seconds))
    return f"{seconds // 60:02d}:{seconds % 60:02d}"


# ---------------------------------------------------------------- UI and log
class ConsoleUI:
    def __init__(self, answers=None, auto=False, out=print):
        self.answers = list(answers or [])
        self.auto = auto
        self.out = out

    def say(self, text=""):
        self.out(text)

    def ask(self, text):
        if self.answers:
            answer = self.answers.pop(0)
            self.out(f"{text}{answer}  (scripted)")
            return answer
        if self.auto:
            self.out(f"{text}(auto Enter)")
            return ""
        return input(text).strip().lower()


class RunLog:
    def __init__(self, root, label):
        stamp = dt.datetime.now(KST).strftime("%Y%m%d_%H%M%S")
        self.dir = Path(root) / f"{stamp}_{label}"
        self.dir.mkdir(parents=True, exist_ok=True)
        self.events = self.dir / "events.jsonl"

    def event(self, kind, **fields):
        record = {"at_kst": dt.datetime.now(KST).isoformat(timespec="seconds"), "kind": kind, **fields}
        with self.events.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record, ensure_ascii=False) + "\n")

    def write_results(self, results, meta):
        with (self.dir / "steps.csv").open("w", encoding="utf-8", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(["id", "title", "type", "status", "attempts", "adjusts", "seconds", "detail"])
            for r in results:
                writer.writerow([r["id"], r["title"], r["type"], r["status"], r["attempts"], r["adjusts"],
                                 f"{r['seconds']:.1f}", r["detail"]])
        lines = [f"# 촬영 리허설 기록 {meta['label']}", "",
                 f"- 시작: {meta['started_kst']}", f"- 전체 시간: {clock(meta['total_seconds'])}",
                 f"- 결과: {meta['verdict']}", f"- 사람이 대신한 단계: {meta['manual']}개, "
                 f"로봇 단계 실패 후 재시도: {meta['retries']}회, 수동 위치 보정: {meta['adjusts']}회",
                 f"- 모드: {meta['mode']}", "",
                 "| 단계 | 내용 | 결과 | 시도 | 보정 | 시간 | 비고 |", "|---|---|---|---|---|---|---|"]
        for r in results:
            lines.append(f"| {r['id']} | {r['title']} | {r['status']} | {r['attempts']} | {r['adjusts']} | "
                         f"{clock(r['seconds'])} | {r['detail'].replace('|', '/')} |")
        (self.dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
        return "\n".join(lines)


# ---------------------------------------------------------------- rehearsal
class Rehearsal:
    def __init__(self, scenario, field, web, lift, ui, log, nav_restart=False, record=True,
                 allow_preview_arm=False, label="rehearsal", sleep=time.sleep, ultrasonic=None):
        self.scenario = scenario
        self.settings = scenario.get("settings", {})
        self.field, self.web, self.lift = field, web, lift
        self.ultrasonic = ultrasonic
        self.ultrasonic_open = False
        self.ui, self.log = ui, log
        self.nav_restart = nav_restart
        self.record = record
        self.allow_preview_arm = allow_preview_arm
        self.label = label
        self.sleep = sleep
        self.results = []
        self.floor = "F1"
        self.recording = False
        self.lift_open = False
        self.started = time.monotonic()
        self.started_kst = dt.datetime.now(KST).isoformat(timespec="seconds")

    # -- helpers
    def say(self, text=""):
        self.ui.say(text)

    def elapsed(self):
        return time.monotonic() - self.started

    def safe_stop(self, why):
        self.say(f"  ■ 정지: {why}")
        self.log.event("safe_stop", why=why)
        if self.lift_open and (getattr(self.lift, "moving", False) or getattr(self.lift, "stop_unconfirmed", False)):
            if hasattr(self.lift, "stop_and_confirm") and not self.lift.stop_and_confirm():
                self.say("  !! 지게팔 정지 확인 실패 — 물리 비상정지를 누르세요")
        try:
            self.field.stop()
        except BackendError as exc:
            self.say(f"  !! {exc}")

    def _with_motion(self, action):
        """정지 유지 상태에서 goal 없음·/cmd_vel 단독 확인 → resume → 동작 → 무조건 stop."""
        self.field.ensure_idle()
        self.field.resume()
        try:
            return action()
        finally:
            self.field.stop()

    # -- step handlers: return (status, detail); raise StepFailed on failure
    def do_pose(self, step):
        wp = self.field.waypoint(step["waypoint"])
        if wp["floor"] != self.floor:
            raise StepFailed(f"{step['waypoint']} is on {wp['floor']} but {self.floor} is loaded")
        if not self.field.map_guard(self.floor):
            raise StepFailed(f"{self.floor} map guard failed (loaded map is not the pinned one)")
        self.field.pose_set(wp["x"], wp["y"], wp["yaw"])
        return "OK", f"pose {step['waypoint']}"

    def do_goal(self, step):
        wp = self.field.waypoint(step["waypoint"])
        if wp["floor"] != self.floor:
            raise StepFailed(f"{step['waypoint']} is on {wp['floor']} but {self.floor} is loaded")
        retries = int(self.settings.get("goal_retry_limit", 1))
        status = "unknown"
        for attempt in range(retries + 1):
            ok, status = self._with_motion(lambda: self.field.goal(step["waypoint"], wp["floor"]))
            self.log.event("goal", step=step["id"], waypoint=step["waypoint"], attempt=attempt + 1, status=status)
            if ok:
                self._auto_retries += attempt
                return "OK", f"{step['waypoint']} {status}" + (f" (자동 재시도 {attempt}회)" if attempt else "")
            if attempt < retries:
                self.say(f"  ↻ goal {status}: costmap 을 비우고 자동으로 한 번 더 보냅니다")
                cleared, data = self.field.clear_costmaps()
                if not cleared:
                    self._auto_retries += attempt + 1
                    raise StepFailed(f"goal {step['waypoint']} {status}; costmap clear failed: {data.get('reason')}")
        self._auto_retries += retries
        raise StepFailed(f"goal {step['waypoint']} {status}")

    def do_teleop(self, step):
        self.say("  키보드 조종: w/s/a/d 이동 · k 또는 Space 정지 · z 감속 · 끝나면 Ctrl+C")
        rc = self._with_motion(self.field.teleop)
        if rc not in (0, 130):
            raise StepFailed(f"keyboard teleop exited with code {rc} (조종이 열리지 않았거나 비정상 종료)")
        while step.get("confirm"):
            answer = self.ui.ask(f"  {step['confirm']}  [Enter=예 · r=다시 조종 · q=중단] ")
            if answer == "":
                break
            if answer == "r":
                raise StepFailed("operator: not at the target yet")
            if answer == "q":
                raise Abort(step["id"])
        return "OK", "manual drive"

    def do_move(self, step):
        distance = float(step["distance_m"])
        speed = float(self.settings.get("straight_speed_mps", 0.05))
        ok, data = self._with_motion(lambda: self.field.straight(distance, speed))
        if not ok:
            raise StepFailed(f"straight {distance:+.2f} m failed: {data.get('reason')}")
        return "OK", f"straight {distance:+.2f} m (moved {data.get('traveled_m', '?')} m)"

    def us_text(self):
        if self.ultrasonic is None:
            return "초음파 없음"
        value, age = self.ultrasonic.reading()
        if value is None:
            return "초음파 값 없음"
        return f"초음파 {value:.1f} cm" + ("" if age <= self.ultrasonic.stale_sec else f" (오래됨 {age:.1f} s)")

    def do_approach(self, step):
        """초음파 거리가 stop_cm 이하가 되면 멈추는 저속 전진. 값이 끊기면 즉시 정지·실패."""
        if self.ultrasonic is None:
            raise StepFailed("ultrasonic is not configured (settings.ultrasonic)")
        if not self.ultrasonic.fresh():
            raise StepFailed(f"ultrasonic has no fresh reading ({self.us_text()}) — 배선·펌웨어 출력 확인")
        stop_cm = float(step["stop_cm"])
        value, _ = self.ultrasonic.reading()
        if value is not None and value <= stop_cm:
            return "OK", f"already at {value:.1f} cm (≤ {stop_cm:.0f} cm), no motion"
        speed = float(self.settings.get("approach_speed_mps", 0.03))
        last = {"value": None, "printed": 0.0}

        def check():
            value, age = self.ultrasonic.reading()
            if value is None or age > self.ultrasonic.stale_sec:
                return "fail:ultrasonic reading lost during approach"
            last["value"] = value
            now = time.monotonic()
            if now - last["printed"] >= 1.0:
                last["printed"] = now
                self.say(f"  … 접근 중 {value:.1f} cm (멈춤 {stop_cm:.0f} cm)")
            return "stop" if value <= stop_cm else None

        ok, data = self._with_motion(lambda: self.field.straight_until(float(step["max_distance_m"]), speed, check))
        self.log.event("approach", step=step["id"], stop_cm=stop_cm, last_cm=last["value"], **{
            k: v for k, v in data.items() if k in ("reason", "traveled_m")})
        if not ok:
            raise StepFailed(f"approach failed: {data.get('reason')} (last {last['value']} cm)")
        return "OK", f"stopped at {last['value']} cm after {data.get('traveled_m', '?')} m"

    def do_lift(self, step):
        self.field.stop()
        if getattr(self.lift, "stop_unconfirmed", False):
            if not self.lift.stop_and_confirm():
                raise StepFailed("lift stop is still unconfirmed — 지게팔 상태를 눈으로 확인하고 진행기를 다시 시작")
        action = step["action"]
        state = self.lift.status()
        if state.get("reference") != "CONFIRMED":
            raise StepFailed(f"lift reference is {state.get('reference')} — 포크 끝 위치를 눈으로 확인하고 다시 잡아야 함")
        expected = 0 if action == "up" else int(self.settings.get("lift_move_size", 166400))
        if int(state.get("position_pulses", -1)) != expected:
            raise StepFailed(f"lift is at {state.get('position_pulses')} pulses, expected {expected} before {action}")
        last = [0.0]

        def progress(seconds):
            if seconds - last[0] >= 5.0:
                last[0] = seconds
                self.say(f"  … 지게팔 {'올리는' if action == 'up' else '내리는'} 중 {seconds:4.0f} s")

        before = self.us_text()
        self.lift.move(action, float(self.settings.get("lift_move_timeout_sec", 90)), progress)
        after = self.us_text()
        self.log.event("lift_ultrasonic", step=step["id"], before=before, after=after)
        return "OK", f"lift {action} complete ({before} → {after})"

    def do_button(self, step):
        self.field.stop()
        label = BUTTON_LABELS[(step["stage"], step["direction"])]
        shown = BUTTON_SHOWN[label]
        try:
            self.web.set_mission(step["stage"], step["direction"])
            token = self._wait_button_ready(label, float(self.settings.get("button_ready_timeout_sec", 45)))
            self.say(f"  ▶ 버튼 {shown} 인식 안정 → 누릅니다")
            self.web.press(token)
            message = self._wait_press_done(float(self.settings.get("button_press_timeout_sec", 60)))
        except BaseException:
            self.web.stop()  # 팔 동작 취소 요청 (Ctrl+C·실패 모두)
            raise
        answer = self.ui.ask(f"  버튼 {shown} 불이 켜졌나요?  [Enter=예 · r=다시 누르기 · h=사람이 눌렀음] ")
        if answer == "r":
            raise StepFailed(f"button {shown}: lamp not lit after press")
        if answer == "h":
            return "MANUAL", f"button {shown}: robot press done, lamp off → pressed by hand"
        return "OK", f"button {shown}: {message}"

    def _wait_button_ready(self, label, timeout):
        deadline = time.monotonic() + timeout
        last_print = 0.0
        while time.monotonic() < deadline:
            state = self.web.state()
            ready, token = state.get("ready"), state.get("token")
            if ready and ready.get("label") == label:
                if token:
                    return token
                if ready.get("reason"):
                    raise StepFailed(f"button {label} seen but cannot press: {ready['reason']}")
            now = time.monotonic()
            if now - last_print >= 2.0:
                last_print = now
                seen = ", ".join(o.get("label", "?") for o in state.get("observations", [])) or "없음"
                self.say(f"  … 버튼 {BUTTON_SHOWN[label]} 찾는 중 (보이는 버튼: {seen}"
                         + (f", {state['error']}" if state.get("error") else "") + ")")
            self.sleep(0.25)
        raise StepFailed(f"button {BUTTON_SHOWN[label]} not stably recognized within {timeout:.0f} s"
                         " — t 로 버튼이 화면에 보이게 위치를 맞추고 다시")

    def _wait_press_done(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            state = self.web.state()
            if not state.get("busy"):
                message = state.get("message") or ""
                if "오류" in message or "중지" in message:
                    raise StepFailed(f"arm press failed: {message}")
                return message
            self.sleep(0.25)
        self.web.stop()
        raise StepFailed(f"arm press did not finish within {timeout:.0f} s (stop requested)")

    def do_wait(self, step):
        return "OK", "operator confirmed"

    def do_map(self, step):
        floor = step["floor"]
        wp = self.field.waypoint(step["waypoint"])
        if wp["floor"] != floor:
            raise StepFailed(f"{step['waypoint']} is on {wp['floor']}, not {floor}")
        self.field.stop()
        if self.nav_restart:
            if self.field.arm_node_running():
                raise StepFailed("ROS arm node is running: Nav2 restart would be refused — use load_map mode")
            self.field.stop_navigation()
            self.field.start_background(f"nav_{floor}", ["nav", "start", floor], r"Managed nodes are active",
                                        150, self.log.dir)
        else:
            ok, data = self.field.load_map(self.field.pinned_map(floor))
            if not ok:
                raise StepFailed(f"load_map {floor} failed: {data.get('reason')}")
        self.floor = floor
        if not self.field.map_guard(floor):
            raise StepFailed(f"{floor} map guard failed after switch")
        self.field.pose_set(wp["x"], wp["y"], wp["yaw"])
        cleared, data = self.field.clear_costmaps()
        if not cleared:
            raise StepFailed(f"costmap clear after pose failed: {data.get('reason')}")
        return "OK", f"{floor} map + pose {step['waypoint']}"

    # -- per-step loop
    def menu_text(self, step, failed):
        if failed and step["type"] in ("teleop", "wait"):
            return "  r=다시  q=중단 > "
        if failed:
            return "  r=다시  t=위치 수동 보정 후 다시  s=사람이 대신함  q=중단 > "
        kind = step["type"]
        if kind == "wait":
            return "  Enter=확인됨  q=중단 > "
        if kind == "teleop":
            return "  Enter=조종 시작" + ("  n=필요 없음" if step.get("optional") else "") + "  q=중단 > "
        return "  Enter=시작  t=위치 수동 보정  s=사람이 대신함  q=중단 > "

    def banner(self, step, index, total):
        self.say("━" * 64)
        self.say(f" [{step['id']} · {index}/{total}] {step['title']}    (현재 층 지도 {self.floor} · 경과 {clock(self.elapsed())})")
        if step.get("hint"):
            self.say(f"  · {step['hint']}")
        if step["type"] in ("approach", "lift") and self.ultrasonic is not None:
            self.say(f"  · {self.us_text()}")
        if step.get("show_floor") and self.web is not None:
            try:
                floor = self.web.state().get("floor", {})
                self.say(f"  · 팔 카메라 층수 인식: {floor.get('floor', '?')} (점수 {floor.get('score', '?')})")
            except BackendError:
                pass

    def adjust(self, step):
        self.say("  수동 보정: 위치를 맞춘 뒤 Ctrl+C 로 조종을 끝내면 이 단계를 다시 묻습니다")
        self.log.event("adjust", step=step["id"])
        try:
            self._with_motion(self.field.teleop)
        except BackendError as exc:
            self.say(f"  !! {exc}")

    def run_step(self, step, index, total):
        handler = getattr(self, "do_" + step["type"])
        started = time.monotonic()
        attempts, adjusts, failed, detail = 0, 0, False, ""
        self._auto_retries = 0
        self.banner(step, index, total)
        while True:
            if failed or not step.get("auto"):
                choice = self.ui.ask(self.menu_text(step, failed))
                if choice == "q":
                    raise Abort(step["id"])
                if choice == "n" and step["type"] == "teleop" and step.get("optional"):
                    return self._result(step, "SKIPPED", attempts, adjusts, started, "not needed")
                if choice == "s" and step["type"] not in ("wait", "teleop"):
                    return self._result(step, "MANUAL", attempts, adjusts, started,
                                        "done by a person" + (f" after: {detail}" if detail else ""))
                if choice == "t" and step["type"] not in ("wait", "teleop"):
                    adjusts += 1
                    self.adjust(step)
                    continue
                if failed and choice not in ("r", ""):
                    continue
            attempts += 1
            self.log.event("step_start", step=step["id"], attempt=attempts)
            try:
                status, detail = handler(step)
                return self._result(step, status, attempts, adjusts, started, detail)
            except KeyboardInterrupt:
                detail = "Ctrl+C"
            except (StepFailed, BackendError) as exc:
                detail = str(exc)
            failed = True
            self.log.event("step_fail", step=step["id"], attempt=attempts, detail=detail)
            self.say(f"  ✗ {step['id']} 실패: {detail}")
            self.safe_stop("step failed")

    def _result(self, step, status, attempts, adjusts, started, detail):
        seconds = time.monotonic() - started
        result = {"id": step["id"], "title": step["title"], "type": step["type"], "status": status,
                  "attempts": attempts, "adjusts": adjusts, "seconds": seconds, "detail": detail,
                  "auto_retries": getattr(self, "_auto_retries", 0)}
        self.results.append(result)
        self.log.event("step_done", **result)
        mark = {"OK": "✓", "MANUAL": "✋", "SKIPPED": "–"}.get(status, "?")
        self.say(f"  {mark} {step['id']} {status} ({clock(seconds)}) {detail}")
        return result

    # -- prep / finish
    def prep(self, selected, all_steps):
        types = {s["type"] for s in selected}
        start = state_before(all_steps, selected[0]["id"])
        self.floor = start["floor"]
        self.say("━" * 64)
        self.say(f" 사전 점검 — 시작 단계 {selected[0]['id']}, 예상 상태: 지도 {start['floor']}, "
                 f"포크 {'위 끝' if start['lift'] == 'top' else '아래 끝'}, 마지막 위치 {start['waypoint'] or '-'}")
        if not self.field.container_running():
            raise Abort("ROS container is not running")
        status = self.field.status()
        if status.get("base") == "running":
            # 단계 종류와 관계없이 맨 먼저 멈춘다(이전 실행의 goal 이 살아 있을 수 있음)
            self.field.stop()
            if status.get("navigation") == "running":
                self.field.ensure_idle()
        web_state = self._prep_button_web() if "button" in types else None
        if types & set(NAV_TYPES):
            self._prep_navigation(start["floor"], status, web_state,
                                  restart_needed=any(s["type"] == "map" for s in selected))
        if "lift" in types or (types & {"approach"} and self.ultrasonic is not None and not self.ultrasonic.port):
            self._prep_lift(start["lift"])
        if "approach" in types:
            self._prep_ultrasonic()
        if self.record:
            self.recording = True
            ok = self.field.record_start(self.label)
            self.say("  rosbag 녹화 " + ("확인됨" if ok else "시작했지만 용량 증가 미확인 — 진단 판정에서 제외"))
        if types & set(NAV_TYPES) and selected[0]["type"] not in ("pose", "map") and start["waypoint"]:
            wp = self.field.waypoint(start["waypoint"])
            answer = self.ui.ask(f"  로봇이 {start['waypoint']} 에 있나요? [Enter=그 좌표로 위치 지정 · k=지금 위치 유지] ")
            if answer != "k":
                self.field.pose_set(wp["x"], wp["y"], wp["yaw"])
        self.field.stop()
        self.log.event("prep_done", start=start)

    def _prep_navigation(self, floor, status, web_state=None, restart_needed=False):
        ros_arm = bool(web_state and web_state.get("ros_arm")) or self.field.arm_node_running()
        if self.nav_restart and restart_needed and ros_arm:
            raise Abort("--nav-restart cannot be used while the ROS arm node runs "
                        "(start_field_navigation.sh refuses to start while the arm node runs)")
        if status.get("navigation") == "running":
            pass  # prep() 에서 이미 stop + ensure_idle
        elif ros_arm:
            raise Abort("Nav2 is off but the button web runs with --ros-arm. Order: base → Nav2(F1) → arm node "
                        "→ website, then rerun (start_field_navigation.sh refuses while the arm node runs)")
        if status.get("base") != "running":
            if self.ui.ask("  base 가 꺼져 있습니다. 바퀴 구동 포함으로 켤까요? [Enter=켜기 · q=중단] ") == "q":
                raise Abort("base not running")
            self.field.start_background("base", ["base", "start", "--drive"], r"process started with pid",
                                        90, self.log.dir)
            self.sleep(10)
        if status.get("navigation") != "running":
            self.say(f"  Nav2 를 {floor} 지도로 켭니다 (goal 은 보내지 않음)")
            for attempt in (1, 2):
                try:
                    self.field.start_background(f"nav_{floor}", ["nav", "start", floor],
                                                r"Managed nodes are active", 150, self.log.dir)
                    self.field.stop()
                    break
                except BackendError as exc:
                    if attempt == 2:
                        raise
                    self.say(f"  Nav2 시작 실패({exc}) — base 준비를 15 초 기다린 뒤 한 번 더")
                    self.sleep(15)
        if not self.field.map_guard(floor):
            if self.nav_restart:
                raise Abort(f"loaded map is not the pinned {floor} map")
            self.say(f"  실행 중 지도가 {floor} 고정본이 아니라서 {floor} 지도로 바꿉니다")
            ok, data = self.field.load_map(self.field.pinned_map(floor))
            if not ok or not self.field.map_guard(floor):
                raise Abort(f"could not load the pinned {floor} map: {data.get('reason')}")

    def _prep_button_web(self):
        state = self.web.state()
        if state.get("error"):
            raise Abort(f"button web camera not ready: {state['error']}")
        self.say(f"  버튼 웹사이트: {state.get('mode')} · 프레임 {state.get('frames')}")
        if not state.get("execute") and not self.allow_preview_arm:
            raise Abort("button web is in preview mode (arm will not move). Start it with --execute or --ros-arm")
        if state.get("busy"):
            raise Abort("arm is busy on the button web; wait or press 중지")
        return state

    def _prep_lift(self, expected):
        self.lift.open()
        self.lift_open = True
        state = self.lift.status()
        size = int(state.get("move_size", -1))
        if size != int(self.settings.get("lift_move_size", 166400)):
            raise Abort(f"lift board move_size={size}; wrong board or firmware")
        where = "아래 끝" if expected == "bottom" else "위 끝(택배 실은 상태)"
        if self.ui.ask(f"  포크가 눈으로 보아 완전히 {where}인가요? [Enter=예 · q=중단] ") == "q":
            raise Abort("lift position not confirmed")
        self.lift.set_reference(expected)

    def _prep_ultrasonic(self):
        if self.ultrasonic is None:
            raise Abort("scenario has an approach step but no ultrasonic sensor is configured")
        self.ultrasonic.open()
        self.ultrasonic_open = True
        for _ in range(30):
            if self.ultrasonic.fresh():
                break
            self.sleep(0.1)
        if not self.ultrasonic.fresh():
            raise Abort("no fresh ultrasonic reading within 3 s — 아두이노가 'US distance_cm=<값>' 줄을 내보내는지 확인")
        self.say(f"  초음파: {self.us_text()} (포트 {self.ultrasonic.port or '지게팔 Uno 와 공유'})")

    def _cleanup(self, label, action):
        try:
            action()
        except BaseException as exc:  # noqa: BLE001 — 한 정리가 실패해도 나머지는 한다
            self.say(f"  !! 정리 실패 ({label}): {exc}")
            self.log.event("cleanup_error", what=label, error=str(exc))

    def _lift_final(self):
        state = self.lift.status()
        self.say(f"  지게팔 최종 위치 {state.get('position_pulses')} pulses")

    def finish(self, aborted=None):
        self._cleanup("stop", self.field.stop)
        if self.recording:
            self._cleanup("record stop", self.field.record_stop)
        if self.lift_open:
            self._cleanup("lift status", self._lift_final)
            self._cleanup("lift close", self.lift.close)
        if self.ultrasonic_open:
            self._cleanup("ultrasonic close", self.ultrasonic.close)
        manual = sum(1 for r in self.results if r["status"] == "MANUAL")
        retries = sum(max(0, r["attempts"] - 1) + r.get("auto_retries", 0) for r in self.results)
        adjusts = sum(r["adjusts"] for r in self.results)
        if aborted:
            verdict = f"중단 ({aborted})"
        elif manual == 0 and retries == 0 and adjusts == 0:
            verdict = "한 번에 통과 — 이 순서 그대로 촬영 가능"
        else:
            verdict = "완주했지만 개입 있음 — 표의 재시도·보정 단계를 먼저 손볼 것"
        meta = {"label": self.label, "started_kst": self.started_kst,
                "total_seconds": self.elapsed(), "verdict": verdict, "manual": manual, "retries": retries,
                "adjusts": adjusts, "mode": "nav restart" if self.nav_restart else "nav persistent (load_map)"}
        text = self.log.write_results(self.results, meta)
        self.log.event("finish", **meta)
        self.say("━" * 64)
        self.say(text)
        self.say(f"\n기록 폴더: {self.log.dir}")
        return meta

    def run(self, selected, all_steps):
        aborted = None
        try:
            self.prep(selected, all_steps)
            self.started = time.monotonic()
            for index, step in enumerate(selected, 1):
                self.run_step(step, index, len(selected))
        except Abort as exc:
            aborted = str(exc)
            self.safe_stop(f"abort: {aborted}")
        except KeyboardInterrupt:
            aborted = "Ctrl+C"
            self.safe_stop("Ctrl+C")
        except Exception as exc:  # noqa: BLE001 — 어떤 오류든 정지·기록은 남긴다
            aborted = f"{type(exc).__name__}: {exc}"
            self.safe_stop(f"error: {aborted}")
        finally:
            meta = self.finish(aborted)
        return meta
