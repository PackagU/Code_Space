#!/usr/bin/env python3
"""pedestrians.py 행위 회귀 테스트 (rclpy 스텁, ROS 불필요).

2026-08-18 커밋 a6a5115 가 __init__ 본문(스폰·타이머 생성)을 /amcl_pose 콜백 안으로
밀어 넣어 amcl 수신마다 보행자 전원 리셋 + 타이머 누수가 생겼다(2026-08-20 적대 리뷰
findings #1: 증거런 2개에서 재초기화 274/329회). 문자열 contract 로는 못 잡는 결함이라
스텁으로 노드를 실제 생성해 행위를 고정한다.

검증 항목:
  (1) 생성 직후 타이머 정확히 1개, 스폰 호출 = PEDESTRIANS 수, peds 존재
  (2) /amcl_pose · /gazebo/model_states 콜백을 각각 20회 주입해도 타이머/스폰 수 불변,
      걷던 보행자 상태가 리셋되지 않는다
  (3) 근접 판정용 로봇 위치는 참값(model_states)이 belief(amcl)보다 우선한다
  (4) 근접 0.7m 이내면 위치 갱신을 보류한다(텔레포트 관통 차단)
사용: python3 test_pedestrians_stub.py
"""
from __future__ import annotations

import math
import sys
import types
from pathlib import Path


# ---------------- rclpy / msgs 스텁 ----------------
class _Time:
    def __init__(self, ns=0):
        self.nanoseconds = ns

    def __sub__(self, other):
        return _Time(self.nanoseconds - other.nanoseconds)


class _Clock:
    def now(self):
        return _Time(0)


class _Logger:
    def __init__(self):
        self.msgs = []

    def info(self, m, **kw):
        self.msgs.append(("info", m))

    def error(self, m, **kw):
        self.msgs.append(("error", m))


class _Future:
    def add_done_callback(self, cb):
        pass


class _Client:
    def __init__(self, tracker, name):
        self.tracker = tracker
        self.name = name

    def wait_for_service(self, timeout_sec=None):
        return True

    def call_async(self, req):
        self.tracker.setdefault(self.name, []).append(req)
        return _Future()


class _Param:
    def __init__(self, v):
        self.value = v


class Node:
    def __init__(self, name):
        self._tracker = {}
        self._timers = []
        self._subs = []
        self._params = {}
        self._logger = _Logger()

    def declare_parameter(self, n, d=None):
        self._params[n] = d

    def get_parameter(self, n):
        return _Param(self._params.get(n, ""))

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return _Clock()

    def create_client(self, srv, name):
        return _Client(self._tracker, name)

    def create_subscription(self, mtype, topic, cb, qos):
        self._subs.append((topic, cb))
        return object()

    def create_timer(self, dt, cb):
        self._timers.append((dt, cb))
        return object()


def _install_stubs():
    rclpy = types.ModuleType("rclpy")
    rclpy.node = types.ModuleType("rclpy.node")
    rclpy.node.Node = Node
    rclpy.init = lambda: None
    rclpy.spin = lambda n: None
    rclpy.ok = lambda: True
    rclpy.shutdown = lambda: None
    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.node"] = rclpy.node

    class _Req:
        def __init__(self):
            self.name = ""
            self.xml = ""
            self.initial_pose = types.SimpleNamespace(
                position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0))
            self.state = types.SimpleNamespace(
                name="", reference_frame="",
                pose=types.SimpleNamespace(
                    position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                    orientation=types.SimpleNamespace(z=0.0, w=1.0)))

    class SpawnEntity:
        Request = _Req

    class SetEntityState:
        Request = _Req

    class ModelStates:
        pass

    gm = types.ModuleType("gazebo_msgs")
    gms = types.ModuleType("gazebo_msgs.srv")
    gmm = types.ModuleType("gazebo_msgs.msg")
    gms.SpawnEntity = SpawnEntity
    gms.SetEntityState = SetEntityState
    gmm.ModelStates = ModelStates
    gm.srv = gms
    gm.msg = gmm
    sys.modules["gazebo_msgs"] = gm
    sys.modules["gazebo_msgs.srv"] = gms
    sys.modules["gazebo_msgs.msg"] = gmm

    geo = types.ModuleType("geometry_msgs")
    geom = types.ModuleType("geometry_msgs.msg")

    class PoseWithCovarianceStamped:
        pass

    geom.PoseWithCovarianceStamped = PoseWithCovarianceStamped
    geo.msg = geom
    sys.modules["geometry_msgs"] = geo
    sys.modules["geometry_msgs.msg"] = geom


def _amcl_msg(x, y):
    return types.SimpleNamespace(pose=types.SimpleNamespace(
        pose=types.SimpleNamespace(position=types.SimpleNamespace(x=x, y=y))))


def _model_states_msg(names_xy):
    names = [n for n, _ in names_xy]
    poses = [types.SimpleNamespace(position=types.SimpleNamespace(x=x, y=y))
             for _, (x, y) in names_xy]
    return types.SimpleNamespace(name=names, pose=poses)


def main():
    _install_stubs()
    ped_dir = Path(__file__).resolve().parents[1] / "pedestrian"
    sys.path.insert(0, str(ped_dir))
    import pedestrians  # noqa: E402

    node = pedestrians.Pedestrians()
    n_ped = len(pedestrians.PEDESTRIANS)
    spawn = node._tracker.get("/spawn_entity", [])

    # (1) 생성 직후
    assert len(node._timers) == 1, f"timers after init must be 1, got {len(node._timers)}"
    assert len(spawn) == n_ped, f"spawn calls must be {n_ped}, got {len(spawn)}"
    assert hasattr(node, "peds") and len(node.peds) == n_ped
    subs = dict(node._subs)
    assert "/amcl_pose" in subs and "/gazebo/model_states" in subs, subs.keys()

    # 걷기 시작시켜 둔다 (delay_s 경과 상당 틱)
    tick = node._timers[0][1]
    node._robot_xy = None
    for _ in range(40):  # 8s 상당
        tick()
    walking = [p for p in node.peds if p["state"] == "walking"]
    assert walking, "some pedestrian should be walking after 8s of ticks"
    s_before = {p["name"]: p["s"] for p in walking}

    # (2) 콜백 20회씩 주입 — 재초기화 없음
    for i in range(20):
        subs["/amcl_pose"](_amcl_msg(1.6 + 0.25 * i, 0.0))
        subs["/gazebo/model_states"](_model_states_msg(
            [("kku_f1_building", (0.0, 0.0)), ("elevator_robot_f1", (2.0 + 0.1 * i, 0.05))]))
    assert len(node._timers) == 1, f"timer leak: {len(node._timers)} timers after callbacks"
    assert len(node._tracker.get("/spawn_entity", [])) == n_ped, "re-spawn on callback"
    for p in walking:
        assert p["state"] == "walking", f"{p['name']} reset to {p['state']} by callback"
        assert p["s"] == s_before[p["name"]], "walk progress reset by callback"

    # (3) 참값 우선: model_states 마지막 값 (3.9, 0.05) 가 amcl (6.35, 0) 보다 우선
    assert node._robot_xy is not None
    assert abs(node._robot_xy[0] - 3.9) < 1e-9 and abs(node._robot_xy[1] - 0.05) < 1e-9, node._robot_xy
    # 참값 부재 시에는 belief 폴백
    node2 = pedestrians.Pedestrians()
    dict(node2._subs)["/amcl_pose"](_amcl_msg(7.0, -0.5))
    assert node2._robot_xy == (7.0, -0.5), "amcl fallback must set robot_xy when no truth yet"

    # (4) 근접 보류: 로봇을 보행자 바로 앞(0.3m)에 두면 s 가 증가하지 않는다
    p = walking[0]
    nx, ny, _ = node._pose_at_s(p, p["s"] + p["speed"] * node.dt)
    node._robot_xy = (nx + 0.3, ny)
    s0 = p["s"]
    tick()
    assert p["s"] == s0, "pedestrian must pause within 0.7m of robot (truth)"
    node._robot_xy = (nx + 5.0, ny)
    tick()
    assert p["s"] > s0, "pedestrian must resume when robot is far"

    print("PASS pedestrians stub behavior (init once, no callback re-init, truth-first proximity)")


if __name__ == "__main__":
    main()
