#!/usr/bin/env python3
"""다중 보행자(동적 장애물) 컨트롤러 — 실제 통행 같은 transit 이동.

한 노드가 여러 보행자를 스폰하고, 각자 route 를 "한 번 지나가고 사라지는" 방식으로
움직인다(사람은 복도를 무한 왕복하지 않는다). 퇴장 후 rest_s 부재 -> 반대 방향 재등장.
- 복도를 관통하는 사람 + 방으로 들어가는 사람 + 가로질러 건너는 사람의 혼합.
- 이동은 /gazebo/set_entity_state 절대좌표 위치제어. 속도제어(planar_move)와 달리
  로봇이 밀어도 안 밀리고, waypoint 가 맵 안이라 벽을 통과해 이탈하지 않는다.
- 진행 방향으로 yaw 를 돌려 사람처럼 보이게 한다.
- collision 원통(높이 1.8m)이 LiDAR 평면(z=0.75)을 가로지르므로 로봇이 /scan 으로 감지.

경로/속도는 PEDESTRIANS 에 정의한다(현재 맵 기준). 맵을 바꾸면 여기 좌표를 갱신.
clamp 로 복도 안전 영역(X_RANGE/Y_RANGE)을 절대 벗어나지 않게 한다.
"""
import math

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState, SpawnEntity
from geometry_msgs.msg import PoseWithCovarianceStamped

# 복도 내부 안전 영역 (5m 폭, 벽 여유 포함). 보행자별 xr/yr 가 없으면 이 기본값으로 clamp.
# x<1.5(엘리베이터 문)와 깊은 남측(택배 하강 통로)만 비워 임무/복귀를 보장한다.
X_RANGE = (1.5, 13.0)
Y_RANGE = (-2.2, 2.2)

# 뒤 복도(방 복도) 안전 영역. 벽 x=BC_WEST(1.0)/BC_EAST(4.0), y=CORRIDOR_HALF(2.5)~14.
# 벽 두께·보행자 반지름(0.22) 여유로 x[1.3,3.7], y[2.8,13.6] 안으로 clamp.
# 엘베 분기(y<2.8)는 비워 로봇 진입을 막지 않는다. F2 뒤 복도 보행자 전용.
BC_XRANGE = (1.3, 3.7)
BC_YRANGE = (2.8, 13.6)

# 보행자 시나리오 — 실제 사람처럼 "지나가는(transit)" 모델. 사람은 복도를 무한 왕복하지
# 않는다: 한 방향으로 걸어가 목적지(방/모퉁이)로 사라지고, 시간이 지나면 다른 사람이
# 지나간다. 각 보행자는 route 를 한 번 걷고 퇴장(PARK 로 텔레포트) -> rest_s 만큼 부재 ->
# 반대 방향으로 재등장(반대편에서 오는 다른 사람 효과)을 반복한다.
#  - delay_s: 첫 등장 지연(위상차). 전원이 동시에 복도를 점유하지 않게 어긋나게 둔다.
#  - rest_s: 퇴장 후 부재 시간. 복도가 대부분의 시간 비어 있는 실제 통행 밀도를 만든다.
#  - 난수 없이 고정 위상 -> smoke 재현성 유지.
#  shirt: 보행자별 상의 색(rgba) — SDF __SHIRT__ 치환.
PEDESTRIANS = [
    # 1) 메인 복도를 동->서로 관통하는 사람 (중심선 북측 통행).
    #    서쪽 종점은 x=2.6 — 엘베 출구 깔때기(문 0.8~약 2.5)에 들어가면 로봇이
    #    문턱에서 collision ahead 로 갇힌다(2026-07-03 재현). x<2.6 접근 금지.
    {"name": "ped_main_e2w", "speed": 0.5, "shirt": "0.2 0.55 0.85 1",
     "delay_s": 6.0, "rest_s": 22.0,
     "waypoints": [(12.8, 0.6), (2.6, 0.6)]},
    # 2) 메인 복도를 서->동으로 관통하는 사람 (중심선 남측 통행, 반대 흐름).
    #    서쪽 시작점도 같은 이유로 x=2.6 밖.
    {"name": "ped_main_w2e", "speed": 0.45, "shirt": "0.2 0.7 0.3 1",
     "delay_s": 24.0, "rest_s": 26.0,
     "waypoints": [(2.6, -0.6), (12.8, -0.6)]},
    # 3) 방에서 나와 복도를 가로질러 반대편으로 가는 사람 — 로봇 주행선(y~0) 횡단.
    #    동적 회피 검증 요소 유지(지나가는 동안만 점유, 이동 중이라 틈이 생겨 결국 통과).
    {"name": "ped_cross", "speed": 0.45, "shirt": "0.85 0.2 0.2 1",
     "delay_s": 14.0, "rest_s": 18.0,
     "waypoints": [(3.5, -2.0), (4.2, 2.0)]},
    # 4) 뒤 복도(방 복도)를 끝까지 걸어가 자기 방으로 들어가는 사람 (F2/F3 구간).
    #    동측(x=3.4)으로 걸어 로봇 주행선(중심선 x=2.5)을 상시 점유하지 않는다.
    {"name": "ped_bc_transit", "speed": 0.5, "shirt": "0.3 0.5 0.9 1",
     "delay_s": 10.0, "rest_s": 28.0,
     "xr": BC_XRANGE, "yr": BC_YRANGE,
     "waypoints": [(3.4, 3.0), (3.4, 13.4)]},
    # 5) 뒤 복도에서 비스듬히 건너편 방으로 가는 사람.
    {"name": "ped_bc_cross", "speed": 0.5, "shirt": "0.9 0.5 0.1 1",
     "delay_s": 34.0, "rest_s": 32.0,
     "xr": BC_XRANGE, "yr": BC_YRANGE,
     "waypoints": [(2.9, 6.5), (3.6, 10.5)]},
]

# 퇴장한 보행자 대기 위치(건물 밖, LiDAR 사거리 밖). 인덱스로 서로 겹치지 않게 배치.
PARK_BASE = (30.0, 30.0)

# 로봇 근접 일시정지 거리(m): 로봇 footprint 최대 반경(노즈 0.40) + 보행자 반지름 0.22 + 여유.
PAUSE_DIST = 0.7
# 양보(yield): 이 시간 이상 막히면 RETREAT_STEP 만큼 경로를 되돌아가고, MAX_RETREATS 초과나
# 물러나도 PAUSE_DIST 이내면 이번 통행을 포기(퇴장). 교착(로봇↔보행자 상호 대기) 해소용.
YIELD_PAUSE_S = 4.0
RETREAT_STEP = 0.8
MAX_RETREATS = 3


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Pedestrians(Node):
    def __init__(self):
        super().__init__("pedestrians")
        self.declare_parameter("sdf_path", "")
        sdf_path = str(self.get_parameter("sdf_path").value)
        if not sdf_path:
            from pathlib import Path
            sdf_path = str(Path(__file__).resolve().parent / "pedestrian.sdf")
        self.sdf_template = open(sdf_path).read()

        self.spawn_cli = self.create_client(SpawnEntity, "/spawn_entity")
        self.state_cli = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        # set_entity_state 실패 누적 감지: call_async 결과가 조용히 실패하면 보행자가
        # 통로 한가운데 프리즈된 채 정지 장애물이 된다(2026-08-17 run_05 유력 가설, §1.25).
        # 응답이 아예 안 오는(콜백 미발화) 모드도 잡기 위해 마지막 성공 시각을 추적한다.
        self._set_fail = 0
        self._last_ok = self.get_clock().now()
        self._last_attempt = self._last_ok   # 마지막 set_entity_state 호출 시각(프리즈 감지 오탐 방지)
        # 로봇 근접 시 보행자 일시정지용 로봇 위치 추적.
        # set_entity_state 텔레포트가 로봇과 겹치면 Gazebo 가 관통을 충격량으로 해소해
        # 로봇이 뒤집힌 사례 실측 (2026-08-18 분산 run2: 로봇 참값 z=0.23m — §1.25g).
        # 1순위: /gazebo/model_states 참값(50Hz, gazebo_ros_state) — 액터 안전장치는
        #   로봇 능력 검증이 아니므로 참값이 옳다(belief 드리프트 0.716~0.974m 실측이
        #   근접 임계 0.7m 를 넘어 belief 기반 판정은 무발화할 수 있음).
        # 2순위(참값 부재 시): /amcl_pose belief 폴백.
        # 회귀 주의(a6a5115): 이 아래 초기화 본문은 반드시 __init__ 안에 있어야 한다.
        #   콜백으로 이동하면 amcl 수신마다 재초기화(타이머 누수·전면 리셋)된다 —
        #   test_smoke_scripts_contract.py 의 AST 가드가 이를 강제한다.
        self.declare_parameter("robot_model_name", "elevator_robot_f1")
        self._robot_model_name = str(self.get_parameter("robot_model_name").value)
        self._robot_xy = None       # 근접 판정에 쓰는 최종 로봇 (x, y)
        self._robot_xy_true = None  # 참값 캐시
        self.create_subscription(ModelStates, "/gazebo/model_states", self._on_model_states, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, 10)

        self.get_logger().info("waiting for /spawn_entity, /gazebo/set_entity_state...")
        self.spawn_cli.wait_for_service()
        self.state_cli.wait_for_service()

        self.peds = []
        for i, spec in enumerate(PEDESTRIANS):
            xr = spec.get("xr", X_RANGE)
            yr = spec.get("yr", Y_RANGE)
            wps = [(clamp(x, *xr), clamp(y, *yr)) for (x, y) in spec["waypoints"]]
            segs_fwd = self._segments(wps, False)
            segs_rev = self._segments(list(reversed(wps)), False)
            total = sum(s[2] for s in segs_fwd)
            park = (PARK_BASE[0] + 2.0 * i, PARK_BASE[1])
            self.peds.append({
                "name": spec["name"], "speed": float(spec["speed"]),
                "segs_fwd": segs_fwd, "segs_rev": segs_rev, "total": total,
                "park": park, "forward": True, "s": 0.0,
                "state": "resting", "wait": float(spec.get("delay_s", 0.0)),
                "rest": float(spec.get("rest_s", 20.0)),
                "xr": xr, "yr": yr,
                "paused_s": 0.0, "retreats": 0,
            })
            # 처음에는 건물 밖(PARK)에서 스폰 -> delay_s 후 첫 등장.
            self._spawn(spec["name"], park, spec.get("shirt", "0.2 0.4 0.8 1"))

        # 위치 갱신 주기 5Hz(dt=0.2). 0.5 m/s 보행 -> 10cm/step.
        # 이력: 20Hz -> 10Hz -> 5Hz. 7명 x 10Hz(초당 70회 set_entity_state)에서
        # nav2 control_loop_missed_rate 1 -> 20, tf_extrapolation 증가로 엘베 진입 정차가
        # 벽에 붙고(중앙 (0.2,0.0) 대신 (0.0,0.4)) 탈출 불능 ABORTED 재현(2026-07-03).
        # transit 모델은 부재(resting) 중 호출이 없어 평균 부하가 더 낮다.
        self.dt = 0.2
        self.timer = self.create_timer(self.dt, self._tick)
        self.get_logger().info(f"driving {len(self.peds)} pedestrians")

    def _on_model_states(self, msg):
        # Gazebo 참값 (1순위). 콜백은 위치 갱신만 한다 — 초기화 금지(a6a5115 회귀).
        try:
            i = msg.name.index(self._robot_model_name)
        except ValueError:
            return
        p = msg.pose[i].position
        self._robot_xy_true = (p.x, p.y)
        self._robot_xy = self._robot_xy_true

    def _on_amcl(self, msg):
        # belief 폴백 (참값 미수신 시에만 사용). 콜백은 위치 갱신만 한다.
        if self._robot_xy_true is not None:
            return
        p = msg.pose.pose.position
        self._robot_xy = (p.x, p.y)

    def _segments(self, wps, loop):
        pts = wps + ([wps[0]] if loop and len(wps) > 1 else [])
        segs = []
        for a, b in zip(pts, pts[1:]):
            d = math.hypot(b[0] - a[0], b[1] - a[1])
            if d > 1e-6:
                segs.append((a, b, d))
        return segs

    def _spawn(self, name, pos, shirt="0.2 0.4 0.8 1"):
        xml = self.sdf_template.replace('name="pedestrian_1"', f'name="{name}"')
        xml = xml.replace("__SHIRT__", shirt)
        req = SpawnEntity.Request()
        req.name = name
        req.xml = xml
        req.initial_pose.position.x = float(pos[0])
        req.initial_pose.position.y = float(pos[1])
        req.initial_pose.position.z = 0.0
        self.spawn_cli.call_async(req)
        self.get_logger().info(f"spawn {name} at {pos}")

    def _pose_at(self, ped):
        return self._pose_at_s(ped, ped["s"])

    def _pose_at_s(self, ped, s):
        # 현재 진행 방향 route 의 호 길이 s 위치의 (x,y,yaw)
        segs = ped["segs_fwd"] if ped["forward"] else ped["segs_rev"]
        for (a, b, d) in segs:
            if s <= d:
                t = s / d
                x = a[0] + (b[0] - a[0]) * t
                y = a[1] + (b[1] - a[1]) * t
                yaw = math.atan2(b[1] - a[1], b[0] - a[0])
                return x, y, yaw
            s -= d
        a, b, _ = segs[-1]
        return b[0], b[1], math.atan2(b[1] - a[1], b[0] - a[0])

    def _set_state(self, name, x, y, yaw):
        req = SetEntityState.Request()
        req.state.name = name
        req.state.pose.position.x = float(x)
        req.state.pose.position.y = float(y)
        req.state.pose.position.z = 0.0
        req.state.pose.orientation.z = math.sin(yaw / 2.0)
        req.state.pose.orientation.w = math.cos(yaw / 2.0)
        req.state.reference_frame = "world"
        self._last_attempt = self.get_clock().now()
        fut = self.state_cli.call_async(req)
        fut.add_done_callback(lambda f, n=name: self._check_set_result(f, n))

    def _check_set_result(self, fut, name):
        try:
            res = fut.result()
            ok = bool(res and res.success)
        except Exception:
            ok = False
        if ok:
            self._set_fail = 0
            self._last_ok = self.get_clock().now()
            return
        self._set_fail += 1
        if self._set_fail % 25 == 1:
            self.get_logger().error(
                f"set_entity_state 실패 누적 {self._set_fail}회 ({name}) — "
                "보행자 위치 갱신 정체(프리즈 장애물화) 위험"
            )

    def _tick(self):
        for ped in self.peds:
            if ped["total"] <= 0:
                continue
            if ped["state"] == "resting":
                # 부재 중(방/모퉁이 너머). 대기 시간에는 호출도 없어 부하가 준다.
                ped["wait"] -= self.dt
                if ped["wait"] <= 0.0:
                    ped["state"] = "walking"
                    ped["s"] = 0.0
                    ped["paused_s"] = 0.0
                    ped["retreats"] = 0
                continue
            # 로봇 근접 시 일시정지: 사람이 로봇을 뚫고 걷지 않도록 0.7m 이내 위치
            # 갱신은 보류한다(로봇 통과 후 재개). 관통-충격량 캐터펄트 원천 차단(§1.25g).
            if self._robot_xy is not None:
                nx, ny, _ = self._pose_at_s(ped, ped["s"] + ped["speed"] * self.dt)
                if math.hypot(nx - self._robot_xy[0], ny - self._robot_xy[1]) < PAUSE_DIST:
                    # 양보(yield, 2026-08-21 캠페인 run_05 실측): 보행자가 로봇 진행 경로 위에 멈춰
                    # 서고 로봇은 그 보행자 때문에 collision-ahead 로 멈추는 교착(8분 정지) 발생.
                    # 사람은 마주 선 로봇 앞에서 영원히 기다리지 않고 물러선다 — YIELD_PAUSE_S 이상
                    # 막히면 경로를 따라 RETREAT_STEP 뒤로 물러나고(로봇에서 멀어지는 방향),
                    # 물러나도 가까우면(또는 MAX_RETREATS 초과) 이번 통행을 포기하고 퇴장한다.
                    ped["paused_s"] += self.dt
                    if ped["paused_s"] >= YIELD_PAUSE_S:
                        ped["paused_s"] = 0.0
                        ped["retreats"] += 1
                        back_s = max(0.0, ped["s"] - RETREAT_STEP)
                        bx, by, byaw = self._pose_at_s(ped, back_s)
                        far_enough = math.hypot(bx - self._robot_xy[0], by - self._robot_xy[1]) >= PAUSE_DIST
                        if ped["retreats"] <= MAX_RETREATS and far_enough and back_s < ped["s"]:
                            ped["s"] = back_s
                            self.get_logger().info(
                                f"yield: {ped['name']} steps back {RETREAT_STEP}m (retreat {ped['retreats']}/{MAX_RETREATS})")
                            self._set_state(ped["name"], clamp(bx, *ped["xr"]), clamp(by, *ped["yr"]), byaw)
                        else:
                            self.get_logger().info(f"yield: {ped['name']} gives way and leaves (park)")
                            ped["state"] = "resting"
                            ped["wait"] = ped["rest"]
                            ped["forward"] = not ped["forward"]
                            self._set_state(ped["name"], ped["park"][0], ped["park"][1], 0.0)
                    continue
            ped["paused_s"] = 0.0
            ped["s"] += ped["speed"] * self.dt
            if ped["s"] >= ped["total"]:
                # 목적지 도착 -> 퇴장. 다음 등장은 반대 방향(반대편에서 오는 사람 효과).
                ped["state"] = "resting"
                ped["wait"] = ped["rest"]
                ped["forward"] = not ped["forward"]
                self._set_state(ped["name"], ped["park"][0], ped["park"][1], 0.0)
                continue
            x, y, yaw = self._pose_at(ped)
            x = clamp(x, *ped["xr"])
            y = clamp(y, *ped["yr"])
            self._set_state(ped["name"], x, y, yaw)
        # 무응답 프리즈 감지: walking 보행자가 있는데 5초 이상 성공 응답이 없으면
        # Gazebo 쪽 정체 — 보행자가 마지막 위치에 정지 장애물로 굳는다(§1.25).
        # 단, 전원 근접 일시정지 중이면 호출 자체가 없어 성공도 없다 — 마지막 호출이 마지막 성공보다
        # 뒤일 때(응답 없는 호출이 있을 때)만 프리즈로 본다(2026-08-21 run_05 오탐 정정).
        if any(p["state"] == "walking" for p in self.peds) \
                and self._last_attempt.nanoseconds > self._last_ok.nanoseconds:
            stale_s = (self.get_clock().now() - self._last_ok).nanoseconds / 1e9
            if stale_s > 5.0:
                self.get_logger().error(
                    f"set_entity_state 성공 응답 {stale_s:.1f}s 부재 — 보행자 프리즈 의심(§1.25)",
                    throttle_duration_sec=5.0,
                )


def main():
    rclpy.init()
    node = Pedestrians()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
