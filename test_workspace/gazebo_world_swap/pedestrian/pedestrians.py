#!/usr/bin/env python3
"""다중 보행자(동적 장애물) 컨트롤러 — 현실적 waypoint 이동.

한 노드가 여러 보행자를 스폰하고 각자 waypoint 경로(폐루프)를 따라 이동시킨다.
- 복도를 길이방향으로 걷는 사람(정면/추월 마주침) + 교차하는 사람을 혼합 -> 사람답고,
  좁은 복도를 가로로만 쓸어 막던 문제(통행 불가)를 해소한다.
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
from gazebo_msgs.srv import SetEntityState, SpawnEntity

# 복도 내부 안전 영역 (5m 폭, 벽 여유 포함). 보행자별 xr/yr 가 없으면 이 기본값으로 clamp.
# x<1.5(엘리베이터 문)와 깊은 남측(택배 하강 통로)만 비워 임무/복귀를 보장한다.
X_RANGE = (1.5, 13.0)
Y_RANGE = (-2.2, 2.2)

# 뒤 복도(방 복도) 안전 영역. 벽 x=BC_WEST(1.0)/BC_EAST(4.0), y=CORRIDOR_HALF(2.5)~14.
# 벽 두께·보행자 반지름(0.22) 여유로 x[1.3,3.7], y[2.8,13.6] 안으로 clamp.
# 엘베 분기(y<2.8)는 비워 로봇 진입을 막지 않는다. F2 뒤 복도 보행자 전용.
BC_XRANGE = (1.3, 3.7)
BC_YRANGE = (2.8, 13.6)

# 보행자 시나리오 (5m 메인 복도 y in [-2.44,2.44], x ~0..15). 속도는 사람 보행(0.3~0.5 m/s).
#  - ped_cross / ped_diag 가 로봇 주행선(charge 1.6 -> corridor 5, y~0)을 가로질러
#    로봇이 멈추거나/우회하게 만든다(동적 회피 관찰). 이동 중이라 틈이 생겨 결국 통과.
#  - 나머지는 동측에서 다양한 경로(직선/L자/배회)로 돌아다녀 사람 붐비는 느낌.
#  shirt: 보행자별 상의 색(rgba) — SDF __SHIRT__ 치환.
PEDESTRIANS = [
    # 1) 로봇 경로(x≈3.5) 대각 횡단
    {"name": "ped_cross", "speed": 0.45, "loop": True, "shirt": "0.85 0.2 0.2 1",
     "waypoints": [(3.2, -2.0), (4.2, 2.0)]},
    # 2) corridor goal(5,0) 부근 반대 대각 횡단 (위상차)
    {"name": "ped_diag", "speed": 0.35, "loop": True, "shirt": "0.2 0.55 0.85 1",
     "waypoints": [(5.2, 2.0), (4.0, -1.6), (5.2, -1.6), (4.0, 2.0)]},
    # 3) 동측 복도를 길이방향으로 왕복하는 사람 (사각 순환)
    {"name": "ped_walk", "speed": 0.5, "loop": True, "shirt": "0.2 0.7 0.3 1",
     "waypoints": [(7.0, 1.6), (12.5, 1.6), (12.5, -1.6), (7.0, -1.6)]},
    # 4) 동측에서 넓게 배회 (다각형 경로)
    {"name": "ped_wander", "speed": 0.4, "loop": True, "shirt": "0.8 0.6 0.15 1",
     "waypoints": [(8.0, 0.0), (10.5, 1.8), (12.5, -0.3), (9.5, -1.8), (7.5, -0.5)]},
    # 5) 동측 한 지점을 가로지르는 사람
    {"name": "ped_cross_e", "speed": 0.4, "loop": True, "shirt": "0.6 0.3 0.7 1",
     "waypoints": [(10.0, -1.8), (10.0, 1.8)]},
    # 6) F2 뒤 복도 동측(방 앞)을 대각으로 오가는 사람. 로봇 주행선(x≈2.1, 중심선)을
    #    상시 점유하지 않도록 동측(x>=2.9)에 둬 E2E 통과를 보장하면서 보행자 존재감만 준다.
    {"name": "ped_bc_cross", "speed": 0.5, "loop": True, "shirt": "0.9 0.5 0.1 1",
     "xr": BC_XRANGE, "yr": BC_YRANGE,
     "waypoints": [(2.9, 6.5), (3.6, 10.5)]},
    # 7) F2 뒤 복도 동측(방 앞)을 길이방향으로 왕복 — 중심선(x=2.5) 비켜선 ambiance.
    {"name": "ped_bc_walk", "speed": 0.45, "loop": True, "shirt": "0.3 0.5 0.9 1",
     "xr": BC_XRANGE, "yr": BC_YRANGE,
     "waypoints": [(3.4, 3.5), (3.4, 12.5)]},
]


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
        self.get_logger().info("waiting for /spawn_entity, /gazebo/set_entity_state...")
        self.spawn_cli.wait_for_service()
        self.state_cli.wait_for_service()

        self.peds = []
        for spec in PEDESTRIANS:
            xr = spec.get("xr", X_RANGE)
            yr = spec.get("yr", Y_RANGE)
            wps = [(clamp(x, *xr), clamp(y, *yr)) for (x, y) in spec["waypoints"]]
            segs = self._segments(wps, spec.get("loop", True))
            total = sum(s[2] for s in segs)
            self.peds.append({
                "name": spec["name"], "speed": float(spec["speed"]),
                "wps": wps, "segs": segs, "total": total,
                "loop": spec.get("loop", True), "s": 0.0,
                "xr": xr, "yr": yr,
            })
            self._spawn(spec["name"], wps[0], spec.get("shirt", "0.2 0.4 0.8 1"))

        # 위치 갱신 주기 10Hz(dt=0.1). 0.5 m/s 보행 -> 5cm/step 으로 충분히 매끄럽고,
        # GUI+다수 보행자 동시 실행 시 set_entity_state 호출 부하를 20Hz 대비 절반으로 낮춰
        # 좁은 엘베 문 통과(제어 정밀 구간)의 제어 주기 깨짐을 완화한다.
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self._tick)
        self.get_logger().info(f"driving {len(self.peds)} pedestrians")

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
        # 폐루프 호 길이 s 위치의 (x,y,yaw)
        s = ped["s"] % ped["total"] if ped["total"] > 0 else 0.0
        for (a, b, d) in ped["segs"]:
            if s <= d:
                t = s / d
                x = a[0] + (b[0] - a[0]) * t
                y = a[1] + (b[1] - a[1]) * t
                yaw = math.atan2(b[1] - a[1], b[0] - a[0])
                return x, y, yaw
            s -= d
        a, b, _ = ped["segs"][-1]
        return b[0], b[1], math.atan2(b[1] - a[1], b[0] - a[0])

    def _tick(self):
        for ped in self.peds:
            if ped["total"] <= 0:
                continue
            ped["s"] += ped["speed"] * self.dt
            x, y, yaw = self._pose_at(ped)
            x = clamp(x, *ped["xr"])
            y = clamp(y, *ped["yr"])
            req = SetEntityState.Request()
            req.state.name = ped["name"]
            req.state.pose.position.x = x
            req.state.pose.position.y = y
            req.state.pose.position.z = 0.0
            req.state.pose.orientation.z = math.sin(yaw / 2.0)
            req.state.pose.orientation.w = math.cos(yaw / 2.0)
            req.state.reference_frame = "world"
            self.state_cli.call_async(req)


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
