#!/usr/bin/env python3
"""정적 장애물(정지 보행자) 스포너 — 스폰 완료 후 종료하는 원샷 노드.

최종 시나리오 요구인 '정적 장애물이 존재하는 상태의 반복 완주' 검증용.
배치/XML 변환 규칙은 static_obstacle_layout.py 가 단일 소스(오프라인 테스트가 같은 값을 검증):
- 로봇 주행선 위(F1 메인 복도 중앙 (3.3,0.0), F2 뒤 복도 중앙 (2.5,7.5)) → 우회 강제
- 동적 보행자 레인과 겹치지 않음(텔레포트 관통 방지), static=true(밀리지 않음, 물리 부하 0)

사용: python3 spawn_static_obstacles.py  (스폰 확인 후 exit 0, 실패 시 exit 1)
재실행(KEEP_RUNNING) 시 이미 존재하는 모델은 경고만 하고 통과한다.
"""
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity

sys.path.insert(0, str(Path(__file__).resolve().parent))
from static_obstacle_layout import STATIC_OBSTACLES, static_xml  # noqa: E402


class StaticObstacleSpawner(Node):
    def __init__(self):
        super().__init__("static_obstacle_spawner")
        self.declare_parameter("sdf_path", "")
        sdf_path = str(self.get_parameter("sdf_path").value)
        if not sdf_path:
            sdf_path = str(Path(__file__).resolve().parent / "pedestrian.sdf")
        self.sdf_template = open(sdf_path).read()
        self.cli = self.create_client(SpawnEntity, "/spawn_entity")

    def spawn_all(self, timeout_s=30.0):
        if not self.cli.wait_for_service(timeout_sec=timeout_s):
            self.get_logger().error("/spawn_entity unavailable")
            return False
        ok = True
        for spec in STATIC_OBSTACLES:
            req = SpawnEntity.Request()
            req.name = spec["name"]
            req.xml = static_xml(self.sdf_template, spec["name"], spec["shirt"])
            req.initial_pose.position.x = float(spec["pos"][0])
            req.initial_pose.position.y = float(spec["pos"][1])
            fut = self.cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_s)
            res = fut.result()
            if res is None:
                self.get_logger().error(f"spawn {spec['name']}: no response")
                ok = False
            elif not res.success and "exist" not in (res.status_message or ""):
                self.get_logger().error(f"spawn {spec['name']}: {res.status_message}")
                ok = False
            else:
                self.get_logger().info(
                    f"static obstacle {spec['name']} at {spec['pos']} ({spec['route']})")
        return ok


def main():
    rclpy.init()
    node = StaticObstacleSpawner()
    try:
        ok = node.spawn_all()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(f"STATIC OBSTACLES {'OK' if ok else 'FAIL'}: {[s['name'] for s in STATIC_OBSTACLES]}")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
