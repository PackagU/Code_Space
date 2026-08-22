#!/usr/bin/env python3
"""정적 장애물(정지 보행자) 배치 단일 소스 — ROS 의존 없음 (spawner + 오프라인 테스트 공용).

배치 원칙 (2026-08-20 리뷰 후속에서 확정):
- 반드시 **로봇 주행선 위**에 둔다. 경로 밖 장애물은 "정적 장애물 하 완주"를 검증하지 못한다
  (끊긴 세션의 (8.0, 0.9)/(3.3, 9.5) 는 F1 경로 x<=5.0 밖·동적 레인 겹침으로 폐기).
- 동적 보행자 레인(pedestrians.PEDESTRIANS 경로)과 겹치지 않는다 — 보행자는 set_entity_state
  텔레포트라 겹치면 정적 모델을 관통·밀어낸다. 간격 >= 두 반지름(0.22+0.22) + 여유 0.1.
- 우회 공간은 남긴다: 복도 폭 5m(F1 메인) / 3m(뒤 복도) 에서 장애물 한쪽에 로봇 실폭 0.54m
  + inflation 0.30x2 이상.

F1 메인 복도 중앙 (3.3, 0.0): 충전소(1.6,0)->택배 복도(5,0) 왕복선 위. 로봇은 남/북으로 우회.
  e2w 레인 y=0.6 / w2e 레인 y=-0.6 과 거리 0.6 (>0.54), ped_cross 경로 x 3.5~4.2 와 y=0 에서 0.55.
F2 뒤 복도 중앙 (2.5, 7.5): 엘베(0,0)->f2_corridor(2.5,12) 주행선 위. 서측 벽(x=1.0)과 1.28m 남음.
  bc_transit 레인 x=3.4 와 0.9, bc_cross 경로(2.9,6.5)->(3.6,10.5) 와 y=7.5 에서 0.575.
  (F1 월드에도 같은 뒤 복도가 있으나 F1 미션은 가지 않으므로 상시 존재해도 무간섭.)
"""
import math
import re

ROBOT_HALF_WIDTH = 0.27      # 실폭 0.5382 m
PED_RADIUS = 0.22            # pedestrian.sdf collision 반지름
LANE_CLEARANCE_MIN = PED_RADIUS * 2 + 0.10   # 동적 레인과 최소 간격 (관통 방지)

STATIC_OBSTACLES = [
    {"name": "static_ped_f1_corridor", "pos": (3.3, 0.0), "shirt": "0.5 0.5 0.5 1",
     "route": "f1_main_corridor"},
    {"name": "static_ped_f2_backcorr", "pos": (2.5, 7.5), "shirt": "0.6 0.6 0.3 1",
     "route": "f2_back_corridor"},
]

# 로봇 주행선(세그먼트) — smoke 의 goal 시퀀스에서 유도. 장애물은 이 중 하나에서 0.6m 이내여야 한다.
ROBOT_ROUTE_SEGMENTS = {
    "f1_main_corridor": ((1.6, 0.0), (5.0, 0.0)),
    "f2_back_corridor": ((2.5, 2.5), (2.5, 12.0)),
}
ON_ROUTE_MAX_DIST = 0.6      # 로봇 실반폭 0.27 + inflation 0.30 = 0.57 → 이 안이면 우회 강제


def point_segment_distance(p, a, b):
    (px, py), (ax, ay), (bx, by) = p, a, b
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def static_xml(template: str, name: str, shirt: str) -> str:
    """pedestrian.sdf 템플릿을 정적 장애물용으로 변환: 이름/상의색 치환 + static=true.
    static=true 면 물리 바디가 없어 로봇이 밀어도 안 움직이고(정적 장애물 의미 보존), LiDAR 는
    collision 을 그대로 본다."""
    xml = template.replace('name="pedestrian_1"', f'name="{name}"')
    xml = xml.replace("__SHIRT__", shirt)
    xml = xml.replace("<static>false</static>", "<static>true</static>")
    assert "<static>true</static>" in xml, "pedestrian.sdf must carry a <static> tag to override"
    # 템플릿의 모델 <pose>(4.0 0 0 …)는 spawn initial_pose 에 합성되어 정적 장애물이 +4m 어긋나
    # 스폰됐다(2026-08-21 G004 run2 world_state: (3.3,0)→(7.3,0), (2.5,7.5)→(6.5,7.5) — 경로 밖).
    # 동적 보행자는 매 틱 절대좌표 텔레포트라 무관했다. 모델 포즈를 0 으로 고정한다.
    xml = re.sub(r"<model name=\"[^\"]+\">(\s*)<static>true</static>(\s*)<pose>[^<]*</pose>",
                 lambda m: m.group(0).rsplit("<pose>", 1)[0] + "<pose>0 0 0 0 0 0</pose>", xml, count=1)
    assert re.search(r"<model name=\"[^\"]+\">\s*<static>true</static>\s*<pose>0 0 0 0 0 0</pose>", xml), (
        "static obstacle model pose must be zeroed (template pose offsets the spawn position)"
    )
    return xml
