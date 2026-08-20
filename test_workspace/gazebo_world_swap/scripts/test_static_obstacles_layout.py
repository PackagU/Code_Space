#!/usr/bin/env python3
"""정적 장애물 배치 회귀 테스트 (ROS 불필요).

검증:
  (1) 각 정적 장애물은 로봇 주행선(ROBOT_ROUTE_SEGMENTS)에서 ON_ROUTE_MAX_DIST 이내 — 우회 강제
      (경로 밖 배치는 통과율만 부풀린다: 2026-08-20 리뷰 후속 판정)
  (2) 동적 보행자(pedestrians.PEDESTRIANS) 모든 경로 세그먼트와 LANE_CLEARANCE_MIN 이상 떨어짐
      (텔레포트 관통·밀림 방지). 보행자 클램프 영역(xr/yr)을 적용한 실제 waypoint 로 계산.
  (3) static_xml 은 static=true 로 바꾸고 이름/상의색을 치환한다 (pedestrian.sdf 실파일 기준)
사용: python3 test_static_obstacles_layout.py
"""
from __future__ import annotations

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PED_DIR = HERE.parent / "pedestrian"
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(PED_DIR))

from test_pedestrians_stub import _install_stubs  # noqa: E402

_install_stubs()
import pedestrians  # noqa: E402
import static_obstacle_layout as L  # noqa: E402


def main():
    # (1) 경로 위
    for ob in L.STATIC_OBSTACLES:
        a, b = L.ROBOT_ROUTE_SEGMENTS[ob["route"]]
        d = L.point_segment_distance(ob["pos"], a, b)
        assert d <= L.ON_ROUTE_MAX_DIST, (
            f"{ob['name']} at {ob['pos']} is {d:.2f}m off route {ob['route']} "
            f"(> {L.ON_ROUTE_MAX_DIST}) — would not force a detour")

    # (2) 동적 레인과 간격
    for ob in L.STATIC_OBSTACLES:
        for spec in pedestrians.PEDESTRIANS:
            xr = spec.get("xr", pedestrians.X_RANGE)
            yr = spec.get("yr", pedestrians.Y_RANGE)
            wps = [(pedestrians.clamp(x, *xr), pedestrians.clamp(y, *yr)) for (x, y) in spec["waypoints"]]
            for p, q in zip(wps, wps[1:]):
                d = L.point_segment_distance(ob["pos"], p, q)
                assert d >= L.LANE_CLEARANCE_MIN, (
                    f"{ob['name']} at {ob['pos']} is {d:.2f}m from dynamic lane {spec['name']} "
                    f"segment {p}->{q} (< {L.LANE_CLEARANCE_MIN}) — teleport would punch through")

    # (3) XML 변환
    template = (PED_DIR / "pedestrian.sdf").read_text(encoding="utf-8")
    xml = L.static_xml(template, "static_x", "0.1 0.2 0.3 1")
    assert "<static>true</static>" in xml and "<static>false</static>" not in xml
    assert 'name="static_x"' in xml and "__SHIRT__" not in xml
    # 템플릿 모델 pose(4.0 0 0) 가 spawn 위치에 합성돼 +4m 어긋나던 결함(G004 run2) 회귀 차단
    import re
    m = re.search(r'<model name="static_x">\s*<static>true</static>\s*<pose>([^<]*)</pose>', xml)
    assert m and [float(v) for v in m.group(1).split()] == [0.0] * 6, (
        f"static obstacle model pose must be zero, got {m.group(1) if m else None}"
    )
    # 이름 중복 금지
    names = [o["name"] for o in L.STATIC_OBSTACLES]
    assert len(names) == len(set(names))
    print(f"PASS static obstacle layout ({len(L.STATIC_OBSTACLES)} obstacles on-route, lane-clear, static=true)")


if __name__ == "__main__":
    main()
