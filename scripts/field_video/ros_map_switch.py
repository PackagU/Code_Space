#!/usr/bin/env python3
"""촬영 리허설용 층 지도 교체 (ROS 컨테이너 안에서 실행).

Nav2 를 끄지 않고 /map_server/load_map 으로 지도만 바꾼 뒤 costmap 을 비운다.
반영 확인(크기만 비교하지 않는다):
  1. /map 이 요청한 YAML·PGM 의 폭·높이·해상도·원점과 정확히 같아야 한다.
  2. /global_costmap/get_costmap 으로 지금 costmap 을 직접 받아, 크기(m)·원점이 새 지도와 같고
     새 지도의 벽 칸(점유 100)이 costmap 에서 거의 다 lethal(254)인지 본다(해상도가 같을 때).
     토픽 발행을 기다리지 않으므로 같은 지도로 다시 시도해도 확인할 수 있다.
costmap 비우기나 반영 확인이 실패하면 실패로 끝낸다(다음 goal 금지).
Nav2 재시작(약 수십 초)과 팔 노드 충돌(start_field_navigation.sh 가 팔 노드를
거부)을 피하기 위해서다. 지도 버전 확인은 호출하는 쪽이 field_map_guard 로 한다.

결과는 마지막 줄 `MAP_SWITCH_RESULT {json}` 으로 출력한다.
"""

import argparse
import json
import re
import sys
import time

YAML_RE = re.compile(r"^/ros2_ws/maps/[A-Za-z0-9_./-]+\.yaml$")
CLEAR_SERVICES = (
    "/global_costmap/clear_entirely_global_costmap",
    "/local_costmap/clear_entirely_local_costmap",
)


def emit(**fields):
    print("MAP_SWITCH_RESULT " + json.dumps(fields, ensure_ascii=False, sort_keys=True), flush=True)


def size_m(grid):
    return grid.info.width * grid.info.resolution, grid.info.height * grid.info.resolution


def same_size(a, b, tolerance=0.15):
    return abs(a[0] - b[0]) <= tolerance and abs(a[1] - b[1]) <= tolerance


def pgm_size(path):
    """PGM(P5/P2) 헤더의 폭·높이."""
    tokens = []
    with open(path, "rb") as handle:
        while len(tokens) < 3:
            line = handle.readline()
            if not line:
                break
            line = line.split(b"#", 1)[0]
            tokens.extend(line.split())
    if len(tokens) < 3 or tokens[0] not in (b"P5", b"P2"):
        raise ValueError(f"not a PGM file: {path}")
    return int(tokens[1]), int(tokens[2])


def expected_geometry(yaml_path):
    import os
    import yaml

    with open(yaml_path, encoding="utf-8") as handle:
        meta = yaml.safe_load(handle)
    image = meta["image"] if os.path.isabs(meta["image"]) else os.path.join(os.path.dirname(yaml_path), meta["image"])
    width, height = pgm_size(image)
    return {"width": width, "height": height, "resolution": float(meta["resolution"]),
            "origin": [float(meta["origin"][0]), float(meta["origin"][1])]}


def map_matches(grid, geometry, tol=1e-3):
    info = grid.info
    return (info.width == geometry["width"] and info.height == geometry["height"]
            and abs(info.resolution - geometry["resolution"]) < tol
            and abs(info.origin.position.x - geometry["origin"][0]) < tol
            and abs(info.origin.position.y - geometry["origin"][1]) < tol)


LETHAL = 254
MIN_WALL_MATCH = 0.90


def costmap_matches_map(meta, data, grid, sample_limit=4000):
    """GetCostmap 결과(meta, data)가 /map(grid)을 반영했는지. (일치 여부, 벽 일치 비율 또는 None)."""
    map_w, map_h = grid.info.width * grid.info.resolution, grid.info.height * grid.info.resolution
    cost_w, cost_h = meta.size_x * meta.resolution, meta.size_y * meta.resolution
    res = max(grid.info.resolution, meta.resolution)
    geometry = (abs(map_w - cost_w) <= 2 * res and abs(map_h - cost_h) <= 2 * res
                and abs(meta.origin.position.x - grid.info.origin.position.x) <= 2 * res
                and abs(meta.origin.position.y - grid.info.origin.position.y) <= 2 * res)
    if not geometry:
        return False, None
    if abs(meta.resolution - grid.info.resolution) > 1e-6 or meta.size_x != grid.info.width:
        return True, None  # 해상도가 다르면 형상만 본다
    walls = [i for i, v in enumerate(grid.data) if v == 100]
    if not walls:
        return True, None
    step = max(1, len(walls) // sample_limit)
    sample = walls[::step]
    hit = sum(1 for i in sample if i < len(data) and data[i] == LETHAL)
    ratio = hit / len(sample)
    return ratio >= MIN_WALL_MATCH, round(ratio, 3)


def call(rclpy, node, client, request, timeout):
    if not client.wait_for_service(timeout_sec=timeout):
        return None, "service unavailable"
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done() or future.result() is None:
        return None, "no response"
    return future.result(), None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-yaml", default="")
    parser.add_argument("--clear-only", action="store_true")
    parser.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args(argv)
    if not args.clear_only and not YAML_RE.match(args.map_yaml):
        emit(ok=False, reason="map yaml must be an absolute path below /ros2_ws/maps")
        return 2

    import rclpy
    from nav2_msgs.srv import ClearEntireCostmap, GetCostmap, LoadMap
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    rclpy.init()
    node = rclpy.create_node("packagu_field_video_map_switch")
    grids = {"map": None, "costmap": None}
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    result = {"ok": False, "map_yaml": args.map_yaml or None, "cleared": []}
    try:
        if not args.clear_only:
            try:
                geometry = expected_geometry(args.map_yaml)
            except (OSError, ValueError, KeyError) as exc:
                result["reason"] = f"cannot read map geometry: {exc}"
                return 1
            result["expected"] = geometry
            node.create_subscription(OccupancyGrid, "/map", lambda m: grids.__setitem__("map", m), latched)
            request = LoadMap.Request()
            request.map_url = args.map_yaml
            response, error = call(rclpy, node, node.create_client(LoadMap, "/map_server/load_map"),
                                   request, args.timeout)
            if error:
                result["reason"] = f"load_map {error}"
                return 1
            result["load_result"] = int(response.result)
            if response.result != LoadMap.Response.RESULT_SUCCESS:
                result["reason"] = f"load_map failed result={response.result}"
                return 1
        for service in CLEAR_SERVICES:
            _, error = call(rclpy, node, node.create_client(ClearEntireCostmap, service),
                            ClearEntireCostmap.Request(), 5.0)
            if error:
                result["reason"] = f"{service}: {error}"
                return 1
            result["cleared"].append(service)
        if not args.clear_only:
            get_costmap = node.create_client(GetCostmap, "/global_costmap/get_costmap")
            deadline = time.monotonic() + args.timeout
            map_ok = cost_ok = False
            wall_ratio = None
            next_query = 0.0
            while time.monotonic() < deadline and not (map_ok and cost_ok):
                rclpy.spin_once(node, timeout_sec=0.1)
                map_ok = grids["map"] is not None and map_matches(grids["map"], geometry)
                if map_ok and time.monotonic() >= next_query:
                    next_query = time.monotonic() + 0.5
                    response, error = call(rclpy, node, get_costmap, GetCostmap.Request(), 3.0)
                    if error:
                        result["costmap_query"] = error
                        continue
                    cost_ok, wall_ratio = costmap_matches_map(response.map.metadata, response.map.data, grids["map"])
            result["map_matches_yaml"] = map_ok
            result["costmap_matches_map"] = cost_ok
            result["wall_match_ratio"] = wall_ratio
            if not map_ok:
                result["reason"] = "/map does not match the requested map file (size/resolution/origin)"
                return 1
            if not cost_ok:
                result["reason"] = "global costmap does not reflect the new map (geometry or walls)"
                return 1
            result["map_size_m"] = [round(v, 2) for v in size_m(grids["map"])]
        result["ok"] = True
        return 0
    finally:
        emit(**result)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
