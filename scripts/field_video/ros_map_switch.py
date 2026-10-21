#!/usr/bin/env python3
"""촬영 리허설용 층 지도 교체 (ROS 컨테이너 안에서 실행).

Nav2 를 끄지 않고 /map_server/load_map 으로 지도만 바꾼 뒤 costmap 을 비운다.
반영 확인(크기만 비교하지 않는다):
  1. /map 이 요청한 YAML·PGM 의 폭·높이·해상도·원점과 정확히 같아야 한다.
  2. 전역 costmap 이 load 이후 시각으로 새로 발행되고, 크기(m)·원점이 새 지도와 같아야 한다.
     (전역 costmap 은 정적 지도 레이어라 지도 크기·원점이 바뀌면 전체를 다시 발행한다.)
costmap 비우기나 반영 확인이 실패하면 실패로 끝낸다(다음 goal 금지). 같은 크기·원점의 지도로
바꾸면 costmap 이 다시 발행되지 않아 실패하므로, 그때는 Nav2 를 다시 켠다.
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


def costmap_matches(costmap, grid):
    res = max(grid.info.resolution, costmap.info.resolution)
    return (same_size(size_m(costmap), size_m(grid), tolerance=2 * res)
            and abs(costmap.info.origin.position.x - grid.info.origin.position.x) <= 2 * res
            and abs(costmap.info.origin.position.y - grid.info.origin.position.y) <= 2 * res)


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


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
    from nav2_msgs.srv import ClearEntireCostmap, LoadMap
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
            node.create_subscription(OccupancyGrid, "/global_costmap/costmap",
                                     lambda m: grids.__setitem__("costmap", m), latched)
            loaded_at = stamp_ns(node.get_clock().now().to_msg())
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
            deadline = time.monotonic() + args.timeout
            map_ok = cost_ok = False
            while time.monotonic() < deadline and not (map_ok and cost_ok):
                rclpy.spin_once(node, timeout_sec=0.1)
                map_ok = grids["map"] is not None and map_matches(grids["map"], geometry)
                cost_ok = (map_ok and grids["costmap"] is not None
                           and stamp_ns(grids["costmap"].header.stamp) >= loaded_at
                           and costmap_matches(grids["costmap"], grids["map"]))
            result["map_matches_yaml"] = map_ok
            result["costmap_fresh_and_matching"] = cost_ok
            if not map_ok:
                result["reason"] = "/map does not match the requested map file (size/resolution/origin)"
                return 1
            if not cost_ok:
                result["reason"] = "global costmap was not republished with the new map geometry after load"
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
