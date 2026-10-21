#!/usr/bin/env python3
"""촬영 리허설용 층 지도 교체 (ROS 컨테이너 안에서 실행).

Nav2 를 끄지 않고 /map_server/load_map 으로 지도만 바꾼 뒤 costmap 을 비운다.
새 /map 과 전역 costmap 의 크기(m)가 같아질 때까지 기다려 정적 지도 반영을 확인한다.
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
            node.create_subscription(OccupancyGrid, "/map", lambda m: grids.__setitem__("map", m), latched)
            node.create_subscription(OccupancyGrid, "/global_costmap/costmap",
                                     lambda m: grids.__setitem__("costmap", m), latched)
            deadline = time.monotonic() + args.timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if grids["map"] is not None and grids["costmap"] is not None and \
                        same_size(size_m(grids["map"]), size_m(grids["costmap"])):
                    break
            else:
                result["reason"] = "global costmap did not take the new map size within timeout"
                result["map_size_m"] = grids["map"] and [round(v, 2) for v in size_m(grids["map"])]
                result["costmap_size_m"] = grids["costmap"] and [round(v, 2) for v in size_m(grids["costmap"])]
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
