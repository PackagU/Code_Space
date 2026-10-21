#!/usr/bin/env python3
"""촬영 리허설용 층 지도 교체 (ROS 컨테이너 안에서 실행).

Nav2 를 끄지 않고 /map_server/load_map 으로 지도만 바꾼 뒤 costmap 을 비운다.
Nav2 재시작(약 수십 초)과 팔 노드 충돌(start_field_navigation.sh 가 팔 노드를
거부)을 피하기 위해서다. 지도 버전 확인은 호출하는 쪽이 field_map_guard 로 한다.

결과는 마지막 줄 `MAP_SWITCH_RESULT {json}` 으로 출력한다.
"""

import argparse
import json
import re
import sys

YAML_RE = re.compile(r"^/ros2_ws/maps/[A-Za-z0-9_./-]+\.yaml$")
CLEAR_SERVICES = (
    "/global_costmap/clear_entirely_global_costmap",
    "/local_costmap/clear_entirely_local_costmap",
)


def emit(**fields):
    print("MAP_SWITCH_RESULT " + json.dumps(fields, ensure_ascii=False, sort_keys=True), flush=True)


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

    rclpy.init()
    node = rclpy.create_node("packagu_field_video_map_switch")
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
                result.setdefault("warnings", []).append(f"{service}: {error}")
            else:
                result["cleared"].append(service)
        result["ok"] = True
        return 0
    finally:
        emit(**result)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
