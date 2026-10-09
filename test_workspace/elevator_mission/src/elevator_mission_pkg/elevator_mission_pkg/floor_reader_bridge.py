"""Read the existing floor-reader HTTP API and publish live frame observations."""
import json
import time
from urllib.request import urlopen

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from elevator_mission_pkg.elevator_camera import reader_observation


class FloorReaderBridge(Node):
    def __init__(self):
        super().__init__("packagu_floor_reader_bridge")
        self.declare_parameter("reader_url", "http://127.0.0.1:8765/api/state")
        self.declare_parameter("observation_topic", "/elevator/vision_floor")
        self.pub = self.create_publisher(String, self.get_parameter("observation_topic").value, 10)
        self.create_timer(.1, self.poll)

    def poll(self):
        try:
            with urlopen(self.get_parameter("reader_url").value, timeout=.4) as response:
                data = json.loads(response.read(65536))
            observation = reader_observation(data, time.time())
        except (OSError, ValueError, TypeError):
            observation = {"valid": False, "floor": "UNKNOWN"}
        msg = String()
        msg.data = json.dumps(observation)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FloorReaderBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
