import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ElevatorSimNode(Node):
    def __init__(self):
        super().__init__("elevator_sim_node")
        self.declare_parameter("initial_floor", "F1")
        self.declare_parameter("floor_travel_time_sec", 5.0)
        self.declare_parameter("door_open_time_sec", 3.0)

        self.current_floor = self.get_parameter("initial_floor").value
        self.target_floor = self.current_floor
        self.state = "IDLE"
        self.door_state = "open"
        self.state_started = time.monotonic()

        self.state_pub = self.create_publisher(String, "/elevator/state", 10)
        self.call_sub = self.create_subscription(String, "/elevator/call", self.on_call, 10)
        self.create_timer(0.2, self.tick)

    def on_call(self, msg):
        command = msg.data.strip()
        if command in ("F1", "F2", "F3"):
            self.target_floor = command
            if self.target_floor == self.current_floor:
                self.state = "ARRIVED_OPEN"
                self.door_state = "open"
            else:
                self.state = "CALLED"
                self.door_state = "closing"
            self.state_started = time.monotonic()
        elif command == "close":
            self.state = "CLOSING"
            self.door_state = "closing"
            self.state_started = time.monotonic()
        elif command == "open":
            self.state = "ARRIVED_OPEN"
            self.door_state = "open"
            self.state_started = time.monotonic()
        else:
            self.get_logger().warning(f"ignored elevator command: {command}")

    def tick(self):
        elapsed = time.monotonic() - self.state_started
        travel_time = float(self.get_parameter("floor_travel_time_sec").value)
        door_time = float(self.get_parameter("door_open_time_sec").value)

        if self.state == "CALLED" and elapsed >= 0.5:
            self.state = "MOVING"
            self.door_state = "closed"
            self.state_started = time.monotonic()
        elif self.state == "CLOSING" and elapsed >= 0.5:
            self.state = "MOVING"
            self.door_state = "closed"
            self.state_started = time.monotonic()
        elif self.state == "MOVING" and elapsed >= travel_time:
            self.current_floor = self.target_floor
            self.state = "ARRIVED_OPEN"
            self.door_state = "open"
            self.state_started = time.monotonic()
        elif self.state == "ARRIVED_OPEN" and elapsed >= door_time:
            self.state = "IDLE"

        payload = {
            "current_floor": self.current_floor,
            "target_floor": self.target_floor,
            "door_state": self.door_state,
            "state": self.state,
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.state_pub.publish(msg)


def main():
    rclpy.init()
    node = ElevatorSimNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
