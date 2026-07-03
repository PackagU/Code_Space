from dataclasses import dataclass
import math


EPSILON = 1.0e-6


@dataclass(frozen=True)
class RoutePose:
    floor: str
    point_id: str
    point_type: str
    x: float
    y: float
    yaw_deg: float

    @property
    def yaw_rad(self):
        return math.radians(self.yaw_deg)


class OrthogonalRouter:
    def __init__(self, registry):
        self.registry = registry

    def route(self, floor, start_point_id, goal_point_id):
        start = self.registry.get(floor, start_point_id)
        goal = self.registry.get(floor, goal_point_id)
        routing = self.registry.routing(floor)

        if self._same_pose(start, goal):
            return []
        if self._aligned(start, goal):
            return [goal]

        waypoints = self._corridor_waypoints(start, goal, routing)
        compacted = self._drop_duplicate_positions([start] + waypoints + [goal])
        route = compacted[1:]
        return self._orient_route(start, route, goal)

    def _corridor_waypoints(self, start, goal, routing):
        if "main_corridor_x" in routing and abs(start.y - goal.y) > EPSILON:
            x = float(routing["main_corridor_x"])
            return [
                self._pose(start.floor, "route_1", x, start.y, 0.0),
                self._pose(start.floor, "route_2", x, goal.y, 0.0),
            ]
        if "main_corridor_y" in routing and abs(start.x - goal.x) > EPSILON:
            y = float(routing["main_corridor_y"])
            return [
                self._pose(start.floor, "route_1", goal.x, y, 0.0),
            ]
        return [
            self._pose(start.floor, "route_1", goal.x, start.y, 0.0),
        ]

    def _orient_route(self, start, route, goal):
        oriented = []
        for index, point in enumerate(route):
            next_point = route[index + 1] if index + 1 < len(route) else goal
            yaw = goal.yaw_deg if point.point_id == goal.point_id else self._yaw_to(point, next_point)
            oriented.append(
                RoutePose(
                    floor=point.floor,
                    point_id=point.point_id,
                    point_type=point.point_type,
                    x=point.x,
                    y=point.y,
                    yaw_deg=yaw,
                )
            )
        return oriented

    def _yaw_to(self, current, next_point):
        dx = next_point.x - current.x
        dy = next_point.y - current.y
        if abs(dx) <= EPSILON and abs(dy) <= EPSILON:
            return current.yaw_deg
        if abs(dx) >= abs(dy):
            return 0.0 if dx >= 0 else 180.0
        return 90.0 if dy >= 0 else -90.0

    def _pose(self, floor, point_id, x, y, yaw_deg):
        return RoutePose(
            floor=floor,
            point_id=point_id,
            point_type="route",
            x=float(x),
            y=float(y),
            yaw_deg=float(yaw_deg),
        )

    def _same_pose(self, a, b):
        return self._same_position(a, b) and abs(a.yaw_deg - b.yaw_deg) <= EPSILON

    def _aligned(self, a, b):
        return abs(a.x - b.x) <= EPSILON or abs(a.y - b.y) <= EPSILON

    def _same_position(self, a, b):
        return abs(a.x - b.x) <= EPSILON and abs(a.y - b.y) <= EPSILON

    def _drop_duplicate_positions(self, points):
        result = []
        for point in points:
            if result and self._same_position(result[-1], point):
                continue
            result.append(point)
        return result
