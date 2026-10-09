"""Website client for the single ROS arm owner; never opens a serial device."""
import json
import threading
import time
import uuid


class Session:
    def __init__(self, source):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from std_msgs.msg import String
        self.source, self.String = source, String
        self.condition, self.io = threading.Condition(), threading.Lock()
        self.arm, self.mission, self.responses = {}, {}, {}
        self.button = 'UP'
        self.context = Context()
        self.context.init(args=[])
        self.node = rclpy.create_node('packagu_camera_web', context=self.context)
        self.pub = self.node.create_publisher(String, '/packagu_arm/command', 10)
        self.stop_pub = self.node.create_publisher(String, '/delivery_mission/stop', 10)
        self.node.create_subscription(String, '/packagu_arm/status', self.on_arm,
                                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node.create_subscription(String, '/elevator/camera_state', self.on_mission, 10)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def on_arm(self, msg):
        try:
            status = json.loads(msg.data)
            if not isinstance(status, dict):
                return
            with self.condition:
                self.arm = status
                self.responses[status.get('request_id', '')] = status
                if len(self.responses) > 64:
                    self.responses.pop(next(iter(self.responses)))
                self.condition.notify_all()
        except (ValueError, TypeError):
            pass

    def on_mission(self, msg):
        try:
            state = json.loads(msg.data)
            if isinstance(state, dict):
                with self.condition:
                    self.mission = state
        except (ValueError, TypeError):
            pass

    def snapshot(self):
        with self.condition:
            return {'arm': dict(self.arm), 'mission': dict(self.mission)}

    def publish(self, command, publisher=None):
        msg = self.String()
        msg.data = json.dumps(command)
        (publisher or self.pub).publish(msg)

    def call(self, action, cancel=None, **fields):
        with self.io:
            with self.condition:
                if self.mission.get('mission_active'):
                    raise RuntimeError('자동 미션 중에는 수동 팔 조작을 사용할 수 없습니다')
            deadline = time.monotonic() + 3
            while self.node.count_subscribers('/packagu_arm/command') == 0:
                if time.monotonic() > deadline:
                    raise RuntimeError('ROS 팔 노드가 연결되지 않았습니다')
                time.sleep(.05)
            if self.node.count_subscribers('/packagu_arm/command') != 1:
                raise RuntimeError('팔 명령 수신 노드는 하나여야 합니다')
            request = dict(fields, action=action, request_id='web-' + uuid.uuid4().hex)
            self.publish(request)
            deadline = time.monotonic() + 30
            while True:
                if (cancel is not None and cancel.is_set()) or time.monotonic() > deadline:
                    self.publish(dict(action='cancel', request_id='web-stop-' + uuid.uuid4().hex,
                                      target_request_id=request['request_id']))
                    raise RuntimeError('중지 또는 팔 응답 시간 초과')
                with self.condition:
                    status = self.responses.get(request['request_id'], {})
                    if status.get('event') in ('failed', 'rejected', 'cancelled'):
                        raise RuntimeError(status.get('error') or '팔 명령 실패')
                    if status.get('event') == 'completed':
                        if (status.get('completion_basis') != 'controller_position_response'
                                or status.get('simulation_mode') is not False
                                or status.get('action') != action
                                or (action == 'view' and status.get('current_view') != fields['view'])):
                            raise RuntimeError('실제 컨트롤러 응답이 아닙니다')
                        measured = status.get('measured_pwm')
                        if (not isinstance(measured, dict) or set(measured) != {'000','001','002','003'}
                                or any(type(v) is not int for v in measured.values())):
                            raise RuntimeError('관절 위치 응답이 없습니다')
                        return dict(measured)
                    self.condition.wait(.1)

    def read(self):
        return self.call('read')

    def move(self, pose, duration_ms, cancel):
        if pose == self.source['home']:
            return self.call('home', cancel)
        return self.call('move', cancel, pose=pose, duration_ms=duration_ms)

    def view(self, name, cancel):
        return self.call('view', cancel, view=name)

    def press(self, pose, cancel, report):
        report('웹 인식·보정 자세를 ROS 팔 노드로 전달')
        return self.call('press', cancel, press_pose=pose, press_cycle=1,
                         target='call' if self.button in ('UP', 'DOWN') else 'destination',
                         button=self.button)

    def stop_idle(self):
        self.publish({'reason': 'website_stop'}, self.stop_pub)
        with self.condition:
            target = self.arm.get('request_id', '')
        self.publish(dict(action='cancel', request_id='web-stop-' + uuid.uuid4().hex,
                          target_request_id=target))

    def close(self):
        self.executor.shutdown(timeout_sec=1)
        self.thread.join(timeout=1)
        self.node.destroy_node()
        self.context.shutdown()
