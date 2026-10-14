#!/usr/bin/env python3
"""Lamp/request lifecycle through idle, open, travel and arrival (no Gazebo required)."""
from types import SimpleNamespace
import unittest

try:
    from elevator_sim_node import Elevator, spin_simulator
except ModuleNotFoundError as exc:
    if exc.name in ('rclpy', 'geometry_msgs', 'gazebo_msgs', 'nav_msgs', 'sensor_msgs', 'std_msgs'):
        print('SKIP elevator lamp lifecycle (ROS unavailable)')
        raise SystemExit(0)
    raise


class ElevatorLampTests(unittest.TestCase):
    def scene(self, phase):
        t = [0.0]
        e = SimpleNamespace(current='F2', phase=phase, requests=[], request_ready_at={},
                            timing={'call_response_s':20., 'hold_open_s':60., 'travel_per_floor_s':45.,
                                    'door_open_s':2.5, 'door_close_s':2.5},
                            car_lit=set(), hall_lit=set(), events={'calls':0,'travel':0}, fraction=1. if phase=='open' else 0.,
                            last_tick=None, idle_since=None, hold_until=100., carry_robot=False,
                            now=lambda:t[0], inside=lambda _:False,
                            get_logger=lambda:SimpleNamespace(warning=lambda _:None),
                            get_parameter=lambda _:SimpleNamespace(value=False), push_leaves=lambda:None,
                            push_indicators=lambda:None, publish=lambda _:None)
        e.arrived = lambda floor: Elevator.arrived(e,floor)
        return e,t

    def test_call_to_current_floor_stays_lit_until_request_is_handled_then_goes_out(self):
        for phase in ('idle','open'):
            with self.subTest(phase=phase):
                e,t = self.scene(phase)
                Elevator.request(e,'call','F2')
                t[0] = 19.
                Elevator.tick(e)
                self.assertIn('F2',e.hall_lit)
                t[0] = 20.
                Elevator.tick(e)
                self.assertNotIn('F2',e.hall_lit)
                self.assertNotIn('F2',e.requests)
                self.assertIn(e.phase,('opening','open'))

    def test_floor_button_lights_until_destination_arrival_then_goes_out(self):
        e,t = self.scene('idle')
        Elevator.request(e,'go','F1')
        Elevator.tick(e)
        self.assertEqual(e.phase,'moving')
        self.assertIn('F1',e.car_lit)
        t[0] = 44.
        Elevator.tick(e)
        self.assertIn('F1',e.car_lit)
        t[0] = 45.
        Elevator.tick(e)
        self.assertEqual(e.current,'F1')
        self.assertEqual(e.phase,'opening')
        self.assertFalse(e.car_lit)

    def test_native_conversion_failure_is_bounded_and_unrelated_errors_escape(self):
        e,_ = self.scene('idle')
        calls = []
        def recover_once(node, **kw):
            calls.append(1)
            if len(calls)==1:
                raise RuntimeError('Unable to convert call argument to Python object (compile in debug mode for details)')
        spin_simulator(e,recover_once,lambda:len(calls)<2)
        self.assertEqual(e.events['native_message_drop'],1)
        def always_bad(node, **kw):
            raise RuntimeError('Unable to convert call argument to Python object')
        with self.assertRaises(RuntimeError):
            spin_simulator(e,always_bad,lambda:True)
        self.assertEqual(e.events['native_message_drop'],3)
        def unrelated(node, **kw):
            raise RuntimeError('broken scene geometry')
        with self.assertRaisesRegex(RuntimeError,'broken scene'):
            spin_simulator(e,unrelated,lambda:True)

    def test_arm_pause_ack_is_not_sent_until_trajectory_has_finished(self):
        from arm_lift import ArmLift
        t=[0.0]; messages=[]; writes=[]
        arm=SimpleNamespace(node=SimpleNamespace(elevator={'transferring':True},now=lambda:t[0]),
                            pose_writes_until=2.,pause_pub=SimpleNamespace(publish=messages.append),
                            hold={'arm_joint_000':0.},lift_hold=.1,
                            _send=lambda *a:writes.append(a),_lift=lambda *a:writes.append(a))
        ArmLift._hold(arm)
        self.assertFalse(messages[-1].data)
        t[0]=2.4
        ArmLift._hold(arm)
        self.assertTrue(messages[-1].data)
        self.assertFalse(writes)
        arm.node.elevator['transferring']=False
        ArmLift._hold(arm)
        self.assertFalse(messages[-1].data)
        self.assertEqual(len(writes),2)

    def test_moving_robot_is_not_teleported_and_wait_is_bounded(self):
        e,t=self.scene('moving')
        e.pending_carry=None
        e.transfer_prepare_since=None
        e.truth=SimpleNamespace(twist=SimpleNamespace(twist=SimpleNamespace(
            linear=SimpleNamespace(x=0.,y=0.),angular=SimpleNamespace(z=.3))))
        self.assertFalse(Elevator.carry(e,'F2','F1'))
        self.assertIsNone(e.pending_carry)
        t[0]=5.1
        self.assertFalse(Elevator.carry(e,'F2','F1'))
        self.assertEqual(e.phase,'fault')

    def test_fork_and_load_hold_door_open_before_body_reaches_sensor(self):
        e,_=self.scene('closing')
        e.current='F1'
        e.get_parameter=lambda _:SimpleNamespace(value=True)
        e.loaded=False
        e.empty_front=.26;e.loaded_front=.345;e.rear_extent=.327
        e.half_width=e.loaded_half=.219
        e.robot_floor=lambda:'F1'
        position=[0.,-.38,1.57079632679]
        e.robot_map_pose=lambda _:position
        e.door_box=lambda _:[0.,0.,1.,.30,0.]
        e.footprint=lambda:Elevator.footprint(e)
        self.assertTrue(Elevator.obstructing(e))
        position[1]=-.50
        self.assertFalse(Elevator.obstructing(e))
        e.loaded=True
        self.assertTrue(Elevator.obstructing(e))


if __name__ == '__main__':
    unittest.main()
