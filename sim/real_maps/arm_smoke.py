#!/usr/bin/env python3
"""Quick --arm-sim smoke on building.world (no driving): fold check, press cycle 1, lift load away from
the locker (attach must be REFUSED by the reach check), lift unload path. Simulation only, [가정값] arm.
  python3 sim/real_maps/arm_smoke.py --name arm_smoke_YYYYmmdd
"""
from host_exec import ensure_container
ensure_container(__file__)

import argparse   # noqa: E402
import json       # noqa: E402

import rclpy      # noqa: E402

from sim_stack import Stack, WORLD   # noqa: E402
from arm_lift import ArmLift         # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', required=True)
    args = parser.parse_args()
    rclpy.init()
    stack = Stack(args.name, 'building', 'F1', 'P0', 'f1_initial_test', record=False, arm=True)
    out = {'validation_scope': 'simulation', 'arm_geometry': 'assumed', 'error': None}
    try:
        stack.start()
        node = stack.node
        node.wait(lambda: all(n in node.joints for n in ('lift_joint', 'arm_joint_000', 'arm_joint_003')), 20)
        arm = ArmLift(stack, WORLD)
        out['joint_states_seen'] = sorted(node.joints)
        out['initial_stowed'] = arm.stowed()
        out['stow'] = arm.stow()
        out['press_cycle_1'] = arm.press_cycle(1, on_press=lambda: out.setdefault('on_press_fired', True))
        out['load_far_from_locker'] = arm.load_unload('load')
        out['attach_refused_far_from_locker'] = out['load_far_from_locker'].get('parcel', {}).get('parcel_state') == 'on_shelf'
        out['final_lift_m'] = node.joints.get('lift_joint')
    except Exception as exc:
        out['error'] = str(exc)
    finally:
        stack.finish()
        rclpy.try_shutdown()
    print('ARM_SMOKE '+json.dumps(out, default=str))


if __name__ == '__main__':
    main()
