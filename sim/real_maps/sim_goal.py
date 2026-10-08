#!/usr/bin/env python3
"""Send one Nav2 goal to a running start_sim.sh stack and print measured results (simulation only).

  python3 sim/real_maps/sim_goal.py --name RUN --to f1_locker [--timeout 300]
"""
from host_exec import ensure_container
ensure_container(__file__)

import argparse   # noqa: E402
import json       # noqa: E402

import rclpy      # noqa: E402

from sim_stack import Stack, resolve_pose, LOG   # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', required=True, help='run name given to start_sim.sh')
    parser.add_argument('--to', required=True, help='waypoint name')
    parser.add_argument('--timeout', type=float, default=300.0, help='[제안값] sim seconds per goal')
    args = parser.parse_args()
    rclpy.init()
    stack = Stack.attach(args.name)
    goal = resolve_pose(args.to)
    if goal[3] != stack.floor:
        raise SystemExit(f'{args.to} is on {goal[3]}; running stack is {stack.floor}')
    stack.node.wait(lambda: stack.node.truth is not None and stack.node.client.server_is_ready(), 30)
    seg = stack.begin(args.to)
    status = stack.navigate(goal, args.timeout)
    row = stack.end(seg, status, goal)
    out = LOG/args.name/f'goal_{args.to}_{int(seg["sim_start"])}.json'
    out.write_text(json.dumps(row, indent=2, default=str)+'\n')
    print(json.dumps(row, default=str))
    stack.finish()
    rclpy.try_shutdown()
    raise SystemExit(0 if status == 'SUCCEEDED' else 1)


if __name__ == '__main__':
    main()
