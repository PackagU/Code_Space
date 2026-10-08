#!/usr/bin/env python3
"""Compare AMCL to interpolated ground truth at each message's ROS header time.

Cached AMCL poses in clearance.csv must not be compared to newer truth samples.
This reads only completed simulation bags and exports small numerical summaries.
"""
import argparse
import json
import math
import re
import sqlite3
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec / 1e9


def pose(message):
    p = message.pose.pose
    q = p.orientation
    return [stamp(message), p.position.x, p.position.y,
            math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))]


def analyze(directory, deserialize_message, types):
    values = {name: [] for name in types}
    for database in sorted((directory/'bag').glob('*.db3')):
        with sqlite3.connect(f'file:{database}?mode=ro', uri=True) as connection:
            for name, blob in connection.execute(
                    'SELECT topics.name, messages.data FROM messages JOIN topics '
                    'ON messages.topic_id=topics.id WHERE topics.name IN (?,?) '
                    'ORDER BY messages.timestamp', tuple(types)):
                values[name].append(pose(deserialize_message(blob, types[name])))
    truth = np.asarray(values['/sim/ground_truth'])
    estimates = np.asarray(values['/amcl_pose'])
    if len(truth) < 2 or not len(estimates):
        raise ValueError('missing timestamped truth or AMCL samples')
    truth = truth[np.argsort(truth[:, 0])]
    estimates = estimates[np.argsort(estimates[:, 0])]
    # Drop duplicates and refuse extrapolation or gaps larger than 0.15 seconds.
    truth = truth[np.r_[True, np.diff(truth[:, 0]) > 0]]
    indices = np.searchsorted(truth[:, 0], estimates[:, 0])
    valid = (indices > 0) & (indices < len(truth))
    indices = np.clip(indices, 1, len(truth)-1)
    valid &= (truth[indices, 0]-truth[indices-1, 0]) <= .15
    estimates = estimates[valid]
    if not len(estimates):
        raise ValueError('no AMCL timestamps within a continuous truth interval')
    xy = np.column_stack([np.interp(estimates[:, 0], truth[:, 0], truth[:, axis])
                          for axis in (1, 2)])
    yaw = np.interp(estimates[:, 0], truth[:, 0], np.unwrap(truth[:, 3]))
    errors = np.linalg.norm(estimates[:, 1:3]-xy, axis=1)
    angles = np.abs(np.arctan2(np.sin(estimates[:, 3]-yaw), np.cos(estimates[:, 3]-yaw)))
    final_count = min(10, len(errors))
    result = dict(scope='simulation', alignment='message header stamp; interpolated ground truth',
                  truth_samples=len(truth), amcl_samples=len(values['/amcl_pose']),
                  matched_samples=len(errors), max_truth_gap_s=.15,
                  first_stamp_s=float(estimates[0, 0]), last_stamp_s=float(estimates[-1, 0]),
                  first_xy_error_m=float(errors[0]), final_xy_error_m=float(errors[-1]),
                  final_yaw_error_deg=float(np.degrees(angles[-1])),
                  max_xy_error_m=float(errors.max()), final_window_samples=final_count,
                  final_window_max_xy_error_m=float(errors[-final_count:].max()),
                  recovered_to_025_by_end=bool(errors[0] > .25 and
                                              final_count == 10 and np.all(errors[-10:] <= .25)))
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', help='one completed S* condition, otherwise all completed conditions')
    args = parser.parse_args()
    if args.name and not re.fullmatch(r'S[A-Za-z0-9_-]+', args.name):
        parser.error('name must identify a simulation S* condition')
    try:
        from rclpy.serialization import deserialize_message
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from nav_msgs.msg import Odometry
    except ImportError:
        print('SKIP: ROS message deserialization unavailable')
        return
    log = ROOT/'logs/real_map_sim'
    destination = log/'localization_alignment'
    destination.mkdir(exist_ok=True)
    paths = [log/args.name/'result.json'] if args.name else sorted(log.glob('S*/result.json'))
    for path in paths:
        if not path.exists() or 'truth_samples' not in json.loads(path.read_text()):
            continue
        result = analyze(path.parent, deserialize_message,
                         {'/sim/ground_truth': Odometry, '/amcl_pose': PoseWithCovarianceStamped})
        (destination/(path.parent.name+'.json')).write_text(json.dumps(result, indent=2)+'\n')
        print(path.parent.name, json.dumps(result))


if __name__ == '__main__':
    main()
