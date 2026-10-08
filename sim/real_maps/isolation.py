"""Refuse to run simulation code outside the isolated container (sim_container.sh).

The field Jetson is on the same LAN; simulated /cmd_vel must never reach the real robot.
"""
import os


def assert_isolated():
    domain = os.environ.get('ROS_DOMAIN_ID', '')
    problems = []
    if os.environ.get('PACKAGU_SIM_ISOLATED') != '1':
        problems.append('not inside sim_container.sh container')
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        problems.append('ROS_LOCALHOST_ONLY != 1')
    if not domain.isdigit() or int(domain) == 0:
        problems.append('ROS_DOMAIN_ID missing or 0')
    if 'lan_peers' in os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE', ''):
        problems.append('LAN peer DDS profile in use')
    if problems:
        raise SystemExit('REFUSED (isolation): '+'; '.join(problems))
