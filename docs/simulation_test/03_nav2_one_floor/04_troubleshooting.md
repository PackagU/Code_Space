# Nav2 One-Floor 문제 해결

## 1. Nav2 launch 파일을 못 찾음

증상:

```text
file 'kku_navigation.launch.py' was not found
```

확인:

```bash
ls src/slam_pkg/launch
colcon build --symlink-install --packages-select slam_pkg
source install/setup.bash
ros2 pkg prefix slam_pkg
```

`src/slam_pkg/launch/kku_navigation.launch.py`를 만든 뒤 빌드하지 않았거나, `source install/setup.bash`를 다시 하지 않은 경우가 많다.

## 2. map yaml을 못 찾음

증상:

```text
map yaml not found
```

확인:

```bash
ls src/slam_pkg/maps/kku_virtual/f2
```

필수 파일:

```text
kku_f2.yaml
kku_f2.pgm
```

YAML 안의 `image:` 경로도 실제 `.pgm` 파일 이름과 맞아야 한다.

## 3. `/map`이 안 나옴

확인:

```bash
ros2 node list | grep map
ros2 topic list | grep map
ros2 topic echo /map --once
```

`map_server`가 lifecycle active 상태가 아니면 `/map`이 안 나올 수 있다. `kku_navigation.launch.py`에서 `autostart`가 `true`인지 확인한다.

## 4. `/amcl_pose`가 안 나옴

확인:

```bash
ros2 topic echo /scan --once
ros2 topic echo /tf --once
ros2 topic echo /odom --once
ros2 topic echo /amcl_pose --once
```

AMCL은 `/scan`, `/map`, `/tf`가 모두 필요하다. 특히 `base_link`, `odom`, `map` frame 연결이 깨지면 위치추정이 실패한다.

## 5. RViz에서 goal을 찍어도 path가 안 나옴

먼저 `2D Pose Estimate`를 찍었는지 확인한다. AMCL 초기 위치가 없으면 Nav2가 현재 위치를 모른다.

추가 확인:

```bash
ros2 topic echo /goal_pose --once
ros2 action list | grep navigate
ros2 topic echo /plan --once
```

## 6. `/cmd_vel`은 나오는데 로봇이 안 움직임

이 경우 Nav2보다 Gazebo diff_drive, `/cmd_vel` remap, robot URDF 쪽 문제일 가능성이 높다.

확인:

```bash
ros2 topic info /cmd_vel
ros2 topic echo /cmd_vel --once
ros2 topic echo /odom --once
```

수동 teleop으로도 움직이는지 비교한다.

```bash
ros2 run drive_pkg keyboard_teleop
```

teleop도 안 움직이면 Nav2 문제가 아니라 구동부 시뮬레이션 문제다.

## 7. 로봇이 벽을 긁거나 너무 흔들림

처음에는 아래 파라미터를 작게 잡는 쪽이 안전하다.

```text
max_vel_x
max_vel_theta
inflation_radius
robot_radius
```

정확한 튜닝은 RViz에서 costmap과 footprint를 보면서 조정한다.

