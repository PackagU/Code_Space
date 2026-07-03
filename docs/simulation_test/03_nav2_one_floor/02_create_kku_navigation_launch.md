# `kku_navigation.launch.py` 만드는 법

이 문서는 아래 목표 명령이 실제로 동작하도록 필요한 파일을 만들거나 확인하는 절차다.

```bash
ros2 launch slam_pkg kku_navigation.launch.py floor:=F2
```

이 명령은 정보성 예시가 아니라, launch 파일이 준비된 뒤 실제로 실행할 목표 명령이다.

## 1. Nav2 설치 확인

컨테이너 내부에서 확인한다.

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 pkg prefix nav2_bringup
```

경로가 출력되면 설치되어 있는 것이다. 없으면 컨테이너 내부에서 설치한다.

```bash
apt update
apt install -y ros-humble-navigation2 ros-humble-nav2-bringup
```

## 2. 만들 파일

필요한 파일은 2개다.

```text
src/slam_pkg/launch/kku_navigation.launch.py
src/slam_pkg/config/nav2_params.yaml
```

`src/slam_pkg/CMakeLists.txt`는 이미 `launch`, `config`, `maps` 디렉터리를 install하고 있어서 보통 추가 수정이 필요 없다.

## 3. `nav2_params.yaml` 만들기

처음에는 Nav2 기본 파라미터를 복사해서 시작하는 것이 가장 안전하다.

```bash
cd /ros2_ws
cp /opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml \
  src/slam_pkg/config/nav2_params.yaml
```

여기까지 했으면 파일 복사는 끝난 것이다.

아래 내용은 터미널에 입력하는 명령이 아니라, `src/slam_pkg/config/nav2_params.yaml` 파일 안에서 확인하거나 수정해야 하는 YAML 값이다.

```yaml
use_sim_time: true
global_frame: map
robot_base_frame: base_link
odom_topic: odom
```

costmap의 LaserScan topic은 `/scan`을 보게 해야 한다.

```yaml
topic: /scan
```

터미널에서 값이 들어있는지만 빠르게 확인하려면 아래처럼 `grep`을 쓴다.

```bash
grep -n "use_sim_time\\|global_frame\\|robot_base_frame\\|odom_topic\\|topic:" \
  src/slam_pkg/config/nav2_params.yaml
```

직접 수정해야 하면 컨테이너 안에서 편한 편집기를 연다.

```bash
nano src/slam_pkg/config/nav2_params.yaml
```

처음에는 완벽하게 튜닝하려고 하지 말고, Nav2가 켜지고 `/cmd_vel`이 나오는지부터 확인한다.

## 4. `kku_navigation.launch.py` 만들기

생성 위치:

```text
src/slam_pkg/launch/kku_navigation.launch.py
```

이미 이 파일이 있으면 새로 만들지 말고 내용만 확인한 뒤 빌드 단계로 넘어간다.

권장 구조는 Nav2의 `bringup_launch.py`를 include하는 방식이다. 이 방식은 `map_server`, `amcl`, `planner`, `controller`, `bt_navigator`, `lifecycle_manager`를 한 번에 관리해준다.

```python
"""KKU saved-map navigation launch.

Usage:
  ros2 launch slam_pkg kku_navigation.launch.py floor:=F1
  ros2 launch slam_pkg kku_navigation.launch.py floor:=F2
  ros2 launch slam_pkg kku_navigation.launch.py floor:=F3

Run Gazebo separately first:
  ros2 launch common_pkg gazebo.launch.py floor:=F2 use_sim_time:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


MAPS = {
    "F1": ("f1", "kku_f1.yaml"),
    "F2": ("f2", "kku_f2.yaml"),
    "F3": ("f3", "kku_f3.yaml"),
}


def _launch_setup(context, *args, **kwargs):
    floor = LaunchConfiguration("floor").perform(context).upper()
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    rviz = LaunchConfiguration("rviz").perform(context).lower()

    if floor not in MAPS:
        raise RuntimeError(f"floor must be one of {list(MAPS)}, got '{floor}'")

    pkg_slam = get_package_share_directory("slam_pkg")
    pkg_nav2 = get_package_share_directory("nav2_bringup")

    map_dir, map_file = MAPS[floor]
    map_yaml = os.path.join(pkg_slam, "maps", "kku_virtual", map_dir, map_file)
    params_file = os.path.join(pkg_slam, "config", "nav2_params.yaml")

    if not os.path.exists(map_yaml):
        raise RuntimeError(f"map yaml not found: {map_yaml}")
    if not os.path.exists(params_file):
        raise RuntimeError(f"nav2 params not found: {params_file}")

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2, "launch", "bringup_launch.py")
            ),
            launch_arguments={
                "slam": "False",
                "map": map_yaml,
                "use_sim_time": use_sim_time,
                "params_file": params_file,
                "autostart": "true",
            }.items(),
        ),
    ]

    if rviz in ("true", "1", "yes", "on"):
        rviz_config = os.path.join(pkg_nav2, "rviz", "nav2_default_view.rviz")
        rviz_args = ["-d", rviz_config] if os.path.exists(rviz_config) else []
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_nav2",
                output="screen",
                arguments=rviz_args,
                parameters=[{"use_sim_time": use_sim_time.lower() == "true"}],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "floor",
            default_value="F1",
            description="Which saved map to load: F1 | F2 | F3",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use Gazebo clock.",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Open RViz2 with Nav2 view.",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
```

## 5. `package.xml` 의존성

launch가 `nav2_bringup`을 사용하므로 `src/slam_pkg/package.xml`에 아래 의존성을 추가하는 것이 좋다.

```xml
<exec_depend>nav2_bringup</exec_depend>
```

Nav2 설치 패키지 자체는 apt로 관리한다.

## 6. 빌드

컨테이너 내부에서 빌드한다.

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select slam_pkg
source install/setup.bash
```

## 7. 다음 단계

파일을 만든 뒤에는 [03_run_one_floor_nav2.md](03_run_one_floor_nav2.md) 순서대로 실행한다.
