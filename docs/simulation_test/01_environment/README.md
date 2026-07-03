# 01 Environment

시뮬레이션을 돌리기 전에 필요한 Docker, VSCode attach, GUI, ROS2 workspace 준비 문서다.

## 추천 순서

1. Linux desktop에서 Docker/ROS2 SLAM 환경을 처음 잡는다면  
   [01_linux_docker_slam_setup.md](01_linux_docker_slam_setup.md)

2. VSCode에서 컨테이너 attach 방식으로 작업한다면  
   [02_vscode_docker_slam_workflow.md](02_vscode_docker_slam_workflow.md)

3. KKU 시뮬레이션을 VSCode attach 기준으로 A to Z 실행한다면  
   [03_vscode_docker_kku_simulation_atoz.md](03_vscode_docker_kku_simulation_atoz.md)

4. Docker compose 자체가 헷갈리면  
   [04_docker_guide.md](04_docker_guide.md)

## 기준 작업 방식

사용자는 컨테이너 attach 방식이 편하므로, 이후 문서의 명령은 기본적으로 컨테이너 내부 터미널 기준이다.

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

