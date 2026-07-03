# 02 Gazebo + SLAM Mapping

이 단계는 Gazebo에서 F1/F2/F3 층을 열고, SLAM Toolbox로 맵을 만든 뒤 저장하는 단계다.

## 추천 순서

1. 시뮬레이션 맵/월드 설계가 궁금하면  
   [01_kku_pre_simulation_plan.md](01_kku_pre_simulation_plan.md)

2. 어떤 파일이 만들어졌고 무엇이 검증됐는지 보려면  
   [02_kku_pre_simulation_completion_report.md](02_kku_pre_simulation_completion_report.md)

3. 실제로 Gazebo + SLAM + teleop + map 저장을 하려면  
   [03_kku_simulation_quickstart.md](03_kku_simulation_quickstart.md)

## 이 단계의 완료 기준

아래 파일들이 존재하면 Nav2 테스트로 넘어갈 수 있다.

```text
src/slam_pkg/maps/kku_virtual/f1/kku_f1.yaml
src/slam_pkg/maps/kku_virtual/f2/kku_f2.yaml
src/slam_pkg/maps/kku_virtual/f3/kku_f3.yaml
```

각 YAML이 참조하는 `.pgm` 파일도 같은 폴더에 있어야 한다.

