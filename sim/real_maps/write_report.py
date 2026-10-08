#!/usr/bin/env python3
"""Regenerate the review report from completed observations, never fabricate runs."""
import json
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
LOG = ROOT/'logs/real_map_sim'


def main():
    rows=[]
    for path in sorted(LOG.glob('S*/result.json')):
        data=json.loads(path.read_text())
        if 'truth_samples' in data:
            rows.append(data)
    geometry=json.loads((HERE/'generated/geometry_validation.json').read_text())
    def val(value):
        return '미측정' if value is None else f'{value:.3f}'
    table=['| 조건 | action | 최소 벽 거리 m | 코너 거리 m | collision-ahead | lethal-start | rpm 차단 | ready 이탈 | 시간 s |',
           '|---|---|---|---|---|---|---|---|---|']
    for row in rows:
        table.append('| '+ ' | '.join([row['name'], '/'.join(row['action_results']), val(row.get('min_wall_m')),
            val(row.get('corner_min_wall_m')),str(row.get('collision_ahead')),str(row.get('lethal_start')),
            str(row.get('rpm_blocks')),str(row.get('ready_drops')),val(row.get('elapsed_sim_s'))])+' |')
    complete_names={r['name'] for r in rows}
    expected=[]
    for rep in range(1,4):
        for p in ('P0','P1'):
            for target in ('f2_elevator_entry','f2_elevator_staging_v1'):
                expected.append(f'S1_{target}_{p}_r{rep}')
            expected += [f'S2_{p}_r{rep}', f'S3_{p}_r{rep}']
        expected += [f'S5_{p}_r{rep}' for p in ('P1','v011','v012')]
    missing=[n for n in expected if n not in complete_names]
    interventions = json.loads((LOG/'user_interventions.json').read_text()) if (LOG/'user_interventions.json').exists() else {}
    missions=[]
    for path in sorted(LOG.glob('M*/result.json')):
        data=json.loads(path.read_text())
        missions.append(f"| {data['name']} | {'완주' if data.get('completed') else '미완주'} | {len(data.get('stages',[]))} | {('사용자 요청으로 중단; autonomous 실패 판정에서 제외' if data['name'] in interventions else data.get('error','없음'))} |")
    operational=[]
    for path in sorted(LOG.glob('sim_s6*/result.json')):
        data=json.loads(path.read_text())
        operational.append(f"| {path.parent.name} | {data.get('original_stop_metadata_present','보정안 직접 실행')} | {data.get('bag_finalized')} | {data.get('contract_exit')} | {data.get('analysis_exit')} |")
    localization=['| 조건 | 초기 오차 m | 마지막 오차 m | 마지막 10표본 최대 m | 0.25 m 이하 회복 |',
                  '|---|---|---|---|---|']
    for path in sorted(LOG.glob('localization_alignment/S3*.json')):
        data=json.loads(path.read_text())
        localization.append(f"| {path.stem} | {val(data['first_xy_error_m'])} | {val(data['final_xy_error_m'])} | {val(data['final_window_max_xy_error_m'])} | {data['recovered_to_025_by_end']} |")
    p1_unsafe=[r['name'] for r in rows if r['params']=='P1' and r['scenario'] in ('S1','S3') and
               ((r.get('min_wall_m') or 0)<.2 or r.get('lethal_start',0))]
    content=f'''# 2026-09-15 실측 지도 Gazebo 검증 보고서

> 갱신: {datetime.now(timezone.utc).isoformat(timespec='seconds')}. 판정 범위는 시뮬레이션이다. 정규 조건 {len(rows)}/33건 종료. 미완료 조건은 성공으로 세지 않는다.

## 1. 환경과 검증 경계

Ubuntu 22.04.5, Intel i7-12700(20 logical CPU), RAM 32 GB. 기존 ROS2 Humble Docker 이미지와 Gazebo Classic 11.10.2를 사용했다. GPU 드라이버는 사용 불가이며 native Gazebo GUI·RViz는 software GL로 표시했다. 원래 작업 체크아웃을 보존하고 독립 체크아웃의 `lee/sim-real-maps`에서 작업했다. tip `44e9fc8`은 Jetson 미러 `e223b7532d98`을 포함한다.

P0는 `nav2_params_pre_wallpush_20260915.yaml`, P1은 현장 기본 `nav2_params.yaml`이다. P1 global inflation 1.0/scaling 2.0/planner multiplier 3.0을 그대로 사용했다. v011/v012 역시 원본 후보를 사용하고 v012만 gate 0.13 override를 적용한다. 현장 기본 파일과 map pins·latest·waypoints는 변경하지 않았다.

Fast DDS는 UDP 전용으로 격리된 Docker 네트워크를 사용한다. 빠른 자동 startup에서 lifecycle 응답 유실이 관측되어 시뮬 전용 새 시작 절차는 discovery 대기 후 localization → initial pose → navigation 순서로 원본 lifecycle manager의 STARTUP 서비스를 호출한다. 초기화 실패는 bag을 preflight로 보존하고 정규 반복에 포함하지 않는다.

## 2. 모델과 지도 정합

| 항목 | F1 A/B 월드 | F2 A/B 월드 |
|---|---|---|
| occupied 셀 | {geometry['F1']['occupied_cells']} | {geometry['F2']['occupied_cells']} |
| 행 run-length 병합 벽 | {geometry['F1']['merged_rectangles']} | {geometry['F2']['merged_rectangles']} |
| raster 복원 불일치 | 0 | 0 |
| 벽 모서리 수치 오차 m | {geometry['F1']['max_corner_error_m']:.3e} | {geometry['F2']['max_corner_error_m']:.3e} |
| 해상도 | 0.05 m | 0.05 m |

F1 월드는 보관함의 유리 고정문 선분만 제거하여 원본과 44셀이 다르고 다른 셀은 모두 동일하다. Nav2는 유리문이 포함된 원본 v3를 로드한다. F2 A/B 월드는 raw occupied 점과 잡음을 모두 유지한다. 벽은 높이 1.0 m 사각형이며 셀당 박스를 만들지 않았다.

사용자가 요청한 수동 승하차 리허설은 문틀과 기존 약 1.0 m 통로를 유지하며 두 sliding leaf의 열린 위치를 Gazebo GetEntityState로 검증한다. F2 리허설 전용 `f2_door_open.world`는 통로 중심 `(-11.925,-1.925)`, 축 각도 `0.741948 rad`, 폭 1.0 m·수직 범위 0.8 m의 마스크 안 occupied {geometry['F2_manual_door_open']['door_open_removed_cells']}셀만 제거한다. F2 문 영역의 스캔 두께를 비우는 시뮬 예외이며 실측 문짝 두께나 원본 지도 정합 검증으로 해석하지 않는다. 원본 pgm/yaml과 A/B 월드는 보존했다.

| 항목 | 원본/실차 기준 | 시뮬레이션 |
|---|---|---|
| 차체 수평 footprint | x -0.327..0.033 / y ±0.219 m | 동일 직사각형 collision |
| 바퀴 반경 | 0.033 m | 동일 |
| 실효 윤거 | 0.4323 m | 동일 |
| LiDAR frame·위치 | laser / URDF (-0.117,0,0.667) | 동일 |
| LiDAR 주기·표본 | 약 7.6 Hz / 약 1450 | 7.6 Hz / 1450 |
| 볼캐스터·보조 바퀴 | 원본 geometry | geometry 유지 |
| 마스트·carrier·laser collision | 원본 모델 | visual/inertia 유지, 충돌 제거 |
| 안전 gate | 현장 원본 코드·설정 | 동일 |
| OpenCR | 실제 펌웨어·모터 | 실제 bridge의 send tick·feedback parsing을 재사용한 serial shim |

shim은 timeout 0.5 s, 48 rpm 초과 시 비율 축소 없이 0과 ready false, 60 rpm/s slew를 적용한다. 지면 진실은 p3d의 world 기준 base_footprint pose이며 20 Hz로 기록한다. Nav2에는 지면 진실을 넣지 않는다.

## 3. 정규 시나리오 관측

최소 거리는 지면 진실의 비대칭 footprint 다각형과 occupied 셀을 병합한 벽 사각형 경계의 정확한 2D 거리다. 오프라인 경로 중심→occupied 셀 중심 0.552→0.886 m와 정의가 다르므로 수치 자체를 동일 비교하지 않는다. S1 코너는 `(-12.95,-5.25)` 반경 3 m다.

{chr(10).join(table)}

미완료 조건: {', '.join(missing) if missing else '없음'}.

P0/P1 결론은 각 조건 3회 결과가 모인 뒤 판정한다. 현재 데이터가 없는 조건이나 중간 리셋 실행으로 효과를 주장하지 않는다. S4 생성 조건(P1 S1/S3 거리 0.20 m 미만 또는 lethal-start) 해당 관측: {', '.join(p1_unsafe) if p1_unsafe else '현재 완료 데이터에서 없음; 미완료 조건 판정 대기'}.

### 3.1 S3 초기 pose 오차와 AMCL 회복

AMCL과 지면 진실의 각 message header 시각을 맞춰 비교했다. 지면 진실 표본 사이를 보간하며 0.15 s보다 큰 공백이나 범위 밖 AMCL 표본은 제외한다. clearance CSV의 캐시된 AMCL pose와 현재 truth를 직접 비교하지 않는다. 회복 기준은 초기 위치 오차 0.25 m 초과에서 마지막 AMCL 10표본 모두 0.25 m 이하로 내려간 경우다.

{chr(10).join(localization)}

## 4. 요청한 수동 배달 왕복

사용자의 추가 지정에 따라 실주행 배달 목적지는 엘리베이터 하차 후 우회전, 왼쪽 네 번째 방(캡처 맨 왼쪽 위)이며 `f2_delivery_left_room4`로 현장 등록한다. 정확한 정차 좌표·yaw는 해당 방 표식에서 확인한다. 현재 시뮬 목표와 A/B 조건, 원본 waypoint registry는 변경하지 않았다. 보관함의 사용자 정지 위치에서 이어 시작한 실행은 최초 위치부터의 무중단 완주와 구분한다.

| 실행 | 완주 | 기록된 단계 | 오류 |
|---|---|---|---|
{chr(10).join(missions) if missions else '| 진행 중 | 미판정 | 진행 중 | 최종 결과 대기 |'}

실제 WASD 노드에 PTY 입력을 넣고 Nav2 process를 종료한 뒤 수동 조종한다. 적재·하차는 수동 전진·후진·정지와 payload marker로 표시하며 팔 구동으로 판정하지 않는다. 층 이동은 수동 층 선택을 나타내는 명시적 월드 교체·캐빈 pose 이동이다. 절차와 현장 전환 명령은 [수동 배달 왕복 가이드](./2026-09-15_manual_delivery_roundtrip.md)에 있다.

## 5. S6 원본 운용 스크립트와 결함

| 리허설 | 원본 stop 직후 metadata | 보정 후 metadata | 원본 contract exit | 원본 analysis exit |
|---|---|---|---|---|
{chr(10).join(operational) if operational else '| 진행 대기 | 미판정 | 미판정 | 미판정 | 미판정 |'}

원본 map guard를 pre-nav/goal 단계에서 호출한다. Humble per-node 로그가 map_io 출력을 exit까지 버퍼링하는 경우에는 실제 launch stdout을 live map_server PID에 대응시켜 원본 guard의 `--log-dir`에 전달한다. pin·SHA·로드 경로 검사를 생략하지 않는다.

원본 `field_pose_capture.py`는 OccupancyGrid의 99/100 값을 raw lethal 253/254와 비교해 lethal false negative가 발생한다. Humble publisher의 값 변환은 [Nav2 원본 소스](https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/src/costmap_2d_publisher.cpp)에서 확인했다. 시뮬 전용 raw Costmap capture 보정안과 합성 lethal 재현 검사를 추가했다. 정차 시 원본 capture의 AMCL 표본 수 부족도 관측되었다. S3 전에는 nomotion update를 강제로 호출하지 않는다.

원본 `fieldctl record start/check` 통과 후 `record stop` 완료 출력에도 metadata가 없는 사례를 재현했다. 시뮬 전용 보정은 정확한 bag 출력 경로의 내부 recorder만 SIGINT하고 metadata·원본 contract·원본 분석까지 검증한다. Nav2 활성 주행에서도 원본 record 시작의 일반 topic list가 hidden action status를 발견하지 못해 원본 분석은 exit 1이다. 수동 구간만 녹화한 경우에도 같은 토픽이 없어 PASS로 처리하지 않는다. 별도 `record_rehearsal_sim.py`는 원본을 로컬 임시 복사해 topic list와 rosbag record 양쪽에 `--include-hidden-topics`를 넣고 지면 진실을 추가하며 내부 recorder부터 정지한 뒤 원본 contract와 분석을 실행한다.

`sim_s6_fixed_nav_active_v2` 보정안은 녹화 성장 24,576→4,718,592 byte, metadata 존재, 원본 wrapper·contract·분석 exit 0으로 통과했다. 원본 현장 운용 스크립트는 수정하지 않았다.

## 6. 검사와 산출물

기하·문 개구부 검사 7건 통과. raw Costmap lethal 합성 검사는 native ROS Humble에서 통과했고 ROS 없는 환경에서는 SKIP한다. 이식성은 현재 src 209파일 검사 통과이며 커밋 직전 다시 확인한다.

기존 PC Docker 오프라인 초기 확인은 PASS46/SKIP3/FAIL5다. 기존 맵 생성기 실행과 robot_arm_pkg 빌드, 하드웨어 없는 `/dev/null` read-only safe runtime 매핑으로 환경을 준비한 뒤 PASS48/SKIP3/FAIL3이었다. 남은 firmware·Nav2 launch·orthogonal tuning 3건은 원본 기대값과 현재 기본 설정의 불일치다. 원본 테스트 러너는 수정하지 않았다. Jetson Python 3.8 기준선과 환경이 달라 실패 개수만으로 회귀를 주장하지 않는다.

코드·작은 요약은 `sim/real_maps/`, bag·dense CSV·preflight 로그는 gitignore된 `logs/real_map_sim/`에 보관한다. 큰 bag·영상·민감정보를 외부로 공유하지 않는다. GitHub 업로드와 Jetson 반영은 정상 왕복과 보고서 검토 뒤 진행한다.

## 7. 실물에서만 검증할 사항

수동 지도의 절대 오차, 유리·반사·라이다 누락, 실제 AMCL 노이즈와 초기 정렬, 보조 바퀴·문턱, 모터 부하·제동·온도, 실제 펌웨어와 전원, 실제 문 제어·층 선택·적재 고정·팔 동작은 이번 시뮬레이션으로 검증할 수 없다. 실주행에서는 바닥 표식·실물 차체 위치·bag 근거로 별도 판정한다.
'''
    target=ROOT/'docs/sim/2026-09-15_real_map_gazebo_report.md'
    target.parent.mkdir(parents=True,exist_ok=True)
    target.write_text(content)
    print(target)


if __name__=='__main__':
    main()
