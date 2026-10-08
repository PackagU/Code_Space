# 2026-10-08 Jetson SSH 읽기 전용 점검

작성일: 2026-10-08. 상태: **점검 완료(20:13 KST, 6절)**. 오늘 현장에서 SSH로 Jetson에 붙어 코드·설정·장치 상태를 읽기만 한다. 결과는 [10_unknowns_register.md](10_unknowns_register.md)의 `U-J` 항목에 채운다. 그 결과가 [08 데스크톱 Gazebo 자체 시험](08_desktop_gazebo_selftest_plan.md)의 기준선이 된다.

표기: `아직 모름` = 사실이지만 아직 확인하지 않음(AGENTS의 ⚠️미확인과 같은 뜻). `미정` = 아직 정하지 않은 결정. 수치는 `[측정값]`·`[제안값]`으로 나눈다.

## 0. 오늘 하지 않는 것

- 바퀴·팔·지게팔 구동, `fieldctl base start`, `resume`, goal 전송, 펌웨어 업로드.
- 시리얼 장치 열기(`/dev/opencr`·`/dev/arm_servo`에 `cat`·`screen`·Python 접속 포함). OpenCR은 USB를 열 때 reset 부작용이 있는지 아직 모른다.
- 노드·컨테이너 시작·정지·재생성, `colcon build`, `git pull`·`checkout`·`stash`·`reset`.
- `.env`·토큰·SSH 키·인증 파일 읽기. `git remote -v`도 쓰지 않는다. 원격 URL에 토큰이 들어 있을 수 있다. 이름만 보는 `git remote`를 쓴다.
- `docker inspect`로 환경변수(`.Config.Env`) 출력하기.

카메라를 여는 것(`floor_reader` 기동)은 구동은 아니지만 오늘 목적에 필요 없다. 이미 떠 있는 앱이 있으면 포트·프로세스만 본다.

## 1. 접속

PC(Windows Git Bash)에서:

```bash
ping -n 2 192.168.0.7
```

```bash
ssh -o ConnectTimeout=8 hsm@192.168.0.7 'hostname; hostname -I; uptime'
```

- 평소 주소는 `192.168.0.7`, 자율주행 시험 Wi-Fi는 `10.141.228.26`이다. 2026-10-07에는 두 주소 모두 응답이 없었다.
- 안 붙으면 순서대로 본다: Jetson 전원·LED → PC와 Jetson이 같은 망인지 → Jetson에 모니터·키보드를 연결해 `hostname -I`로 주소 확인 → 새 주소로 다시 시도. 주소가 바뀌었으면 결과표에 적는다.
- 끝내 안 붙으면 08 문서는 GitHub `2026-10-06`(`43872e8`) 파일을 기준선으로 쓰고, "Jetson과 같은지 아직 모름"을 모든 시뮬 결과에 붙인다.

## 2. 한 번에 수집하기

읽기 전용 수집 스크립트 [tools/jetson_readonly_snapshot.sh](tools/jetson_readonly_snapshot.sh)를 PC에서 SSH 표준입력으로 넘긴다. Jetson에 파일을 만들지 않는다. 결과는 PC에 저장된다.

```bash
mkdir -p logs/jetson_snapshot && ssh -o ConnectTimeout=8 hsm@192.168.0.7 'bash -s' < plan/final_demo/tools/jetson_readonly_snapshot.sh | tee "logs/jetson_snapshot/$(date +%Y%m%d_%H%M)_snapshot.txt"
```

스크립트가 하는 일(모두 읽기):

| 구역 | 내용 |
|---|---|
| A 기본 | 시각·시간 동기, 주소, 가동 시간, 디스크·메모리 |
| B 컨테이너 | `docker ps -a`, `ros2_humble`의 mount·device 목록(환경변수 제외) |
| C Git | HEAD, 브랜치, 최근 커밋 8개, 미커밋 파일 수와 목록 앞부분, stash 수, 원격 이름 |
| D 핵심 파일 SHA256 | Nav2·안전 게이트·구동 보정 설정, 지점·지도 핀·지도 포인터, F1 v3·F2 지도, `camera_views.json` |
| E 2026-10-06 흔적 | 이성덕 브랜치 파일이 있는지(`floor_arrival_probe.py`, `elevator_camera.py` 등) |
| F 장치 | `/dev` 별칭·`/dev/video*`·`/dev/serial/by-id`·`lsusb` (열지 않음) |
| G 프로세스·포트 | 호스트·컨테이너의 ROS·Python 프로세스, 8765 등 포트 수신 여부 |
| H 빌드 | `/ros2_ws/install` 패키지 목록과 시각, 컨테이너 Python 버전 |
| I 기록 | `~/Code_Space/logs`에서 9/17 이후 생긴 폴더 |
| J 팔 시작 동작 | 팔 노드 소스의 home·torque 관련 줄(코드 읽기만) |
| K 앱 데이터 | 버튼 앱·층수 인식기 `data/` 파일 목록과 SHA256 |
| L GitHub 도달 | `git ls-remote`로 원격 브랜치 머리만 조회(프롬프트 끔, 8 s 제한) |

ROS 노드가 떠 있지 않으면 `ros2 node list`는 빈 결과가 정상이다. 수집 도중 한 항목이 실패해도 다음 항목으로 넘어간다.

`fieldctl diagnose`는 이 브랜치 소스 기준으로 상태 조회만 한다(`scripts/fieldctl` 109행). Jetson 쪽 `fieldctl`이 같은 내용인지는 아직 모른다. 쓰려면 먼저 `grep -n -A12 '^diagnose()' ~/Code_Space/scripts/fieldctl`로 읽고 실행한다.

## 3. 결과 해석표

기준 해시는 GitHub blob 바이트로 2026-10-08에 계산한 SHA256 앞 12자리다. 결과 열은 오늘 채운다. Windows checkout은 줄바꿈 변환 때문에 같은 파일이라도 해시가 다르게 나온다(이 PC clone에서 `waypoints.json`·`camera_views.json` 확인, CR 제거 시 기준 해시와 일치). Jetson·데스크톱 같은 리눅스 checkout에서 비교한다.

| 파일 | 기준 해시와 뜻 | 결과 |
|---|---|---|
| `src/slam_pkg/config/nav2_params.yaml` | `e8cf213b33fb` = 원본(9/17 철회 반영). `80787840cd76` = wall_push(GitHub `9c5ee7e`·`2026-10-06` 모두 이 값). 다른 값 = 새 변경, diff 필요 | 아직 모름 |
| `src/drive_pkg/config/nav_safety.yaml` | `f2478064f359` (`2026-10-06`) | 아직 모름 |
| `src/drive_pkg/config/drive_calib.yaml` | `4248deb19979` | 아직 모름 |
| `src/slam_pkg/maps/field/waypoints.json` | `9694225dca40` (지점 13개) | 아직 모름 |
| `src/slam_pkg/maps/field/map_pins.json` | `eb5aaa2976b5` | 아직 모름 |
| `f1/latest_map.txt` · `f2/latest_map.txt` | `4c9e663899d2`(→ `f1_manual_clean_v3.yaml`) · `ecd9754248c4`(→ `f2_nav_unknown_v1.yaml`) | 아직 모름 |
| `f1/f1_manual_clean_v3.yaml` · `.pgm` | `d701d2b6df57` · `d525759b07a8` | 아직 모름 |
| `f2/f2_nav_unknown_v1.yaml` · `f2_raw_20260914.pgm` | `b670cd2b8790` · `f89bfeb9f928` | 아직 모름 |
| `src/robot_arm_pkg/config/camera_views.json` | 없음 = `2026-10-06` 미반영. `e30cb25651ca` = `79fdcb2`판(floor_view 1100/1700). `258c7e269066` = `c9618a4`판(1400/1750). `8d633d8ad2dd` = `0b2feca` 이후판(1480/1670) | 아직 모름 |

Git HEAD 해석:

| HEAD | 뜻 |
|---|---|
| `e223b75…` | 9/15 20:43 미러 시점에서 커밋 변화 없음. 미커밋 변경은 따로 본다 |
| `9c5ee7e…` | 9/16 발표 정리본 |
| `79fdcb2…` ~ `43872e8…` | 이성덕 `2026-10-06` 브랜치 반영 |
| 그 밖 | Jetson에서 직접 커밋했거나 다른 브랜치. 커밋 목록을 기록하고 GitHub에 있는지 확인 |

## 4. 데스크톱으로 가져갈 것

Jetson에 파일을 만들지 않고 PC로 스트림 복사한다. 지도·설정 폴더에 비밀 파일이 없는지 2절 결과의 목록으로 먼저 본다.

```bash
ssh hsm@192.168.0.7 'cd ~/Code_Space && tar czf - --exclude=".env" --exclude="*.posegraph" --exclude="*.data" src/slam_pkg/maps/field src/slam_pkg/config src/drive_pkg/config src/robot_arm_pkg/config src/common_pkg/urdf' > "logs/jetson_snapshot/$(date +%Y%m%d_%H%M)_cfg.tgz"
```

```bash
ssh hsm@192.168.0.7 'cd ~/Code_Space && git diff HEAD' > "logs/jetson_snapshot/$(date +%Y%m%d_%H%M)_diff.patch"
```

- 미커밋 변경이 있으면 위 patch로 담긴다. untracked 파일은 2절 C 구역 목록을 보고 필요한 것만 따로 고른다.
- 백업용 `git bundle`(05 T1)은 Jetson 디스크에 파일을 만든다. 오늘 할지는 미정이다. 토요일 T1에서 해도 된다.
- 데스크톱이 Jetson에 직접 닿는지는 아직 모른다. 안 닿으면 이 PC에서 USB나 GitHub 비공개 저장소를 거쳐 옮긴다. 어느 쪽으로 옮길지는 미정이다.

## 5. 끝나고 갱신할 문서

| 결과 | 갱신 |
|---|---|
| 접속 주소, HEAD, 핵심 해시, 장치, 프로세스 | [10_unknowns_register.md](10_unknowns_register.md) `U-J01`~`U-J13` |
| `nav2_params.yaml` 해시 | [TODO.md](../../TODO.md) D6, [05 T1](05_weekend_field_test_plan.md) |
| `camera_views.json`·`2026-10-06` 반영 여부 | TODO "지금 상태", 05 T1·T2 |
| Jetson과 GitHub의 차이 | [08 1절 기준선 표](08_desktop_gazebo_selftest_plan.md) |
| 9/17 이후 실행 기록 | [02 문서](02_code_review_2026-10-06.md) 근거 보강 여부 판단 |

오늘 실물 동작은 없으므로 결과는 모두 "Jetson 파일·프로세스 상태 확인"까지만 판정한다. 실물 검증으로 적지 않는다.

## 6. 2026-10-08 점검 결과 (20:13 KST)

원본 출력: `logs/jetson_snapshot/20261008_2013_snapshot.txt`(이 PC). 실물 동작은 없었다. 아래는 파일·프로세스 상태 확인까지다.

| 항목 | 결과 |
|---|---|
| 접속 | `192.168.0.7` 키 인증 성공. 점검 시각 부팅 11분 뒤 |
| 시간·자원 | `System clock synchronized: yes`, 디스크 여유 83 G, 메모리 6.7 Gi(가용 4.4 Gi) |
| 컨테이너 | `ros2_humble` **꺼짐**(Exited 255, 재부팅 뒤 자동 시작 안 됨). 표시된 "26 years ago"는 시계 문제로 보인다(추정). `packagu_camera_test`(`humble-jetson-p02`) Exited 137, 24시간 전 |
| mount·device | `~/Code_Space/{src,scripts,test_workspace,logs,src/slam_pkg/maps}` → `/ros2_ws/...`. `/dev/arm_servo`·`/dev/motor_nano`는 `/dev/null`로 연결된 안전 프로파일 |
| Git | HEAD `c83d092cc`(`lee/hw-design-review`), 미커밋·untracked 167개. gitignore·백업·지도 바이너리를 뺀 작업 트리 350개 파일이 GitHub `codex/presentation-20260916`(`9c5ee7e`) 코드와 같다(README·TODO만 다름) |
| GitHub 미러 | 9/15 미러 `e223b75` 대비 `scripts/start_field_base.sh`·`waypoints.json` 2개가 달랐다 → `lee/jetson-live` `6026e79`로 반영(fast-forward, blob id 대조) |
| `nav2_params.yaml` | `80787840cd76` = **wall_push 상태. 9/17 철회는 Jetson에 아직 적용되지 않았다**(D6) |
| 다른 핵심 파일 | `nav_safety`·`drive_calib`·`waypoints`·`map_pins`·`latest_map` 2개·F1 v3·F2 지도 모두 3절 기준 해시와 같다 |
| 이성덕 코드 | `~/Code_Space`에는 없다(`camera_views.json` 없음). 별도 폴더 `~/Code_Space-2026-10-06`가 GitHub `2026-10-06` `43872e8` 그대로인 git checkout이다. 바뀐 것은 `tools/floor_reader/data/config.json`의 ROI뿐이고 `events.jsonl`이 새로 생겼다. `install/robot_arm_pkg`가 10/7 18:19에 빌드돼 있다 |
| 장치 | `/dev/opencr` → `ttyACM0`(OpenCR 연결됨). `/dev/rplidar`·`/dev/arm_servo` 없음(CP210x·CH340 미연결). `/dev/video0`·`/dev/video1` 있음, `lsusb`에 GEMBIRD `1908:2310`(USB 카메라로 추정, 종류 확정은 아직 모름) |
| 프로세스·포트 | 호스트에서 옛 층수 인식기 `~/floor_reader/app.py --host 0.0.0.0` 실행 중(8765 수신, 부팅 뒤 자동 시작으로 보임). 컨테이너가 꺼져 ROS 노드 없음. 호스트에 `curl` 없음 |
| 기록 | `~/Code_Space/logs` 556 M. 9/17 이후 새 기록 폴더 없음. 가장 큰 bag은 `field_bags/manual_roundtrip_test_f2_20260916_034617` 384 M |
| 팔 노드 기본값 | `~/Code_Space`판도 `serial_port ""`·`home_on_start False`·`simulation_mode False`(35~38행) |
| 앱 데이터 | `~/button_arm_test_20260915/data`(9/15~16, 템플릿 12개), `~/floor_reader/data`(9/8) |
| Jetson의 GitHub 접근 | 아직 모름. 원격 URL 확인은 인증 정보와 가까워 하지 않았다 |

이 결과로 [10 목록](10_unknowns_register.md) `U-J`와 [08 1절](08_desktop_gazebo_selftest_plan.md)을 갱신했다. 데스크톱용 GitHub 브랜치는 `PackagU/Code_Space` `lee/sim-e2e-20261008`이다.
