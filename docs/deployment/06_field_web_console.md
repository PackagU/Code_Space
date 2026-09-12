# Field web console

기준일: 2026-09-12 KST

브라우저 한 화면에서 실시간 map, WASD 수동 주행, 매핑 시작·저장·종료, 저장 지도 Nav2 기동, 초기 위치와 목표 지점 지정을 수행한다. 로봇팔·리프트 노드는 시작하지 않는다. 현재 판정은 **오프라인 검증**이며 실차 주행은 H01 승인과 바퀴 들림 확인 뒤 수행한다.

## 1. 연결

노트북은 OpenCR에 직접 연결하지 않는다.

```text
노트북 브라우저 ── SSH tunnel/Wi-Fi ──> Jetson ── USB ──> OpenCR, LiDAR
```

Jetson에서 mapping용 컨테이너와 physical base를 먼저 시작한다. `ENABLE_DRIVE=1`은 실제 OpenCR를 열므로 현장 승인과 바퀴 들림 확인 뒤에만 사용한다.

```bash
cd /home/hsm/Code_Space
ENABLE_DRIVE=1 ./scripts/start_field_base.sh
```

별도 Jetson SSH 터미널에서 화면 서버를 시작한다.

```bash
cd /home/hsm/Code_Space
./scripts/start_field_web_ui.sh
```

노트북 터미널에서 SSH tunnel을 유지한다.

```bash
ssh -N -L 8080:127.0.0.1:8080 hsm@192.168.0.7
```

노트북 브라우저에서 `http://127.0.0.1:8080`을 연다. 서버는 Jetson의 loopback에만 바인딩하며 공개 LAN 포트로 열지 않는다. 화면별 임시 토큰이 없는 제어 요청은 거부한다.

## 2. 매핑과 수동 주행

상단의 LiDAR, odom, 구동 표시가 모두 정상일 때 `매핑 시작`을 누른다. 화면을 한 번 클릭한 뒤 WASD를 누르거나 주행 버튼을 마우스로 누른다. 키 또는 버튼을 놓으면 정지하며, 브라우저가 0.30초 동안 명령을 갱신하지 못해도 0속도를 발행한다.

긴 복도는 2D LiDAR 특징이 적어 먼저 5~10 m만 천천히 왕복한다. 벽이 한 줄로 겹치고 시작점 폐루프가 맞으면 `저장하고 끝내기`를 누른다. 이름을 비우면 시각 기반 이름을 자동 생성한다. 지도 YAML, PGM, posegraph, data, SHA256을 함께 저장하고 검증에 실패하면 매핑을 종료하지 않는다.

수동 텔레옵은 `[제안값] 0.10 m/s`, `[제안값] 0.35 rad/s`; OpenCR와 Jetson bridge의 wheel 상한은 사용자 지정 `[제안값] 30 rpm`이다.

## 3. 저장 지도 네비게이션

저장 지도 목록에서 방금 만든 지도를 선택하고 `지도 열기`를 누른다. 지도 도구를 `초기 위치`로 두고 실제 로봇 위치를 클릭한 다음 방향 각도를 입력해 적용한다. LiDAR 점이 지도 벽과 맞지 않으면 목표를 보내지 않는다.

지도 도구를 `목표 지점`으로 바꿔 같은 복도에서 0.5~1 m 앞을 클릭하고 적용한다. Nav2 현장 상한은 합성 wheel RPM이 30을 넘지 않도록 `[제안값] 0.05 m/s`, `[제안값] 0.20 rad/s`다. WASD 입력이 들어오면 화면이 보낸 Nav2 목표를 취소하고 수동 명령으로 전환한다.

## 4. 정지와 종료

상단 `소프트 정지`는 Nav2 목표를 취소하고 `/nav_safety/stop=true`와 0속도를 보낸다. 물리 E-Stop을 대신하지 않는다. 정지 해제는 같은 버튼을 다시 눌러 명시적으로 수행한다.

정상 종료는 네비게이션 종료, 소프트 정지, 화면 서버 Ctrl+C, base Ctrl+C 순서다. Wi-Fi 또는 SSH가 끊기면 브라우저 deadman, nav safety gate, OpenCR 500 ms watchdog이 각각 정지 경로를 가진다.
