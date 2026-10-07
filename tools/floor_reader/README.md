# 층수 인식 코드

Jetson의 층 인식기를 보존했다. Python·OpenCV·NumPy로 카메라 ROI를 변환하고 등록 숫자 템플릿과 비교한다. 안정된 목표층 관측은 `TARGET_FLOOR_DETECTED` 이벤트로 제공한다.

- `app.py`: 카메라 입력·ROI 지정·템플릿 등록·층수 판정·화면 서버
- `test_reader.py`: 순수 판정·이벤트 오프라인 시험
- `data/`: 현재 ROI 설정과 등록 숫자 템플릿

층 이벤트 자체는 엘리베이터 문 열림·정지·하차 허가를 판정하지 않는다.

실제 미션의 목표층 조건을 주행 없이 확인하려면 프로젝트 루트에서
`python3 scripts/floor_arrival_probe.py --target 4`를 실행한다.
현재 영상 5개에서 연속으로 4층을 확인해야 `exit_condition_met=true`가 된다.
다른 층·인식 불가·영상 끊김에는 false로 돌아가며, 과거 감지 이벤트를 사용하지 않는다.
`motion_authorized`는 항상 false다. 실행 순서는
[4층 판정 시험](../../docs/deployment/08_elevator_camera_mission.md#4층-주행-없는-판정-시험)에 있다.

## 포트 충돌과 웹캠 확인

`Address already in use`는 8765 포트를 다른 프로그램이 사용하는 상태다.
호스트 터미널에서 기존 프로그램과 API 응답을 확인한다.

```bash
sudo ss -ltnp 'sport = :8765'
curl -sS --max-time 3 http://127.0.0.1:8765/api/state
pgrep -af 'floor_reader/app.py|button_arm_test/app.py'
```

기존 층수 인식기의 영상이 갱신되고 있다면 그 서버를 그대로 사용한다.
재시작할 때는 해당 앱을 실행한 터미널에서 Ctrl+C로 종료한 뒤 한 번만 실행한다.
같은 USB 웹캠을 `button_arm_test`와 함께 열지 않는다.
USB 웹캠은 `--source /dev/video0`으로 V4L2를 사용하고,
`--source jetson`은 CSI 카메라의 GStreamer 파이프라인을 사용한다.
서버가 포트를 확보하지 못하면 카메라를 열지 않고 종료한다.
