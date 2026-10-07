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
