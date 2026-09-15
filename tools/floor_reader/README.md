# 층수 인식 코드

Jetson의 층 인식기를 보존했다. Python·OpenCV·NumPy로 카메라 ROI를 변환하고 등록 숫자 템플릿과 비교한다. 안정된 목표층 관측은 `TARGET_FLOOR_DETECTED` 이벤트로 제공한다.

- `app.py`: 카메라 입력·ROI 지정·템플릿 등록·층수 판정·화면 서버
- `test_reader.py`: 순수 판정·이벤트 오프라인 시험
- `data/`: 현재 ROI 설정과 등록 숫자 템플릿

층 이벤트 자체는 엘리베이터 문 열림·정지·하차 허가를 판정하지 않는다.
