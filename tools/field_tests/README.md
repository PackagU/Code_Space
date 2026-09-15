# 현장 주행 시험 스크립트

Jetson에 따로 있던 회전·RPM·주행 확인 스크립트를 보존했다. `rot360.py`는 회전·scan·odom 수집, `stand_test.py`·`stand48*.sh`는 RPM·주행 확인, `run4.sh`는 F3 Nav2 목표 시험, `sep_verify.sh`는 트랙폭 확인, `restart_fb56.sh`는 당시 피드백 설정의 기동 절차다.

이 스크립트에는 당시 장비 경로와 구동 명령이 들어 있다. 일반 오프라인 검사로 실행하지 않으며, 실제 구동은 별도 현장 승인 대상이다.
