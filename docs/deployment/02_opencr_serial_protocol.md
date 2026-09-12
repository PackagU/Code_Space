# OpenCR Serial Protocol v0.2 (draft)

> Jetson(브리지) <-> OpenCR(펌웨어) 시리얼 계약. 상태: 코드 존재·오프라인 계약 검증, 실제 OpenCR 컴파일/업로드/바퀴 정지는 ⚠️미확인.
> 소비자: `src/drive_pkg/drive_pkg/opencr_protocol.py`, `src/drive_pkg/firmware/opencr/opencr_drive_bridge.ino`.
> 변경 절차: 이 문서 갱신 -> 양측 코드 갱신 -> 버전 문자열 동기.

## 1. 물리 계층

| 항목 | 값 |
|------|-----|
| 포트 | USB CDC (`/dev/opencr` udev 별칭) |
| 속도 | 115200 8N1 |
| 인코딩 | ASCII, 공백 구분, `\n` 종료 |

## 2. 명령 프레임 (Jetson -> OpenCR, 20Hz)

```text
V <left_rpm> <right_rpm>\n
```

- float, 소수 2자리. 양수 = 로봇 전진 방향 회전 (좌우 동일 규약)
- 부호 보정(모터 배선 반전)은 브리지의 `left_sign`/`right_sign` 파라미터가 담당.
  펌웨어는 받은 값을 그대로 모터에 적용한다.

## 3. 피드백 프레임 (OpenCR -> Jetson, 50Hz)

### 3.1 최소 프레임(v0.2-minimal, 내일 우선 시험)

```text
F <left_rpm> <right_rpm>\n
```

이 프레임은 Dynamixel의 실측 속도로 wheel odom만 만든다. 브리지는 `/imu`를 발행하지 않는다. 사용자가 구동 확인한 ID 1/2, Protocol 2.0, Dynamixel 1 Mbps 설정을 그대로 쓰는 펌웨어가 이 형식을 낸다.

### 3.2 전체 프레임(v0.2-full, IMU 통합 후)

```text
F <left_rpm> <right_rpm> <gx> <gy> <gz> <ax> <ay> <az> <qw> <qx> <qy> <qz>\n
```

| 필드 | 단위 | 의미 |
|------|------|------|
| left_rpm, right_rpm | rpm | 엔코더 실측 바퀴 속도 (전진 = 양수) |
| gx gy gz | rad/s | 자이로 |
| ax ay az | m/s^2 | 가속도 |
| qw qx qy qz | - | IMU 자세 쿼터니언 (정규화) |

## 4. 안전 규약 (watchdog)

- 펌웨어 코드: `V` 프레임 `[제안값]` 500ms 미수신 -> 모터 정지. 정적 계약 테스트는 통과했지만 실제 OpenCR 업로드와 물리 정지는 ⚠️미확인이다.
- 브리지: `/cmd_vel` `[제안값]` 500ms 미수신 -> `V 0.00 0.00` 송신 (이중 안전).
- 브리지는 첫 유효 `F` 피드백 전, 피드백 timeout, 시각 역행/큰 점프, 직렬 read/write 예외, 비유한·범위 밖 명령/피드백에서 `/drive/ready=false`와 0속도 상태로 간다.
- 정지 경로는 가속 제한을 우회해 즉시 0을 쓴다. 정상 명령만 `[제안값]` RPM 변화율 제한을 받는다.
- 종료 시 0속도 프레임을 3회 시도하지만, 케이블 단절·브리지 강제 종료에는 전달되지 않을 수 있다. 따라서 MCU 독립 watchdog과 물리 E-Stop을 대신하지 않는다.

`drive_calib.yaml`의 timeout·최대 선속도/각속도·RPM·RPM 변화율은 H01/H02 전 `[제안값]`이다. 실제 하중의 바퀴 들림 시험과 저속 지면 시험 전에는 확정값으로 쓰지 않는다.

## 5. 오류 입력과 준비 상태

| 입력/상태 | 브리지 동작 | 실물 한계 |
|---|---|---|
| NaN/Inf, 선·각속도 범위 초과 | 명령 폐기, 즉시 0, ready=false | MCU가 직전 명령을 유지하지 않는지는 watchdog 실측 필요 |
| NaN/Inf, 비정상 quaternion, RPM 범위 초과 피드백 | odom/imu 미갱신, ready=false | 최소 프레임은 quaternion 없이 odom만 발행 |
| 피드백 큐 적체 | 한 tick에서 읽은 최신 유효 프레임만 적분 | v0.1에는 sequence·센서 시각이 없어 오래된 프레임의 절대 나이는 판별 불가 |
| 호스트 시각 역행 또는 큰 dt | 해당 프레임 적분 안 함, ready=false | 다음 정상 프레임에서만 복구 |
| serial read/write 예외 | ready=false, 비영(非零) 성공으로 보고하지 않음 | 물리 모터 정지는 MCU/E-Stop 근거 필요 |

현재 v0.2 프레임에는 firmware version 응답의 강제 확인, sensor timestamp, sequence, checksum, MCU watchdog 상태 필드가 없다. v1.0 합의 시 추가하고 양측을 동시에 갱신해야 한다.

## 6. 부팅/에러 (선택 구현)

```text
HELLO opencr <fw_version>\n
E <code> <message...>\n
```

`HELLO`·`E`와 손상 프레임은 피드백 준비 상태를 해제한다. 이후 정상 `F` 프레임이 들어와야 ready가 복구된다. firmware version 문자열의 일치 검사는 v0.1에 아직 없다.

## 7. odom/IMU 발행 책임

- EKF를 쓰지 않는 현재 실기 구성에서는 bridge가 `odom -> base_footprint` TF의 단일 발행자다.
- SLAM Toolbox 또는 AMCL이 `map -> odom`을 담당하며 둘을 동시에 실행하지 않는다.
- 코드의 odom/IMU covariance는 센서 보정 전 `[제안값]`이다. H02에서 실제 직진·회전·정지 분산을 측정해 갱신한다.
