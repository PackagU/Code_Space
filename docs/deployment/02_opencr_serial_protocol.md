# OpenCR Serial Protocol v0.1 (draft)

> Jetson(브리지) <-> OpenCR(펌웨어) 시리얼 계약. 상태: v0.1 초안 — Han 합의 후 v1.0 승격.
> 소비자: `src/drive_pkg/drive_pkg/opencr_protocol.py`(Lee), OpenCR 펌웨어(Han).
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

- 펌웨어: `V` 프레임 500ms 미수신 -> 모터 정지 (필수, Han)
- 브리지: `/cmd_vel` 500ms 미수신 -> `V 0.00 0.00` 송신 (이중 안전)

## 5. 부팅/에러 (선택 구현)

```text
HELLO opencr <fw_version>\n
E <code> <message...>\n
```

브리지는 `V`/`F` 이외 라인을 로그만 남기고 무시한다 (전방 호환).
