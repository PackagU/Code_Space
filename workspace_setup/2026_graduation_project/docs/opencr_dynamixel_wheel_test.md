# OpenCR + Dynamixel 2개 바퀴 구동 테스트 A to Z

> 목적: OpenCR 1.0에 Dynamixel 2개를 연결해 좌/우 바퀴를 저속으로 돌리고, 이후 `drive_pkg` 바퀴 제어 실기 테스트의 기준 절차로 쓴다.
> 기준일: 2026-05-18

## 0. 전제

이 문서는 **Protocol 2.0 X-series 계열**(예: XL430, XM430, XH540, XM540)을 1차 기준으로 쓴다. 프로젝트 후보인 XH540-W270도 여기에 해당한다.

AX-12, 일부 구형 MX처럼 Protocol 1.0 또는 구형 wheel mode를 쓰는 모델은 절차가 조금 다르다. 그런 모터를 쓰면 먼저 `DYNAMIXEL Wizard 2.0`에서 모델명, firmware, protocol, baudrate를 확인하고 이 문서의 "구형 모터일 때" 절을 적용한다.

## 1. 준비물

| 항목 | 권장 |
|------|------|
| 컨트롤러 | OpenCR 1.0 |
| 모터 | TTL 3핀 Dynamixel 2개, 같은 모델 권장 |
| 전원 | 모터 정격에 맞는 SMPS/배터리. XH/XM 계열은 보통 12V 계열 전원부터 테스트 |
| 케이블 | Dynamixel 3핀 TTL 케이블, USB Micro-B 케이블 |
| PC | Windows + Arduino IDE |
| 툴 | Arduino IDE, Dynamixel2Arduino 라이브러리, DYNAMIXEL Wizard 2.0 |

OpenCR은 USB 전원만으로 보드가 켜지지만, Dynamixel을 실제로 돌리려면 모터 전원이 필요하다. USB만 꽂고 모터가 안 돈다고 판단하면 안 된다.

## 2. 안전 체크

벤치 테스트에서는 바퀴를 바닥에 놓지 말고 공중에 띄운다. 바퀴가 지면을 물고 있으면 코드 실수 하나로 로봇이 튄다.

- 첫 테스트 속도는 `5 rpm` 이하.
- 전원 투입 전 극성 확인.
- Dynamixel 케이블은 전원 인가 중에 꽂거나 빼지 않는다.
- 모터가 뜨겁거나 냄새가 나면 바로 전원 차단.
- `E-Stop`이 없으면 SMPS 스위치나 멀티탭 스위치를 손 닿는 곳에 둔다.
- 바퀴축에 손가락, 케이블, 옷자락이 닿지 않게 한다.

## 3. 배선

### 3.1 TTL Dynamixel 3핀

OpenCR의 Dynamixel TTL 포트 3개 중 아무 포트에 연결해도 같은 버스다. 모터 2개는 직렬 daisy-chain으로 연결해도 되고, OpenCR의 TTL 포트 2개에 나눠 꽂아도 된다.

TTL 3핀 기준:

| 핀 | 의미 |
|----|------|
| 1 | GND |
| 2 | VDD |
| 3 | DATA |

주의할 점:

- TTL 모터는 OpenCR의 TTL 포트에 연결한다.
- RS-485 모터는 RS-485 포트에 연결한다. TTL 포트에 꽂으면 통신이 안 된다.
- 모터 2개는 **ID가 달라야** 한다. 이 문서는 왼쪽 `ID 1`, 오른쪽 `ID 2`를 기준으로 한다.

### 3.2 전원

1. OpenCR에 USB를 연결한다.
2. OpenCR의 배터리/SMPS 입력에 모터 정격 전원을 넣는다.
3. OpenCR 전원 스위치를 켠다.
4. OpenCR Power LED와 Dynamixel LED 점멸을 확인한다.

OpenCR 사양상 입력 전원은 5-24V 범위지만, 실제 전압은 사용하는 Dynamixel 정격에 맞춘다. 이 프로젝트의 주행용 모터 후보는 토크 요구가 크므로, USB 전원이나 작은 어댑터로 부하 테스트를 하면 안 된다.

## 4. ID와 baudrate 설정

테스트 전에 Dynamixel Wizard 2.0으로 각 모터를 하나씩 연결해 설정한다.

1. OpenCR에 `usb_to_dxl` 예제를 올리거나 U2D2를 사용한다.
2. Dynamixel Wizard 2.0에서 Scan.
3. 모터 1개만 연결한 상태로 ID를 `1`로 설정.
4. 다른 모터 1개만 연결한 상태로 ID를 `2`로 설정.
5. 두 모터 baudrate를 같은 값으로 맞춘다. 이 문서는 `57600 bps` 기준.
6. Protocol이 `2.0`인지 확인.

팀 기준값:

```text
LEFT_ID = 1
RIGHT_ID = 2
BAUDRATE = 57600
PROTOCOL = 2.0
MODE = Velocity Control
```

ID가 겹치면 둘 다 응답하지 않거나 이상하게 동시에 움직인다. ID 설정은 한 번에 모터 하나만 연결해서 한다.

## 5. Arduino IDE 설정

1. Arduino IDE 설치.
2. `File > Preferences > Additional Boards Manager URLs`에 OpenCR 보드 매니저 URL 추가:

```text
https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/arduino/opencr_release/package_opencr_index.json
```

3. `Tools > Board > Boards Manager`에서 `OpenCR` 설치.
4. `Tools > Board`에서 `OpenCR Board` 선택.
5. `Tools > Port`에서 OpenCR 포트 선택. Windows에서는 보통 `COMx`, Linux에서는 `/dev/ttyACM0`.
6. `Tools > Manage Libraries`에서 `Dynamixel2Arduino` 설치.

## 6. 1차 통신 확인 코드

먼저 모터가 보이는지 확인한다. 아래 코드를 OpenCR에 업로드하고 Serial Monitor를 `115200 baud`, `Newline`으로 연다.

```cpp
#include <Dynamixel2Arduino.h>

#if defined(ARDUINO_OpenCR)
  #define DXL_SERIAL Serial3
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 84;
#else
  #error "This sketch is intended for OpenCR."
#endif

const float DXL_PROTOCOL_VERSION = 2.0;
const uint32_t DXL_BAUDRATE = 57600;
const uint8_t LEFT_ID = 1;
const uint8_t RIGHT_ID = 2;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void setup() {
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL && millis() < 3000) {}

  dxl.begin(DXL_BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  DEBUG_SERIAL.println("Ping test start");
  DEBUG_SERIAL.print("LEFT  ID 1: ");
  DEBUG_SERIAL.println(dxl.ping(LEFT_ID) ? "OK" : "FAIL");
  DEBUG_SERIAL.print("RIGHT ID 2: ");
  DEBUG_SERIAL.println(dxl.ping(RIGHT_ID) ? "OK" : "FAIL");
}

void loop() {
}
```

둘 다 `OK`가 떠야 다음 단계로 간다. 하나라도 `FAIL`이면 배선, ID, baudrate, protocol부터 잡는다.

## 7. 바퀴 회전 테스트 코드

아래 코드는 양쪽 모터를 Velocity Control Mode로 바꾸고, Serial Monitor 명령으로 전진/후진/좌회전/우회전/정지를 테스트한다.

왼쪽/오른쪽 모터가 서로 마주 보게 장착되면 같은 rpm 명령을 넣어도 바퀴 진행 방향이 반대로 보일 수 있다. 이 문서는 `RIGHT_SIGN = -1`을 기본값으로 둔다. 실제 장착 방향이 다르면 `LEFT_SIGN`, `RIGHT_SIGN`만 바꾼다.

```cpp
#include <Dynamixel2Arduino.h>

#if defined(ARDUINO_OpenCR)
  #define DXL_SERIAL Serial3
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 84;
#else
  #error "This sketch is intended for OpenCR."
#endif

using namespace ControlTableItem;

const float DXL_PROTOCOL_VERSION = 2.0;
const uint32_t DXL_BAUDRATE = 57600;

const uint8_t LEFT_ID = 1;
const uint8_t RIGHT_ID = 2;

// 바퀴 장착 방향에 맞춰 조정한다.
const int LEFT_SIGN = 1;
const int RIGHT_SIGN = -1;

float base_rpm = 5.0;
const float MIN_RPM = 0.0;
const float MAX_RPM = 20.0;   // 벤치 테스트용 상한. 실차 전까지 올리지 않는다.

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

bool setupMotor(uint8_t id) {
  if (!dxl.ping(id)) {
    DEBUG_SERIAL.print("Ping failed: ID ");
    DEBUG_SERIAL.println(id);
    return false;
  }

  dxl.torqueOff(id);
  if (!dxl.setOperatingMode(id, OP_VELOCITY)) {
    DEBUG_SERIAL.print("Mode set failed: ID ");
    DEBUG_SERIAL.println(id);
    return false;
  }
  dxl.torqueOn(id);
  dxl.setGoalVelocity(id, 0.0, UNIT_RPM);
  return true;
}

void setWheelRPM(float left_rpm, float right_rpm) {
  dxl.setGoalVelocity(LEFT_ID, LEFT_SIGN * left_rpm, UNIT_RPM);
  dxl.setGoalVelocity(RIGHT_ID, RIGHT_SIGN * right_rpm, UNIT_RPM);
}

void stopWheels() {
  setWheelRPM(0.0, 0.0);
}

void printStatus() {
  float left_v = dxl.getPresentVelocity(LEFT_ID, UNIT_RPM);
  float right_v = dxl.getPresentVelocity(RIGHT_ID, UNIT_RPM);

  DEBUG_SERIAL.print("base_rpm=");
  DEBUG_SERIAL.print(base_rpm);
  DEBUG_SERIAL.print(" | present left=");
  DEBUG_SERIAL.print(left_v);
  DEBUG_SERIAL.print(" rpm, right=");
  DEBUG_SERIAL.print(right_v);
  DEBUG_SERIAL.println(" rpm");
}

void printHelp() {
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println("Commands:");
  DEBUG_SERIAL.println("  f : forward");
  DEBUG_SERIAL.println("  b : backward");
  DEBUG_SERIAL.println("  l : turn left");
  DEBUG_SERIAL.println("  r : turn right");
  DEBUG_SERIAL.println("  x : stop");
  DEBUG_SERIAL.println("  + : speed up");
  DEBUG_SERIAL.println("  - : speed down");
  DEBUG_SERIAL.println("  p : print velocity");
  DEBUG_SERIAL.println("  h : help");
  DEBUG_SERIAL.println();
}

void setup() {
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL && millis() < 3000) {}

  dxl.begin(DXL_BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  DEBUG_SERIAL.println("OpenCR 2-wheel Dynamixel velocity test");

  bool left_ok = setupMotor(LEFT_ID);
  bool right_ok = setupMotor(RIGHT_ID);

  if (!left_ok || !right_ok) {
    DEBUG_SERIAL.println("Motor setup failed. Check ID, baudrate, protocol, wiring, power.");
    while (true) {
      stopWheels();
      delay(1000);
    }
  }

  stopWheels();
  printHelp();
}

void loop() {
  if (!DEBUG_SERIAL.available()) {
    delay(20);
    return;
  }

  char c = DEBUG_SERIAL.read();

  switch (c) {
    case 'f':
      setWheelRPM(base_rpm, base_rpm);
      DEBUG_SERIAL.println("forward");
      break;
    case 'b':
      setWheelRPM(-base_rpm, -base_rpm);
      DEBUG_SERIAL.println("backward");
      break;
    case 'l':
      setWheelRPM(-base_rpm, base_rpm);
      DEBUG_SERIAL.println("turn left");
      break;
    case 'r':
      setWheelRPM(base_rpm, -base_rpm);
      DEBUG_SERIAL.println("turn right");
      break;
    case 'x':
      stopWheels();
      DEBUG_SERIAL.println("stop");
      break;
    case '+':
      base_rpm += 1.0;
      if (base_rpm > MAX_RPM) base_rpm = MAX_RPM;
      printStatus();
      break;
    case '-':
      base_rpm -= 1.0;
      if (base_rpm < MIN_RPM) base_rpm = MIN_RPM;
      printStatus();
      break;
    case 'p':
      printStatus();
      break;
    case 'h':
      printHelp();
      break;
    case '\n':
    case '\r':
      break;
    default:
      DEBUG_SERIAL.println("Unknown command. Type h.");
      break;
  }
}
```

## 8. 테스트 순서

### 8.1 바퀴를 분리한 상태

1. 모터 2개만 OpenCR에 연결.
2. 통신 확인 코드 업로드.
3. `ID 1 OK`, `ID 2 OK` 확인.
4. 회전 테스트 코드 업로드.
5. `p` 입력해서 현재 속도 출력 확인.
6. `f` 입력. 두 모터가 천천히 돈다.
7. `x` 입력. 둘 다 멈춘다.
8. `b`, `l`, `r`도 1초씩만 확인.

성공 기준:

- 통신 FAIL 없음.
- `x` 명령으로 1초 안에 정지.
- `p`에서 rpm 값이 0 근처로 돌아옴.
- 모터 LED가 에러 패턴으로 깜빡이지 않음.

### 8.2 바퀴 장착 후 공중 테스트

1. 본체를 받침대 위에 올려 바퀴가 지면에서 뜨게 한다.
2. `base_rpm = 5.0`으로 시작.
3. `f` 입력 시 양쪽 바퀴가 로봇 전진 방향으로 도는지 확인.
4. 한쪽이 반대면 코드의 `LEFT_SIGN` 또는 `RIGHT_SIGN`을 바꿔 다시 업로드.
5. `l`, `r` 입력 시 제자리 회전 방향이 맞는지 확인.

### 8.3 지면 저속 테스트

1. 로봇 주변 1m 이상 비운다.
2. 손으로 전원 스위치를 잡을 수 있는 위치에서 시작.
3. `f`를 0.5초 입력 후 `x`.
4. 직진이 틀어지면 기구 마찰, 타이어 접지, 좌우 rpm 보정값을 기록한다.
5. 이 단계에서는 `MAX_RPM`을 올리지 않는다.

## 9. 좌우 보정

같은 rpm을 줬는데 한쪽이 더 빠르면, 간단한 보정계수를 둔다.

```cpp
const float LEFT_GAIN = 1.00;
const float RIGHT_GAIN = 0.95;

void setWheelRPM(float left_rpm, float right_rpm) {
  dxl.setGoalVelocity(LEFT_ID, LEFT_SIGN * LEFT_GAIN * left_rpm, UNIT_RPM);
  dxl.setGoalVelocity(RIGHT_ID, RIGHT_SIGN * RIGHT_GAIN * right_rpm, UNIT_RPM);
}
```

이 보정은 임시값이다. ROS2 `cmd_vel` 연동 단계에서는 wheel radius, wheel separation, encoder/velocity feedback으로 다시 맞춘다.

## 10. `cmd_vel` 변환식

나중에 ROS2에서 `/cmd_vel`을 받을 때는 아래 식으로 좌우 바퀴 선속도를 만든다.

```text
v_left  = v - (omega * wheel_separation / 2)
v_right = v + (omega * wheel_separation / 2)
```

rpm 변환:

```text
wheel_rpm = wheel_linear_velocity / (2 * pi * wheel_radius) * 60
```

프로젝트 현재 가정:

```text
wheel_radius = 0.035 m   # 직경 7cm급
wheel_separation = 실측 필요
```

예시: 바퀴 반지름 0.035m에서 5 rpm은 바퀴 선속도 약 `0.018 m/s`다. 벤치 테스트에는 충분히 느린 속도다.

## 11. 구형 모터일 때

### 11.1 Protocol 1.0 AX/MX 계열

구형 AX/MX는 `OP_VELOCITY` 대신 angle limit을 0으로 두는 wheel mode 방식이 쓰일 수 있다. 이 경우:

- Protocol을 `1.0`으로 설정.
- 모델별 Control Table에서 `CW Angle Limit`, `CCW Angle Limit`, `Moving Speed` 주소를 확인.
- `DYNAMIXEL Wizard 2.0`에서 wheel mode 설정이 가능한지 먼저 확인.

혼용 금지:

- Protocol 1.0 모터와 Protocol 2.0 모터를 같은 테스트 코드에 섞지 않는다.
- baudrate가 다른 모터를 같은 버스에 섞지 않는다.

### 11.2 RS-485 모터

RS-485 모델은 OpenCR의 RS-485 포트를 사용한다. TTL 3핀 포트와 케이블이 다르다. 모터 모델명 끝이 `R` 계열이면 RS-485일 수 있으니 사양서를 먼저 본다.

## 12. 문제 해결

| 증상 | 원인 후보 | 조치 |
|------|-----------|------|
| `ping FAIL` | ID 불일치 | Wizard에서 ID 확인, 한 번에 모터 하나만 연결 |
| `ping FAIL` | baudrate 불일치 | Wizard에서 두 모터를 `57600`으로 맞춤 |
| `ping FAIL` | protocol 불일치 | X-series는 보통 2.0. 구형이면 1.0 확인 |
| `ping FAIL` | 전원 없음 | USB만 연결하지 말고 SMPS/배터리 연결 |
| 한쪽만 돈다 | ID 중복 또는 케이블 문제 | ID 재설정, 케이블 교체 |
| 전진 명령에서 한쪽이 반대로 돈다 | 좌우 모터 장착 방향 | `LEFT_SIGN`, `RIGHT_SIGN` 수정 |
| 움직이다 멈춘다 | 과전류/전압강하/통신 에러 | 속도 낮추기, 전원 용량 확인 |
| 모터 LED 에러 | overload, overheating, voltage error | 즉시 정지 후 Wizard에서 error 확인 |
| 업로드가 안 된다 | 포트 선택 오류 | Arduino IDE 포트 재선택, OpenCR reset |
| 보드가 멈춘다 | 잘못된 펌웨어 업로드 | OpenCR recovery mode 진입 후 재업로드 |

## 13. 테스트 로그 템플릿

테스트할 때 아래 형식으로 기록한다.

```markdown
## YYYY-MM-DD OpenCR + Dynamixel 2-wheel test

- 모터 모델:
- 전원:
- ID / baudrate / protocol:
- 바퀴 반지름:
- 코드 버전:
- 공중 테스트:
  - f:
  - b:
  - l:
  - r:
- 지면 테스트:
  - rpm:
  - 직진 편차:
  - 멈춤/발열/에러:
- 다음 조치:
```

## 14. 다음 단계

1. 이 문서로 모터 2개 공중 테스트 성공.
2. 바퀴 장착 후 `LEFT_SIGN`, `RIGHT_SIGN`, 좌우 gain 확정.
3. `wheel_radius`, `wheel_separation` 실측.
4. `src/drive_pkg/`에 OpenCR serial 명령 형식 설계.
5. ROS2 `cmd_vel -> wheel rpm` 변환 노드 작성.
6. OpenCR에서 rpm 피드백을 Jetson으로 보내 odom 초안 작성.

## 15. 공식 참고 자료

- ROBOTIS OpenCR 1.0 e-Manual: https://emanual.robotis.com/docs/en/parts/controller/opencr10/
- ROBOTIS Dynamixel2Arduino GitHub: https://github.com/ROBOTIS-GIT/Dynamixel2Arduino
- ROBOTIS Dynamixel Quick Start Guide: https://emanual.robotis.com/docs/en/dxl/dxl-quick-start-guide/
- `setOperatingMode()` API: https://emanual.robotis.com/docs/en/popup/arduino_api/setOperatingMode/
- `setGoalVelocity()` API: https://emanual.robotis.com/docs/en/popup/arduino_api/setGoalVelocity/
- `getPresentVelocity()` API: https://emanual.robotis.com/docs/en/popup/arduino_api/getPresentVelocity/
