# 실측 지도 배달 왕복 — 현장 순서 가이드

> 위에서부터 순서대로 따라 하면 되는 실주행 절차다. 주행은 Nav2와 WASD 수동 조종을 번갈아 쓴다. 택배 상하차는 지게팔 터미널, 엘리베이터 버튼은 팔 버튼 앱으로 누른다.
> 명령과 포트는 2026-09-16 01:40 KST에 Jetson(`hsm@10.141.228.26`)에서 직접 확인했다. 확인 대상은 `scripts/lift_uno_position_terminal.py`, `scripts/fieldctl`, `~/button_arm_test_20260915/app.py`와 시리얼 포트 목록이다. 이 가이드 작성 중에는 로봇·팔·지게팔을 움직이지 않았다.

## 0. 먼저 알아둘 것

**터미널 배치.** 모든 창은 `ssh hsm@10.141.228.26`으로 접속한다. 평소 주소는 `hsm@192.168.0.7`이다. 9/16 새벽에는 192.168.0.7이 응답하지 않았다. Jetson 창은 모두 **호스트 셸**이고 컨테이너 안이 아니다.

| 창 | 용도                                           | 시작 위치                      | 켜 두는 기간                          |
| -- | ---------------------------------------------- | ------------------------------ | ------------------------------------- |
| A  | base                                           | `~/Code_Space`               | 처음부터 끝까지                       |
| B  | Nav2 시작                                      | `~/Code_Space`               | Nav2 구간마다                         |
| C  | pose·goal·teleop·record                     | `~/Code_Space`               | 처음부터 끝까지                       |
| D  | `stop`·`resume`·`nav stop`·`status` | `~/Code_Space`               | 처음부터 끝까지                       |
| E  | 지게팔(Uno) 터미널                             | `~/Code_Space`               | **처음부터 끝까지 닫지 않는다** |
| F  | 팔 버튼 앱                                     | `~/button_arm_test_20260915` | 처음부터 끝까지                       |
| W  | Windows PowerShell 터널 + 브라우저             | 노트북                         | 처음부터 끝까지                       |

**꼭 지킬 규칙.**

- Nav2와 teleop을 동시에 켜지 않는다. `fieldctl teleop`은 Nav2가 실행 중이면 거부한다.
- 지게팔 `u`/`d`와 팔 누르기는 차체가 멈춘 상태에서만 한다. 명령 전에 D에서 `./scripts/fieldctl stop`을 입력한다.
- **E 창을 닫지 않는다.** USB를 다시 열면 Uno가 리셋되어 위치 기준이 사라진다. 그러면 `u`/`d`가 거부된다.
- `goal`은 결과가 나올 때까지 C를 점유한다. 도착 뒤 정지 명령은 D에서 입력한다.
- `resume`은 이전 goal을 이어 가지 않는다. 다음 goal을 직접 보낸다.
- 층을 바꾸면 반드시 그 층 지도와 그 좌표계의 pose를 쓴다.

**비상 시.**

| 상황           | 입력 위치 | 입력                                                     |
| -------------- | --------- | -------------------------------------------------------- |
| 차체 정지      | D         | `./scripts/fieldctl stop`                              |
| teleop 중 정지 | C         | `k` 또는 Space, 손을 떼면 0.5초 뒤 deadman 정지        |
| 지게팔 정지    | E         | `!`                                                    |
| 팔 정지        | 브라우저  | `중지` 버튼, 또는 F에서 `Ctrl+C`                     |
| 전체           | 로봇      | 물리 비상정지. 프로그램 정지 버튼은 이를 대체하지 않는다 |

지게팔을 `!`로 멈추면 위치가 중간값이 되어 `u`/`d`가 모두 거부된다. 포크가 눈으로 보아 완전히 아래 끝이면 `b`, 위 끝이면 `t`로 다시 기준을 잡는다. 중간이면 시험을 멈추고 사람이 복구한다. 팔을 멈춘 뒤에는 자세를 알 수 없다. 브라우저 `위치 읽기` 또는 `home`부터 다시 한다.

**미확인 사항.** 아래는 이번 주행에서 관찰해 기록할 항목이다.

- 팔 버튼 앱의 실물 누름 정확도와 실제 눌림 판정은 검증되지 않았다. 보정 데이터는 보정점 2개(메뉴 6·7 자세)와 ▲·▼ 자세 각 1개뿐이다. 층 버튼 `1`·`4`는 두 보정점을 잇는 직선 위에서만 보간된다. 버튼이 눌렸는지는 사람이 버튼 불빛으로 확인한다.
- 지게팔 위치는 소프트웨어 펄스 계수다. 리미트 스위치가 없고 방향·스텝 손실은 실물로 검증되지 않았다. 한 번 이동은 166,400펄스이며 약 35초 걸린다 `[계산값]`.
- 원본 `pose capture`에는 costmap 값 검사 결함이 있다. 원본 `record stop`에는 종료 전달 결함이 있다. 각 단계의 주의 문구를 따른다.
- 팔 앱의 층 규칙은 **내부 올라감 = `4` 버튼, 내부 내려감 = `1` 버튼**으로 고정되어 있다. 실제 목적층 버튼이 이와 다르면 앱으로 누르지 않는다. 손으로 누르고 수동 보조로 기록한다.

## 1. 기동

### 1-1. base와 Nav2 (A, B)

A에서 base를 켠다.

```bash
cd ~/Code_Space
./scripts/fieldctl base start --drive
```

B에서 F1 Nav2를 켠다. 로그에 `Managed nodes are active`가 나올 때까지 기다린다.

```bash
cd ~/Code_Space
./scripts/fieldctl nav start F1
```

### 1-2. 지게팔 터미널 (E)

포트 목록을 본다. 이 명령은 포트를 열지 않는다.

```bash
cd ~/Code_Space
python3 scripts/lift_uno_position_terminal.py --list
```

9/16 확인값은 아래와 같다. 지게팔 Uno 후보는 **FT232R**(`0403:6001`) 하나뿐이다.

```
/dev/ttyUSB1 1a86:7523  → 팔 (/dev/arm_servo)
/dev/ttyUSB2 0403:6001  → FT232R, 지게팔 Uno 후보
/dev/ttyUSB0 10c4:ea60  → LiDAR (스크립트가 거부함)
/dev/ttyACM0 0483:5740  → OpenCR
```

포크가 **완전히 아래 끝**인지 눈으로 확인한 뒤 연다. `ttyUSB` 번호는 바뀔 수 있으므로 by-id 경로를 쓴다. FT232R은 스크립트가 아는 Arduino VID가 아니라서 `--allow-unknown`이 필요하다.

```bash
python3 scripts/lift_uno_position_terminal.py --execute --allow-unknown --port /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0
```

열리면 2초 뒤 자동으로 `s`를 보낸다. 다음 줄이 나와야 맞는 보드다.

```
UNO> STATUS moving=0 direction=NONE position_pulses=0 reference=REQUIRED ... move_size=166400
```

`UNO> STATUS` 줄이 없거나 `move_size=166400`이 아니면 다른 보드다. `q`로 나가고 진행하지 않는다. 맞으면 아래 끝 기준을 잡는다.

```
lift> b
```

`BOTTOM_REFERENCE_CONFIRMED position_pulses=0`이 나오면 된다. **이 창은 끝날 때까지 그대로 둔다.**

### 1-3. 팔 버튼 앱 (F, W)

`/dev/arm_servo`를 다른 프로그램이 쓰고 있지 않은지 확인한다. `servo_test.py`, `arm_servo_menu.py`, ROS arm 노드가 해당된다. 팔 주변에 사람이 없는지도 확인한다.

```bash
cd ~/button_arm_test_20260915
python3 -B app.py --execute --field-approved
```

네 관절 위치를 읽은 뒤 `주변을 확인하고 Enter를 누르면 home으로 이동합니다.`가 나온다. 주변을 확인하고 Enter를 누른다. 팔이 3초 동안 home으로 간 뒤 웹 서버가 열린다. 위치 읽기나 home이 실패하면 앱이 스스로 종료한다. 이때는 원인을 확인하기 전까지 진행하지 않는다.

노트북 PowerShell(W)에서 터널을 연다. 이 창도 끝까지 둔다.

```powershell
ssh -N -L 8091:127.0.0.1:8091 hsm@10.141.228.26
```

브라우저에서 `http://127.0.0.1:8091`을 열고 카메라 영상이 나오는지 본다. USB 웹캠을 다시 꽂았다면 F에서 앱을 재시작한다.

## 2. 초기 위치 → F1 택배 보관함 (Nav2)

로봇을 최초 바닥 표식에 놓는다. C에서 아래를 차례로 입력한다. 아래 pose는 등록된 `f1_initial_test` 좌표다. 표식 위치가 다르면 실측값으로 바꾸고, 부록 A에 따라 waypoint도 교체한다. 옛 `f1_idle`은 사용하지 않는다.

```bash
cd ~/Code_Space
./scripts/fieldctl stop
./scripts/fieldctl pose set -4.518 2.179 -1.701
./scripts/fieldctl map guard F1
./scripts/fieldctl pose capture initial_check F1 --duration 10 --floor-mark F1_INITIAL
./scripts/fieldctl record start manual_roundtrip_test
./scripts/fieldctl record check
./scripts/fieldctl resume
./scripts/fieldctl goal f1_locker F1
```

> **pose capture 판정.** RViz에서 scan과 벽이 맞는지 눈으로 확인한다. FAIL 사유가 AMCL 표본 부족뿐이면 정합을 확인하고 진행한다. **costmap footprint 항목 FAIL이면 goal을 보내지 않는다.** 이 판정은 이후 모든 `pose capture`에 똑같이 적용한다.

**완료 확인:** goal 성공, 보관함 앞 실제 정차.

## 3. 택배 상차 (수동 WASD + 지게팔)

### 3-1. 수동 전환

D에서 Nav2를 끄고 teleop을 허용한다. `status`에서 `navigation=stopped`를 확인한다.

```bash
./scripts/fieldctl stop
./scripts/fieldctl nav stop
./scripts/fieldctl status
./scripts/fieldctl resume
```

C에서 teleop을 켠다. `/controller_server already owns /cmd_vel`로 거부되면 5초 뒤 다시 실행한다.

```bash
./scripts/fieldctl teleop
```

```
w: 전진 / s: 후진 / a: 좌회전 / d: 우회전
k 또는 Space: 정지 / Ctrl+C: teleop 종료
q/z: 선속도 10% 증감(상한 0.10 m/s) / e/c: 각속도 10% 증감(상한 0.35 rad/s)
키를 누르고 있어야 움직이며 입력이 0.5초 끊기면 deadman 정지
```

### 3-2. 포크를 택배 아래로 넣기

`z`로 속도를 낮춘다. 포크가 택배 아래에 완전히 들어가도록 천천히 접근한다. 멈출 위치에서 C에 `k`, `Ctrl+C`를 차례로 누르고, D에서 정지를 건다.

```bash
./scripts/fieldctl stop
```

### 3-3. 들어 올리기 (E)

```
lift> s
```

`position_pulses=0 reference=CONFIRMED`인지 확인한다. 주변을 확인하고 올린다.

```
lift> u
```

`UP start` 뒤 약 35초 기다린다. `UP complete`와 `position_pulses=166400`이 나오면 끝이다. 올라가는 중 걸림이나 이상 소리가 나면 즉시 `!`를 누른다. 택배가 포크 위에 안정적으로 올라갔는지 눈으로 확인한다.

### 3-4. 보관함에서 빠져나오기

D에서 `resume`, C에서 teleop을 켠다. 택배가 보관함에 걸리지 않게 천천히 빠진다. 공간이 확보되면 `k`, `Ctrl+C`를 누르고 D에서 정지한다.

```bash
./scripts/fieldctl resume
```

```bash
./scripts/fieldctl teleop
```

```bash
./scripts/fieldctl stop
```

**완료 확인:** 차체 정지, 택배 적재, teleop 종료.

## 4. 보관함 → F1 엘리베이터 앞 (Nav2)

B에서 Nav2를 켜고 `Managed nodes are active`를 기다린다.

```bash
./scripts/fieldctl nav start F1
```

C에서 아래를 입력한다. 아래 pose는 보관함 waypoint 값이다. 3-4에서 위치가 크게 달라졌다면 실제 정차 좌표로 바꾼다.

```bash
./scripts/fieldctl pose set -5.125 -8.625 -1.69
./scripts/fieldctl map guard F1
./scripts/fieldctl pose capture locker_depart F1 --duration 10 --floor-mark F1_LOCKER
./scripts/fieldctl record check
./scripts/fieldctl resume
./scripts/fieldctl goal f1_elevator_entry F1
```

**완료 확인:** goal 성공, 엘리베이터 문 앞 정차.

## 5. F1 호출 버튼 ▲ 누르기 (팔)

### 5-1. Nav2 끄기

D에서 입력하고 `navigation=stopped`를 확인한다.

```bash
./scripts/fieldctl stop
./scripts/fieldctl nav stop
./scripts/fieldctl status
```

### 5-2. 버튼이 화면에 들어오게 맞추기

브라우저 영상에 호출 버튼이 보이지 않으면 위치를 맞춘다. 팔은 로봇 상단 왼쪽에 있으므로 버튼이 로봇 왼쪽에 오게 한다. D에서 `resume`, C에서 teleop으로 조금씩 움직인다. 맞으면 `k`, `Ctrl+C`를 누르고 D에서 `stop`한다. 버튼이 이미 보이면 이 단계를 건너뛴다.

```bash
./scripts/fieldctl resume
```

```bash
./scripts/fieldctl teleop
```

```bash
./scripts/fieldctl stop
```

### 5-3. 누르기 (브라우저)

1. **① 임무**에서 `탑승 전 · 올라감`(▲)을 누른다.
2. 인식 결과 표에서 ▲ 버튼 하나만 잡혔는지 본다. 상태가 `누르기 준비됨`으로 바뀔 때까지 기다린다. 같은 목표가 두 개 보이거나, 5프레임 연속으로 안정되지 않거나, 영상이 끊기면 버튼이 활성화되지 않는다.
3. `목표 버튼 누르기 1회` → 확인창 확인.
4. 팔이 한 사이클 누르고 home으로 돌아온다. 결과 칸에 오류가 없는지 본다.
5. **호출 버튼 불이 켜졌는지 눈으로 확인한다.** 안 켜졌으면 연타하지 않는다. 인식 결과를 확인하고 한 번만 다시 누른다. 그래도 안 되면 손으로 누르고 수동 보조로 기록한다.

**완료 확인:** 호출등 켜짐, 팔 home 복귀, 차체 정지 유지.

## 6. F1 탑승 + 층 버튼 `4` 누르기

### 6-1. 탑승 (수동 WASD)

엘리베이터가 도착해 **양쪽 문이 기존 통로 밖으로 완전히 열릴 때까지** 기다린다. D에서 `resume`, C에서 teleop을 켠다.

```bash
./scripts/fieldctl resume
```

```bash
./scripts/fieldctl teleop
```

문턱 앞에서 `z`를 3번 눌러 약 0.073 m/s로 낮추고 문틀 중앙을 유지해 들어간다. 캐빈 안에서는 **층 버튼 패널이 로봇 왼쪽에 오고 브라우저 영상에 보이는 위치**로 맞춘다. 로봇 전체와 택배가 문 닫힘 영역 밖에 있어야 한다. `k`, `Ctrl+C`를 누르고 D에서 정지한다.

```bash
./scripts/fieldctl stop
```

### 6-2. 층 버튼 누르기 (브라우저)

1. **① 임무**에서 `내부 · 올라감`(`4`)을 누른다. 앱 규칙상 올라감은 `4` 버튼이다. 실제 목적층 버튼이 `4`가 아니면 누르지 말고 0절 규칙을 따른다.
2. 인식 결과에서 `4`가 맞게 잡혔고 `누르기 준비됨`인지 확인한다.
3. `목표 버튼 누르기 1회` → 확인.
4. **목적층 버튼 불이 켜졌는지, 다른 층이 눌리지 않았는지** 눈으로 확인한다. 잘못 눌렸으면 자동으로 계속하지 않는다. 사람이 바로잡고 기록한다.

### 6-3. 이동 중

문이 닫히고 층이 이동하는 동안 base·recorder·지게팔 터미널·팔 앱은 그대로 둔다. Nav2는 꺼진 상태를 유지한다.

## 7. F2 하차 (수동 WASD)

F2 도착과 **양쪽 문 완전 열림**을 확인한다. D에서 `resume`, C에서 teleop을 켠다. 저속으로 나가 F2 문 앞 퇴장 표식에 선다. `k`, `Ctrl+C`를 누르고 D에서 정지한다.

```bash
./scripts/fieldctl resume
```

```bash
./scripts/fieldctl teleop
```

```bash
./scripts/fieldctl stop
```

**완료 확인:** 로봇 전체가 문 밖, 퇴장 표식 정차.

## 8. F2 엘리베이터 앞 → 배달 위치 (Nav2)

B에서 **F2** Nav2를 켜고 `Managed nodes are active`를 기다린다.

```bash
./scripts/fieldctl nav start F2
```

C에서 입력한다. 아래 pose는 F2 퇴장 표식의 명목 좌표다. 실제 방향이 다르면 yaw까지 확인한다.

```bash
./scripts/fieldctl pose set -11.375 -2.525 -0.828849
./scripts/fieldctl map guard F2
./scripts/fieldctl pose capture f2_exit_check F2 --duration 10 --floor-mark F2_EXIT
./scripts/fieldctl record check
./scripts/fieldctl resume
./scripts/fieldctl goal f2_delivery_left_room4 F2
```

목적지는 엘리베이터에서 내려 오른쪽으로 돌아 진행할 때 **왼쪽 네 번째 방** 앞 복도 중심이다. 도착하면 방 앞 표식과의 위치·방향 차이를 줄자로 재서 기록한다.

**완료 확인:** goal 성공, 배달 위치 정차.

## 9. 택배 하차 (수동 WASD + 지게팔)

### 9-1. 수동 전환

D에서 입력하고 `navigation=stopped`를 확인한다.

```bash
./scripts/fieldctl stop
./scripts/fieldctl nav stop
./scripts/fieldctl status
./scripts/fieldctl resume
```

C에서 teleop을 켠다. 택배를 내려놓을 자리 위로 천천히 맞춘다. `k`, `Ctrl+C`를 누르고 D에서 정지한다.

```bash
./scripts/fieldctl teleop
```

```bash
./scripts/fieldctl stop
```

### 9-2. 내리기 (E)

```
lift> s
```

`position_pulses=166400 reference=CONFIRMED`인지 확인한다. 포크 아래에 발이나 손이 없는지 확인하고 내린다.

```
lift> d
```

약 35초 뒤 `DOWN complete`와 `position_pulses=0`이 나오면 끝이다. 이상하면 즉시 `!`를 누른다. 택배가 바닥에 안정적으로 놓였는지 눈으로 확인한다.

### 9-3. 빠져나오기

D에서 `resume`, C에서 teleop을 켠다. 포크가 택배에서 완전히 빠질 때까지 천천히 뒤로 뺀다. `k`, `Ctrl+C`를 누르고 D에서 정지한다.

```bash
./scripts/fieldctl resume
```

```bash
./scripts/fieldctl teleop
```

```bash
./scripts/fieldctl stop
```

**완료 확인:** 택배 하차, 포크 아래 끝, 차체 정지, teleop 종료.

## 10. 배달 위치 → F2 엘리베이터 앞 (Nav2)

B에서 Nav2를 켜고 `Managed nodes are active`를 기다린다.

```bash
./scripts/fieldctl nav start F2
```

C에서 입력한다. 아래 pose는 등록 좌표다. 9절에서 크게 옮겼거나 부록 A로 `--replace`했다면 실제 정차 좌표로 바꾼다.

```bash
./scripts/fieldctl pose set -24.725 -17.975 -2.3963
./scripts/fieldctl map guard F2
./scripts/fieldctl pose capture delivery_depart F2 --duration 10 --floor-mark F2_DELIVERY
./scripts/fieldctl record check
./scripts/fieldctl resume
./scripts/fieldctl goal f2_elevator_staging_v1 F2
```

문 앞 goal은 staging을 쓴다. staging부터 입장까지는 수동이다. 시뮬 S1 최소 벽 거리는 entry P0 3회가 0.161~0.208 m로 0.20 m 기준 미달을 포함했다. staging P0 2회는 0.319~0.321 m였다.

**완료 확인:** goal 성공, F2 엘리베이터 앞 정차.

## 11. F2 호출 버튼 ▼ 누르기 (팔)

5절과 같다. 달라지는 점은 임무 선택뿐이다.

1. D: `stop` → `nav stop` → `status`로 `navigation=stopped` 확인.
2. 버튼이 화면에 안 보이면 D `resume` → C `teleop`으로 맞춘 뒤 `k`, `Ctrl+C`, D `stop`.
3. 브라우저 **① 임무**에서 `탑승 전 · 내려감`(▼) → `누르기 준비됨` 확인 → `목표 버튼 누르기 1회`.
4. 호출등이 켜졌는지 눈으로 확인한다.

```bash
./scripts/fieldctl stop
./scripts/fieldctl nav stop
./scripts/fieldctl status
```

## 12. F2 탑승 + 층 버튼 `1` 누르기

6절과 같다. 달라지는 점은 임무 선택뿐이다.

1. 양쪽 문 완전 열림 확인 → D `resume` → C `teleop`.
2. `z` 3번으로 감속하고 입장한다. 패널이 로봇 왼쪽·화면 안에 오게 정차한 뒤 `k`, `Ctrl+C`, D `stop`.
3. 브라우저 **① 임무**에서 `내부 · 내려감`(`1`) → `누르기 준비됨` 확인 → `목표 버튼 누르기 1회`.
4. 목적층 버튼 불과 오선택 여부를 눈으로 확인한다.

```bash
./scripts/fieldctl resume
```

```bash
./scripts/fieldctl teleop
```

```bash
./scripts/fieldctl stop
```

## 13. F1 하차 (수동 WASD)

F1 도착과 양쪽 문 완전 열림을 확인한다. D에서 `resume`, C에서 teleop을 켠다. F1 문 앞 퇴장 표식까지 나간 뒤 `k`, `Ctrl+C`를 누르고 D에서 정지한다.

```bash
./scripts/fieldctl resume
```

```bash
./scripts/fieldctl teleop
```

```bash
./scripts/fieldctl stop
```

## 14. F1 엘리베이터 앞 → 초기 위치 (Nav2)

B에서 **F1** Nav2를 켜고 `Managed nodes are active`를 기다린다.

```bash
./scripts/fieldctl nav start F1
```

C에서 입력한다.

```bash
./scripts/fieldctl pose set -8.725 -0.175 0.918927
./scripts/fieldctl map guard F1
./scripts/fieldctl pose capture f1_return_exit F1 --duration 10 --floor-mark F1_EXIT
./scripts/fieldctl record check
./scripts/fieldctl resume
./scripts/fieldctl goal f1_initial_test F1
```

**완료 확인:** goal 성공, 최초 바닥 표식과 차체 방향 일치.

## 15. 종료

### 15-1. 정지와 녹화 종료 (D, C)

```bash
./scripts/fieldctl stop
./scripts/fieldctl record stop
```

`record stop` 출력만으로 정상 종료를 판정하지 않는다. `metadata.yaml`이 생겼는지 확인한다.

```bash
docker exec ros2_humble ls -l /ros2_ws/logs/field_bags/manual_roundtrip_test
```

`metadata.yaml`이 없으면 내부 recorder에 SIGINT를 보내고, 몇 초 뒤 위 `ls`를 다시 실행한다.

```bash
docker exec ros2_humble pkill -INT -f "ros2 bag record"
```

bag을 검사한다. contract 검사는 반드시 통과해야 한다. 원본 녹화에는 hidden action status가 없다. 분석 스크립트가 그 사유로 exit 1이면 알려진 결함으로 기록하고 주행 실패로 판정하지 않는다.

```bash
docker exec -w /ros2_ws ros2_humble bash -lc 'source /opt/ros/humble/setup.bash && source install/setup.bash && python3 scripts/bag_contract.py inspect logs/field_bags/manual_roundtrip_test --require /scan --require /odom --require /tf --require /tf_static && python3 scripts/analyze_nav_bag.py logs/field_bags/manual_roundtrip_test'
```

### 15-2. 팔 앱 종료 (브라우저, F)

브라우저 ③에서 `home`을 누르고 팔이 home에 온 것을 확인한다. 그다음 F에서 `Ctrl+C`를 누른다. `Ctrl+C`는 정지 명령만 보내고 자동 복귀하지 않으므로 반드시 home 뒤에 누른다. W의 터널 창도 `Ctrl+C`로 닫는다.

### 15-3. 지게팔 터미널 종료 (E)

```
lift> s
```

`position_pulses=0`(아래 끝)인지 확인하고 `q`로 나간다.

### 15-4. Nav2와 base 종료 (D)

```bash
./scripts/fieldctl nav stop
./scripts/fieldctl base stop
```

## 16. 판정 기록

| 항목                | 기록                                                                        |
| ------------------- | --------------------------------------------------------------------------- |
| autonomous goal 6개 | 각 성공/실패. 중간 abort나 수동 개입으로 대신한 구간은 성공으로 세지 않는다 |
| 수동 전환           | 매번`navigation=stopped` 확인 여부                                        |
| 문 통과             | 차체·택배 충돌 없음, 양쪽 문 완전 열림 확인                                |
| 상차·하차          | `UP complete`/`DOWN complete`와 눈 확인. 걸림이나 `!` 사용 여부       |
| 버튼 4회            | ▲·`4`·▼·`1` 각각 앱 결과, 불빛 확인, 재시도나 손으로 누름 여부     |
| 복귀                | 최초 표식과의 위치·방향 차이                                               |
| 녹화                | recorder 성장, contract 통과, 분석 결과                                     |
| 기타                | 최소 벽 거리, collision-ahead, rpm 차단, ready 이탈, 배달 위치 줄자 오차    |

앱으로 누르지 못하고 사람이 누른 버튼, 사람이 손으로 옮긴 택배는 수동 보조로 따로 적는다.

---

## 부록 A. waypoint 좌표

| 이름                       | 층 |       x |       y | yaw(rad) | 근거                                        |
| -------------------------- | -- | ------: | ------: | -------: | ------------------------------------------- |
| `f2_delivery_left_room4` | F2 | -24.725 | -17.975 |  -2.3963 | 왼쪽 네 번째 방 홈 앞 복도 중심`[제안값]` |
| `f1_initial_test`        | F1 |  -4.518 |   2.179 |   -1.701 | 9/15 idle 시드, footprint–벽 여유 0.453 m  |

`f2_delivery_left_room4`의 오프라인 확인값 `[측정값]`은 footprint–occupied 0.928 m, footprint–unknown 1.038 m다. Nav2 경로 재현 결과는 아니다. 그림은 `artifacts/manual_delivery_20260915/f2_room4_proposal.png`, 기록은 같은 폴더의 `room4_registration.json`이다. 두 waypoint는 2026-09-15 22:55 KST에 Jetson에 등록했다. 백업은 `/ros2_ws/logs/field_execution/manual_roundtrip_setup_20260915-135535/`에 있다.

도착 위치가 표식과 크게 다르면 teleop으로 표식에 정차한다. 그다음 RViz 정합과 차체 여유를 확인하고 C에서 교체한다. 셸 변수는 창마다 따로이므로 C에서 `export`한다.

```bash
export ROOM4_X=... ROOM4_Y=... ROOM4_YAW=...
./scripts/fieldctl pose set "$ROOM4_X" "$ROOM4_Y" "$ROOM4_YAW"
./scripts/fieldctl pose capture room4_registration F2 --duration 10 --floor-mark F2_LEFT_ROOM4
./scripts/fieldctl waypoint save f2_delivery_left_room4 F2 "$ROOM4_X" "$ROOM4_Y" "$ROOM4_YAW" --replace
```

`f1_initial_test`도 최초 표식이 시드와 다르면 같은 방식으로 교체한다.

## 부록 B. PC 시뮬레이션

저장소 `lee/sim-real-maps` 체크아웃 루트에서 실행한다. 하드웨어는 연결하지 않는다. 팔·지게팔·엘리베이터 버튼은 시뮬레이션하지 않는다. 층 이동은 월드 교체로 대신하고, 상하차는 적재 상태 기록만 한다.

```bash
SIM_GUI=1 SIM_MODE=manual bash sim/real_maps/run_suite.sh --name M1_manual_roundtrip_r1
```

결과는 `logs/real_map_sim/M1_manual_roundtrip_r1/`에 저장된다. 새 실행에는 새 이름을 쓴다. A/B와 증속 검증은 `bash sim/real_maps/run_suite.sh --repeats 3` 뒤 `python3 sim/real_maps/summarize.py`로 돌린다.
