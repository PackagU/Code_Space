import re
import time
import serial

PORT = "/dev/ttyUSB0"
BAUD = 115200

SERVO_IDS = ("000", "001", "002", "003")

# 조립 후 관절별로 실제 안전 범위를 측정해서 수정하세요.
PWM_LIMITS = {
    "000": (900, 2600),
    "001": (900, 2600),
    "002": (900, 2600),
    "003": (900, 2600),
}

# 아래 값은 예시입니다.
# 처음에는 모두 1500으로 두고, 한 관절씩 조금씩 조정하세요.
POSES = {
    "home": {
        "000": 1500,
        "001": 1141,
        "002": 2470,
        "003": 1570,
    },
    "press_ready": {
        "000": 1500,
        "001": 1500,
        "002": 1500,
        "003": 1500,
    },
    "pre_press": {
        "000": 1590,
        "001": 1430,
        "002": 1800,
        "003": 1700,
    },
    "press": {
        "000": 1590,
        "001": 1730,
        "002": 1700,
        "003": 1700,
    },
    "retreat": {
        "000": 1590,
        "001": 1430,
        "002": 1800,
        "003": 1700,
    },
    "pre_press2": {
        "000": 1450,
        "001": 1380,
        "002": 2020,
        "003": 1570,
    },
    "press2": {
        "000": 1450,
        "001": 1680,
        "002": 1920,
        "003": 1570,
    },
    "retreat2": {
        "000": 1450,
        "001": 1380,
        "002": 2020,
        "003": 1570,
    },
}


class ServoArm:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.5)
        time.sleep(0.5)

    def close(self):
        self.ser.close()

    def send(self, command, wait=0.2):
        self.ser.reset_input_buffer()
        self.ser.write(command.encode("ascii"))
        self.ser.flush()
        time.sleep(wait)

        response = self.ser.read_all().decode("ascii", errors="ignore")

        print("SEND:", command)
        if response:
            print("RECV:", response)

        return response

    def check_pwm(self, servo_id, pwm):
        if servo_id not in SERVO_IDS:
            raise ValueError(f"알 수 없는 모터 ID: {servo_id}")

        minimum, maximum = PWM_LIMITS[servo_id]

        if not minimum <= pwm <= maximum:
            raise ValueError(
                f"ID {servo_id}의 PWM {pwm}이 "
                f"안전 범위 {minimum}~{maximum}를 벗어났습니다."
            )

    def read_position(self, servo_id):
        response = self.send(f"#{servo_id}PRAD!", wait=0.2)

        match = re.search(rf"#{servo_id}P(\d+)\!", response)

        if match is None:
            print(f"ID {servo_id} 위치 응답을 읽지 못했습니다.")
            return None

        position = int(match.group(1))
        print(f"ID {servo_id} position: {position}")
        return position

    def move_pose(self, pose_name, duration_ms=2000):
        if pose_name not in POSES:
            raise ValueError(f"알 수 없는 자세: {pose_name}")

        if not 0 <= duration_ms <= 9999:
            raise ValueError("이동 시간은 0~9999ms 범위여야 합니다.")

        pose = POSES[pose_name]

        for servo_id, pwm in pose.items():
            self.check_pwm(servo_id, pwm)

        command_parts = []

        for servo_id in SERVO_IDS:
            pwm = pose[servo_id]
            command_parts.append(
                f"#{servo_id}P{pwm:04d}T{duration_ms:04d}!"
            )

        command = "{" + "".join(command_parts) + "}"

        self.send(command, wait=0.2)

        time.sleep(duration_ms / 1000.0 + 0.2)

        print(f"POSE COMPLETE: {pose_name}")

    def stop_all(self):
        for servo_id in SERVO_IDS:
            self.send(f"#{servo_id}PDPT!", wait=0.05)


def run_press_cycle(arm):
    print("버튼 누르기 전체 동작을 시작합니다.")

    arm.move_pose("press_ready", duration_ms=2000)
    arm.move_pose("pre_press", duration_ms=1500)
    arm.move_pose("press", duration_ms=1000)

    print("버튼을 누른 상태로 유지합니다.")
    time.sleep(0.5)

    arm.move_pose("retreat", duration_ms=1000)
    arm.move_pose("press_ready", duration_ms=1500)
    arm.move_pose("home", duration_ms=1500)

    print("버튼 누르기 전체 동작이 완료되었습니다.")


def run_press_cycle2(arm):
    print("버튼 누르기 전체 동작을 시작합니다.")

    arm.move_pose("press_ready", duration_ms=2000)
    arm.move_pose("pre_press2", duration_ms=1500)
    arm.move_pose("press2", duration_ms=1000)

    print("버튼을 누른 상태로 유지합니다.")
    time.sleep(0.5)

    arm.move_pose("retreat2", duration_ms=1000)
    arm.move_pose("press_ready", duration_ms=1500)
    arm.move_pose("home", duration_ms=1500)

    print("버튼 누르기 전체 동작이 완료되었습니다.")


arm = ServoArm(PORT, BAUD)

try:
    print("현재 모터 위치 확인")

    positions = {}

    for servo_id in SERVO_IDS:
        positions[servo_id] = arm.read_position(servo_id)

    if any(position is None for position in positions.values()):
        raise RuntimeError("일부 모터의 위치를 읽지 못했습니다.")

    input("주변을 확인하고 Enter를 누르면 home으로 이동합니다.")

    arm.move_pose("home", duration_ms=3000)
    current_pose = "home"

    while True:
        print()
        print(f"현재 자세: {current_pose}")
        print()
        print("1: home")
        print("2: press_ready")
        print("3: pre_press")
        print("4: press")
        print("5: retreat")
        print("6: 버튼 누르기 전체 동작")
        print("7: 버튼 누르기 전체 동작2")
        print("q: 프로그램 종료")

        command = input("명령을 입력하세요: ").strip().lower()

        if command == "1":
            arm.move_pose("home", duration_ms=3000)
            current_pose = "home"
        elif command == "2":
            arm.move_pose("press_ready", duration_ms=2000)
            current_pose = "press_ready"
        elif command == "3":
            arm.move_pose("pre_press", duration_ms=1500)
            current_pose = "pre_press"
        elif command == "4":
            arm.move_pose("press", duration_ms=1000)
            current_pose = "press"
        elif command == "5":
            arm.move_pose("retreat", duration_ms=1000)
            current_pose = "retreat"
        elif command == "6":
            run_press_cycle(arm)
            current_pose = "home"
        elif command == "7":
            run_press_cycle2(arm)
            current_pose = "home"
        elif command == "q":
            print(f"{current_pose} 자세를 유지하고 종료합니다.")
            break
        else:
            print("1~7 또는 q를 입력하세요.")

except KeyboardInterrupt:
    print("\n사용자가 동작을 중단했습니다.")
    arm.stop_all()

except Exception as error:
    print("ERROR:", error)
    arm.stop_all()

finally:
    arm.close()
