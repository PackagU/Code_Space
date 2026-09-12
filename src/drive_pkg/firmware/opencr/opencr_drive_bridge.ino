#include <Dynamixel2Arduino.h>
#include <math.h>

#if defined(ARDUINO_OpenCR)
  #define DXL_SERIAL Serial3
  #define HOST_SERIAL Serial
  const int DXL_DIR_PIN = 84;
#else
  #error "This sketch is intended for OpenCR."
#endif

const uint8_t LEFT_ID = 1;
const uint8_t RIGHT_ID = 2;
const float DXL_PROTOCOL_VERSION = 2.0;
const uint32_t DXL_BAUDRATE = 1000000;  // 사용자가 2026-09-11 구동 확인한 값
const uint32_t HOST_BAUDRATE = 115200;
const float MAX_ABS_RPM = 30.0;         // 사용자 지정 현장 상한, 2026-09-12
const uint32_t COMMAND_TIMEOUT_MS = 500;
const uint32_t FEEDBACK_PERIOD_MS = 20;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
char input_buffer[64];
size_t input_length = 0;
uint32_t last_valid_command_ms = 0;
uint32_t last_feedback_ms = 0;
bool watchdog_reported = false;

void setWheels(float left_rpm, float right_rpm) {
  const bool left_ok = dxl.setGoalVelocity(LEFT_ID, left_rpm, UNIT_RPM);
  const bool right_ok = dxl.setGoalVelocity(RIGHT_ID, right_rpm, UNIT_RPM);
  if (!left_ok || !right_ok) {
    HOST_SERIAL.print("E dynamixel_write ");
    HOST_SERIAL.println(dxl.getLastLibErrCode());
  }
}

void stopWheels() {
  setWheels(0.0, 0.0);
}

bool configureMotor(uint8_t id) {
  if (!dxl.ping(id)) return false;
  if (!dxl.torqueOff(id)) return false;
  if (!dxl.setOperatingMode(id, OP_VELOCITY)) return false;
  if (!dxl.setGoalVelocity(id, 0.0, UNIT_RPM)) return false;
  return dxl.torqueOn(id);
}

bool parseVelocityCommand(const char *line, float *left_rpm, float *right_rpm) {
  char extra = '\0';
  if (sscanf(line, "V %f %f %c", left_rpm, right_rpm, &extra) != 2) return false;
  if (!isfinite(*left_rpm) || !isfinite(*right_rpm)) return false;
  if (fabs(*left_rpm) > MAX_ABS_RPM || fabs(*right_rpm) > MAX_ABS_RPM) return false;
  return true;
}

void processLine() {
  input_buffer[input_length] = '\0';
  float left_rpm = 0.0;
  float right_rpm = 0.0;
  if (parseVelocityCommand(input_buffer, &left_rpm, &right_rpm)) {
    setWheels(left_rpm, right_rpm);
    last_valid_command_ms = millis();
    watchdog_reported = false;
  } else if (input_length > 0) {
    stopWheels();
    HOST_SERIAL.println("E invalid_command");
  }
  input_length = 0;
}

void readHostCommands() {
  while (HOST_SERIAL.available()) {
    const char c = static_cast<char>(HOST_SERIAL.read());
    if (c == '\n') {
      processLine();
    } else if (c != '\r') {
      if (input_length + 1 < sizeof(input_buffer)) {
        input_buffer[input_length++] = c;
      } else {
        input_length = 0;
        stopWheels();
        HOST_SERIAL.println("E command_too_long");
      }
    }
  }
}

void publishFeedback() {
  const float left_rpm = dxl.getPresentVelocity(LEFT_ID, UNIT_RPM);
  const float right_rpm = dxl.getPresentVelocity(RIGHT_ID, UNIT_RPM);
  if (!isfinite(left_rpm) || !isfinite(right_rpm)) {
    stopWheels();
    HOST_SERIAL.println("E invalid_feedback");
    return;
  }
  // v0.2 minimal feedback. Jetson publishes wheel odom and intentionally omits IMU.
  HOST_SERIAL.print("F ");
  HOST_SERIAL.print(left_rpm, 3);
  HOST_SERIAL.print(' ');
  HOST_SERIAL.println(right_rpm, 3);
}

void setup() {
  HOST_SERIAL.begin(HOST_BAUDRATE);
  delay(1000);
  dxl.begin(DXL_BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  const bool left_ok = configureMotor(LEFT_ID);
  const bool right_ok = configureMotor(RIGHT_ID);
  if (!left_ok || !right_ok) {
    dxl.torqueOff(LEFT_ID);
    dxl.torqueOff(RIGHT_ID);
    HOST_SERIAL.println("E motor_setup_failed");
    while (true) delay(1000);
  }
  stopWheels();
  last_valid_command_ms = millis();
  HOST_SERIAL.println("HELLO opencr 0.2-minimal");
}

void loop() {
  readHostCommands();
  const uint32_t now = millis();
  if (now - last_valid_command_ms > COMMAND_TIMEOUT_MS) {
    stopWheels();
    if (!watchdog_reported) {
      HOST_SERIAL.println("E command_watchdog_stop");
      watchdog_reported = true;
    }
  }
  if (now - last_feedback_ms >= FEEDBACK_PERIOD_MS) {
    last_feedback_ms = now;
    publishFeedback();
  }
}
