// P2 candidate only. Do not upload without the field H1/firmware approval boundary.
#include <Dynamixel2Arduino.h>
#include <IMU.h>
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
const uint32_t DXL_BAUDRATE = 1000000;
const uint32_t HOST_BAUDRATE = 115200;
const float MAX_ABS_RPM = 30.0;
const uint32_t COMMAND_TIMEOUT_MS = 500;
const uint32_t FEEDBACK_PERIOD_MS = 20;       // [제안값] 50 Hz
const uint32_t IMU_STALE_MS = 100;            // [제안값]
const uint32_t ERROR_REPORT_PERIOD_MS = 1000; // [제안값]

// OpenCR TurtleBot3 library source constants: raw ADC -> SI units.
const float GYRO_FACTOR = 0.0010642;           // rad/s per count
const float ACCEL_FACTOR = 0.000598550415;     // m/s^2 per count

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
cIMU IMU;

char input_buffer[64];
size_t input_length = 0;
uint32_t last_valid_command_ms = 0;
uint32_t last_feedback_ms = 0;
uint32_t last_imu_sample_ms = 0;
uint32_t last_error_report_ms = 0;
bool watchdog_reported = false;
bool motors_ready = false;
bool imu_connected = false;
bool gyro_sample_valid = false;
bool full_imu_sample_valid = false;
float gyro_si[3] = {0.0, 0.0, 0.0};
float accel_si[3] = {0.0, 0.0, 0.0};
float quat_wxyz[4] = {0.0, 0.0, 0.0, 0.0};

void reportErrorRateLimited(const char *token, int detail_a = 0, int detail_b = 0) {
  const uint32_t now = millis();
  if (now - last_error_report_ms < ERROR_REPORT_PERIOD_MS) return;
  last_error_report_ms = now;
  HOST_SERIAL.print("E ");
  HOST_SERIAL.print(token);
  HOST_SERIAL.print(' ');
  HOST_SERIAL.print(detail_a);
  HOST_SERIAL.print(' ');
  HOST_SERIAL.println(detail_b);
}

void setWheels(float left_rpm, float right_rpm) {
  if (!motors_ready) return;
  const bool left_ok = dxl.setGoalVelocity(LEFT_ID, left_rpm, UNIT_RPM);
  const int left_error = static_cast<int>(dxl.getLastLibErrCode());
  const bool right_ok = dxl.setGoalVelocity(RIGHT_ID, right_rpm, UNIT_RPM);
  const int right_error = static_cast<int>(dxl.getLastLibErrCode());
  if (!left_ok || !right_ok) {
    reportErrorRateLimited("dynamixel_write", left_error, right_error);
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
    if (motors_ready) {
      setWheels(left_rpm, right_rpm);
      last_valid_command_ms = millis();
      watchdog_reported = false;
    } else {
      HOST_SERIAL.println("E motors_unavailable");
    }
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

void updateImuSample() {
  if (!imu_connected || IMU.update() == 0) return;

  for (int i = 0; i < 3; ++i) {
    gyro_si[i] = IMU.SEN.gyroADC[i] * GYRO_FACTOR;
    accel_si[i] = IMU.SEN.accADC[i] * ACCEL_FACTOR;
  }
  for (int i = 0; i < 4; ++i) quat_wxyz[i] = IMU.quat[i];

  gyro_sample_valid = true;
  bool accel_valid = true;
  bool quat_finite = true;
  float quat_norm_sq = 0.0;
  for (int i = 0; i < 3; ++i) {
    gyro_sample_valid = gyro_sample_valid && isfinite(gyro_si[i]);
    accel_valid = accel_valid && isfinite(accel_si[i]);
  }
  for (int i = 0; i < 4; ++i) {
    quat_finite = quat_finite && isfinite(quat_wxyz[i]);
    quat_norm_sq += quat_wxyz[i] * quat_wxyz[i];
  }
  const float quat_norm = sqrt(quat_norm_sq);
  const bool quat_valid = quat_finite && quat_norm >= 0.5 && quat_norm <= 1.5;
  full_imu_sample_valid = gyro_sample_valid && accel_valid && quat_valid;
  if (gyro_sample_valid) last_imu_sample_ms = millis();
}

bool imuSampleFresh() {
  return gyro_sample_valid && (millis() - last_imu_sample_ms <= IMU_STALE_MS);
}

void printImuPayload() {
  for (int i = 0; i < 3; ++i) {
    HOST_SERIAL.print(gyro_si[i], 5);
    HOST_SERIAL.print(' ');
  }
  for (int i = 0; i < 3; ++i) {
    HOST_SERIAL.print(accel_si[i], 5);
    HOST_SERIAL.print(' ');
  }
  HOST_SERIAL.print(quat_wxyz[0], 6); // w, x, y, z
  HOST_SERIAL.print(' ');
  HOST_SERIAL.print(quat_wxyz[1], 6);
  HOST_SERIAL.print(' ');
  HOST_SERIAL.print(quat_wxyz[2], 6);
  HOST_SERIAL.print(' ');
  HOST_SERIAL.println(quat_wxyz[3], 6);
}

void printGyroOnly() {
  HOST_SERIAL.print("G ");
  HOST_SERIAL.print(gyro_si[0], 5);
  HOST_SERIAL.print(' ');
  HOST_SERIAL.print(gyro_si[1], 5);
  HOST_SERIAL.print(' ');
  HOST_SERIAL.println(gyro_si[2], 5);
}

bool readWheelFeedback(float *left_rpm, float *right_rpm) {
  *left_rpm = dxl.getPresentVelocity(LEFT_ID, UNIT_RPM);
  const int left_error = static_cast<int>(dxl.getLastLibErrCode());
  *right_rpm = dxl.getPresentVelocity(RIGHT_ID, UNIT_RPM);
  const int right_error = static_cast<int>(dxl.getLastLibErrCode());
  if (left_error != DXL_LIB_OK || right_error != DXL_LIB_OK) {
    stopWheels();
    reportErrorRateLimited("dynamixel_read", left_error, right_error);
    return false;
  }
  if (!isfinite(*left_rpm) || !isfinite(*right_rpm)) {
    stopWheels();
    reportErrorRateLimited("invalid_feedback");
    return false;
  }
  return true;
}

void publishFeedback() {
  const bool imu_fresh = imuSampleFresh();
  const bool full_imu = imu_fresh && full_imu_sample_valid;
  float left_rpm = 0.0;
  float right_rpm = 0.0;
  const bool wheel_valid = motors_ready && readWheelFeedback(&left_rpm, &right_rpm);

  if (wheel_valid) {
    HOST_SERIAL.print("F ");
    HOST_SERIAL.print(left_rpm, 3);
    HOST_SERIAL.print(' ');
    HOST_SERIAL.print(right_rpm, 3);
    if (full_imu) {
      HOST_SERIAL.print(' ');
      printImuPayload(); // combined 13-field frame
    } else {
      HOST_SERIAL.println(); // legacy 3-field frame remains valid
      if (imu_fresh) printGyroOnly();
    }
    return;
  }

  // USB-only/motor-fault mode: never fabricate F 0 0.
  if (full_imu) {
    HOST_SERIAL.print("I ");
    printImuPayload();
  } else if (imu_fresh) {
    printGyroOnly();
  } else if (imu_connected) {
    reportErrorRateLimited("imu_stale");
  }
}

void setup() {
  HOST_SERIAL.begin(HOST_BAUDRATE);
  delay(1000);

  IMU.begin(200);
  imu_connected = IMU.bConnected;
  if (!imu_connected) HOST_SERIAL.println("E imu_init_failed");

  dxl.begin(DXL_BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  const bool left_ok = configureMotor(LEFT_ID);
  const bool right_ok = configureMotor(RIGHT_ID);
  motors_ready = left_ok && right_ok;
  if (!motors_ready) {
    dxl.torqueOff(LEFT_ID);
    dxl.torqueOff(RIGHT_ID);
    HOST_SERIAL.println("E motor_setup_failed");
  } else {
    stopWheels();
  }
  last_valid_command_ms = millis();
  HOST_SERIAL.print("HELLO opencr 0.3-imu motor=");
  HOST_SERIAL.print(motors_ready ? 1 : 0);
  HOST_SERIAL.print(" imu=");
  HOST_SERIAL.println(imu_connected ? 1 : 0);
}

void loop() {
  readHostCommands(); // command/watchdog work is serviced before feedback formatting
  updateImuSample();
  const uint32_t now = millis();
  if (motors_ready && now - last_valid_command_ms > COMMAND_TIMEOUT_MS) {
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
