// QuadSpider firmware entry point (composition root).
// Creates the hardware drivers, injects them into the platform-independent
// Robot, runs the boot self-test and starts the FreeRTOS tasks.

#include <Arduino.h>

#include "quad_core.h"
#include "quad_esp32.h"
#include "robot_config.h"

namespace {

constexpr const char* kFirmwareVersion = "1.0.0";

uint32_t clockMs() { return millis(); }

const qspider::RobotConfig kConfig = qspider::makeRobotConfig();

qspider::EventLog eventLog(&clockMs);
qspider::Pca9685ServoDriver servoDriver(kConfig.pca);
qspider::Ps5Gamepad gamepad(kConfig.controller);
qspider::AdcPowerSensor powerSensor(kConfig.battery);
qspider::NvsCalibrationStore calibrationStore;

qspider::Robot robot(kConfig, servoDriver, gamepad, powerSensor, calibrationStore, eventLog);
qspider::SerialConsole console(robot, eventLog, kConfig, Serial);
qspider::DashboardServer dashboard(robot, eventLog, kConfig, kFirmwareVersion);
qspider::RobotRuntime runtime(robot, kConfig, console, dashboard, gamepad);

}  // namespace

void setup() {
  Serial.setTxBufferSize(4096);
  Serial.begin(115200);
  eventLog.log(qspider::LogLevel::Info, qspider::LogCategory::System, "Firmware %s", kFirmwareVersion);

  // Self-test: config, calibration, PCA9685 (all outputs off), battery.
  powerSensor.begin();
  robot.begin(millis());

  if (gamepad.begin()) {
    eventLog.log(qspider::LogLevel::Info, qspider::LogCategory::Input, "Bluetooth ready - press PS on the controller");
  } else {
    eventLog.log(qspider::LogLevel::Error, qspider::LogCategory::Input, "Bluetooth start failed");
  }

  runtime.start();
}

void loop() {
  // All work happens in the control and service tasks.
  vTaskDelete(nullptr);
}
