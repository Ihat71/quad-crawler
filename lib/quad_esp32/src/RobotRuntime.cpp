#include "RobotRuntime.h"

#include <Arduino.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>

namespace qspider {

namespace {
constexpr uint32_t kControlStack = 8192;
constexpr uint32_t kServiceStack = 12288;
constexpr UBaseType_t kControlPriority = configMAX_PRIORITIES - 5;
constexpr UBaseType_t kServicePriority = 1;
constexpr uint32_t kWatchdogSeconds = 3;

struct Colour {
  uint8_t r, g, b;
};
}  // namespace

RobotRuntime::RobotRuntime(Robot& robot, const RobotConfig& cfg, SerialConsole& console, DashboardServer& dashboard,
                           Ps5Gamepad& gamepad)
    : robot_(robot), cfg_(cfg), console_(console), dashboard_(dashboard), gamepad_(gamepad) {}

void RobotRuntime::start() {
  xTaskCreatePinnedToCore(&RobotRuntime::controlTaskEntry, "control", kControlStack, this, kControlPriority, nullptr,
                          1);
  xTaskCreatePinnedToCore(&RobotRuntime::serviceTaskEntry, "service", kServiceStack, this, kServicePriority, nullptr,
                          0);
}

void RobotRuntime::controlTaskEntry(void* self) { static_cast<RobotRuntime*>(self)->controlLoop(); }
void RobotRuntime::serviceTaskEntry(void* self) { static_cast<RobotRuntime*>(self)->serviceLoop(); }

void RobotRuntime::controlLoop() {
  // A hung control loop resets the chip; the PCA9685 is switched off again at boot.
  esp_task_wdt_init(kWatchdogSeconds, true);
  esp_task_wdt_add(nullptr);

  const TickType_t period = pdMS_TO_TICKS(cfg_.timing.controlPeriodMs);
  TickType_t lastWake = xTaskGetTickCount();
  int64_t prevStartUs = esp_timer_get_time();
  for (;;) {
    vTaskDelayUntil(&lastWake, period);
    const int64_t startUs = esp_timer_get_time();
    robot_.tick(millis());
    const int64_t endUs = esp_timer_get_time();
    robot_.reportTiming(static_cast<uint32_t>(startUs - prevStartUs), static_cast<uint32_t>(endUs - startUs));
    prevStartUs = startUs;
    esp_task_wdt_reset();
  }
}

void RobotRuntime::serviceLoop() {
  dashboard_.begin();
  for (;;) {
    console_.poll();
    dashboard_.poll();
    updateControllerFeedback(millis());
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void RobotRuntime::updateControllerFeedback(uint32_t nowMs) {
  gamepad_.service(nowMs);
  const RobotSnapshot s = robot_.snapshot();

  // Lightbar shows the mode; rumble confirms an emergency stop.
  static const Colour kColours[] = {
      {0, 160, 0},    // 0 NORMAL
      {0, 80, 255},   // 1 IK
      {200, 0, 200},  // 2 DANCE
      {220, 160, 0},  // 3 CALIBRATION
      {255, 0, 0},    // 4 EMERGENCY_STOP
      {20, 20, 20},   // 5 outputs off (waiting / shutdown)
      {255, 60, 0},   // 6 battery low
  };
  int colour = static_cast<int>(s.mode);
  if (s.mode != Mode::EmergencyStop) {
    if (s.lifecycle == Lifecycle::Shutdown || s.lifecycle == Lifecycle::WaitingForController)
      colour = 5;
    else if (s.mode == Mode::Normal && (s.activeFaults & faultBit(Fault::BatteryLow)))
      colour = 6;
  }

  if (s.controllerConnected && (!lastConnected_ || colour != lastColour_)) {
    gamepad_.setLightbar(kColours[colour].r, kColours[colour].g, kColours[colour].b);
    lastColour_ = colour;
  }
  const bool estop = s.mode == Mode::EmergencyStop;
  if (estop && !lastEstop_ && s.controllerConnected) gamepad_.rumble(255, 400);
  lastEstop_ = estop;
  lastConnected_ = s.controllerConnected;
}

}  // namespace qspider
