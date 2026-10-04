// Whole-robot scenarios with fake hardware: startup/shutdown, emergency stop,
// controller mapping, modes, calibration, battery and servo-driver failures.

#include <unity.h>

#include <memory>

#include "RobotHarness.h"
#include "quad_core.h"
#include "robot_config.h"

using namespace qspider;
using namespace qspider::testing;

static std::unique_ptr<RobotHarness> h;

static RobotConfig batteryConfig() {
  RobotConfig c = makeRobotConfig();
  c.battery.enabled = true;
  return c;
}

void setUp() {
  h.reset(new RobotHarness(makeRobotConfig()));
  h->power.volts = 7.8f;
}
void tearDown() { h.reset(); }

static void standUp() {
  h->tap(Button::Cross);
  h->waitIdle();
  TEST_ASSERT_EQUAL(Posture::Standing, h->robot.motion().posture());
}

// ------------------------------------------------------- startup / shutdown --

void test_outputs_stay_off_until_controller_connects() {
  h->boot();
  h->runMs(3000);
  TEST_ASSERT_EQUAL(Lifecycle::WaitingForController, h->robot.lifecycle());
  TEST_ASSERT_FALSE(h->fake.anyOn());
}

void test_arming_enables_one_leg_at_a_time_into_sit_pose() {
  h->boot();
  h->connectController();
  h->runUntil([] { return h->fake.onCount() > 0; }, 1000);
  TEST_ASSERT_EQUAL(Lifecycle::Arming, h->robot.lifecycle());
  TEST_ASSERT_EQUAL_INT(kJointsPerLeg, h->fake.onCount());
  TEST_ASSERT_TRUE(h->runUntil([] { return h->robot.lifecycle() == Lifecycle::Active; }, 3000));
  TEST_ASSERT_EQUAL_INT(kServoCount, h->fake.onCount());
  TEST_ASSERT_EQUAL(Mode::Normal, h->robot.mode());
  TEST_ASSERT_EQUAL(Posture::Sitting, h->robot.motion().posture());
}

void test_options_hold_shuts_down_and_ps_hold_wakes() {
  TEST_ASSERT_TRUE(h->bootAndArm());
  standUp();
  h->hold(Button::Options, 2100);
  TEST_ASSERT_TRUE(h->runUntil([] { return h->robot.lifecycle() == Lifecycle::Shutdown; }, 5000));
  TEST_ASSERT_EQUAL(Posture::Sitting, h->robot.motion().posture());  // sat down before going limp
  TEST_ASSERT_FALSE(h->fake.anyOn());
  TEST_ASSERT_TRUE(h->fake.sleeping);

  h->hold(Button::PlayStation, 2100);
  TEST_ASSERT_TRUE(h->runUntil([] { return h->robot.lifecycle() == Lifecycle::Active; }, 3000));
  TEST_ASSERT_EQUAL_INT(kServoCount, h->fake.onCount());
}

void test_invalid_config_blocks_startup() {
  RobotConfig bad = makeRobotConfig();
  bad.posture.standHeight = 200.0f;  // unreachable
  h.reset(new RobotHarness(bad));
  h->boot();
  h->connectController();
  h->runMs(2000);
  TEST_ASSERT_TRUE(h->robot.hasFault(Fault::ConfigInvalid));
  TEST_ASSERT_EQUAL(Mode::EmergencyStop, h->robot.mode());
  h->hold(Button::PlayStation, 2100);
  TEST_ASSERT_EQUAL(Mode::EmergencyStop, h->robot.mode());
  TEST_ASSERT_FALSE(h->fake.anyOn());
}

// ----------------------------------------------------------- emergency stop --

void test_left_dpad_estop_cuts_outputs_within_one_tick() {
  TEST_ASSERT_TRUE(h->bootAndArm());
  standUp();
  h->pad.press(Button::Left);
  h->tick();
  TEST_ASSERT_EQUAL(Mode::EmergencyStop, h->robot.mode());
  TEST_ASSERT_FALSE(h->fake.anyOn());
  TEST_ASSERT_TRUE(h->robot.snapshot().estopReason[0] != '\0');
}

void test_estop_ignores_movement_and_mode_inputs() {
  TEST_ASSERT_TRUE(h->bootAndArm());
  h->tap(Button::Left);
  h->tap(Button::Cross);
  h->tap(Button::Up);
  h->pad.sticks(0, 1, 1, 0);
  h->runMs(1000);
  TEST_ASSERT_EQUAL(Mode::EmergencyStop, h->robot.mode());
  TEST_ASSERT_FALSE(h->fake.anyOn());
}

void test_estop_clear_requires_ps_hold_of_2_seconds() {
  TEST_ASSERT_TRUE(h->bootAndArm());
  h->tap(Button::Left);
  h->hold(Button::PlayStation, 1000);
  TEST_ASSERT_EQUAL(Mode::EmergencyStop, h->robot.mode());
  h->hold(Button::PlayStation, 2100);
  TEST_ASSERT_EQUAL(Mode::Normal, h->robot.mode());
  TEST_ASSERT_TRUE(h->runUntil([] { return h->robot.lifecycle() == Lifecycle::Active; }, 3000));
  TEST_ASSERT_EQUAL(Posture::Sitting, h->robot.motion().posture());
  TEST_ASSERT_EQUAL_INT(kServoCount, h->fake.onCount());
}

void test_dashboard_estop_and_clear() {
  TEST_ASSERT_TRUE(h->bootAndArm());
  h->submit(CommandType::EmergencyStop);
  TEST_ASSERT_EQUAL(Mode::EmergencyStop, h->robot.mode());
  h->submit(CommandType::ClearEmergencyStop);
  TEST_ASSERT_EQUAL(Lifecycle::Arming, h->robot.lifecycle());
}

void test_estop_is_never_dropped_from_a_full_queue() {
  TEST_ASSERT_TRUE(h->bootAndArm());
  Command c;
  c.type = CommandType::Stand;
  for (int i = 0; i < 40; ++i) h->robot.submit(c);
  c.type = CommandType::EmergencyStop;
  TEST_ASSERT_TRUE(h->robot.submit(c));
  h->tick();
  TEST_ASSERT_EQUAL(Mode::EmergencyStop, h->robot.mode());
}

// -------------------------------------------------------- controller mapping --

void test_cross_toggles_sit_and_stand() {
  TEST_ASSERT_TRUE(h->bootAndArm());
  standUp();
  h->tap(Button::Cross);
  h->waitIdle();
  TEST_ASSERT_EQUAL(Posture::Sitting, h->robot.motion().posture());
}

void test_left_stick_walks_and_deadzone_stops() {
  TEST_ASSERT_TRUE(h->bootAndArm());
  standUp();
  h->pad.sticks(0.0f, 1.0f);
  h->runMs(1500);
  TEST_ASSERT_EQUAL(MotionState::Walking, h->robot.motion().state());
  TEST_ASSERT_TRUE(h->robot.motion().walkCommand().vx > 0.9f);
  TEST_ASSERT_TRUE(h->movementContains("WALK forward"));

  h->pad.sticks(0.05f, 0.05f);  // inside the deadzone
  TEST_ASSERT_TRUE(h->runUntil([] { return !h->robot.motion().busy(); }, 3000));
  TEST_ASSERT_EQUAL(Posture::Standing, h->robot.motion().posture());
  TEST_ASSERT_TRUE(h->movementContains("STOP"));
}

void test_full_stick_is_capped_at_90_percent_speed() {
  TEST_ASSERT_TRUE(h->bootAndArm());
  standUp();
  h->pad.sticks(0.0f, 1.0f);
  h->runMs(2000);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.9f, h->robot.motion().walkCommand().speed);
  h->pad.sticks(0.0f, 0.5f);
  h->runMs(2000);
  TEST_ASSERT_TRUE(h->robot.motion().walkCommand().speed < 0.5f);
}

void test_directions_backward_left_right_and_turning() {
  TEST_ASSERT_TRUE(h->bootAndArm());
  standUp();
  h->pad.sticks(0.0f, -1.0f);
  h->runMs(1000);
  TEST_ASSERT_TRUE(h->robot.motion().walkCommand().vx < -0.9f);
  h->pad.sticks(-1.0f, 0.0f);
  h->runMs(1000);
  TEST_ASSERT_TRUE(h->robot.motion().walkCommand().vy > 0.9f);  // stick left = +y
  h->pad.sticks(1.0f, 0.0f);
  h->runMs(1000);
  TEST_ASSERT_TRUE(h->robot.motion().walkCommand().vy < -0.9f);
  h->pad.sticks(0.0f, 0.0f, 1.0f, 0.0f);
  h->runMs(1000);
  TEST_ASSERT_TRUE(h->robot.motion().walkCommand().turn < -0.9f);  // stick right = clockwise
  TEST_ASSERT_TRUE(h->movementContains("turn-right"));
  h->pad.sticks(0.0f, 0.0f, -1.0f, 0.0f);
  h->runMs(1000);
  TEST_ASSERT_TRUE(h->robot.motion().walkCommand().turn > 0.9f);
}

void test_up_dpad_cycles_modes() {
  TEST_ASSERT_TRUE(h->bootAndArm());
  standUp();
  const Mode expected[] = {Mode::IK, Mode::Dance, Mode::Calibration, Mode::Normal};
  for (Mode m : expected) {
    h->tap(Button::Up);
    TEST_ASSERT_EQUAL(m, h->robot.mode());
    h->runMs(1500);
  }
}

void test_right_dpad_switches_gait_in_normal_and_dance_in_dance_mode() {
  TEST_ASSERT_TRUE(h->bootAndArm());
  const int gait = h->robot.motion().gaitIndex();
  h->tap(Button::Right);
  TEST_ASSERT_EQUAL_INT((gait + 1) % h->cfg.motion.gaitCount, h->robot.motion().gaitIndex());

  h->tap(Button::Up);
  h->tap(Button::Up);  // DANCE
  TEST_ASSERT_EQUAL(Mode::Dance, h->robot.mode());
  const int dance = h->robot.selectedDance();
  h->tap(Button::Right);
  TEST_ASSERT_EQUAL_INT((dance + 1) % animations::danceCount(), h->robot.selectedDance());
}

void test_triangle_resets_movement_defaults() {
  TEST_ASSERT_TRUE(h->bootAndArm());
  h->tap(Button::Right);  // change gait
  h->tap(Button::Triangle);
  h->waitIdle();
  TEST_ASSERT_EQUAL_INT(h->cfg.motion.defaultGait, h->robot.motion().gaitIndex());
  TEST_ASSERT_EQUAL(Posture::Standing, h->robot.motion().posture());
}

void test_circle_dances_and_square_attacks() {
  TEST_ASSERT_TRUE(h->bootAndArm());
  standUp();
  h->tap(Button::Circle);
  TEST_ASSERT_TRUE(h->runUntil([] { return h->robot.motion().animation() != nullptr; }, 500));
  h->waitIdle(15000);
  TEST_ASSERT_TRUE(h->movementContains("DANCE"));

  h->tap(Button::Square);
  TEST_ASSERT_TRUE(h->runUntil([] { return h->robot.motion().animation() == &animations::attack(); }, 500));
  h->waitIdle(15000);
  TEST_ASSERT_TRUE(h->movementContains("ATTACK"));
}

void test_dance_mode_circle_loops_until_pressed_again() {
  TEST_ASSERT_TRUE(h->bootAndArm());
  h->tap(Button::Up);
  h->tap(Button::Up);
  h->runMs(1500);
  h->tap(Button::Circle);
  h->runMs(10000);
  TEST_ASSERT_NOT_NULL(h->robot.motion().animation());
  h->tap(Button::Circle);
  TEST_ASSERT_TRUE(h->waitIdle());
}

void test_sticks_and_buttons_are_logged() {
  TEST_ASSERT_TRUE(h->bootAndArm());
  h->tap(Button::Cross);
  ButtonEntry e[8];
  const size_t n = h->log.buttons().since(0, e, 8);
  TEST_ASSERT_TRUE(n >= 1);
  TEST_ASSERT_EQUAL(Button::Cross, e[n - 1].button);
  TEST_ASSERT_EQUAL(ButtonAction::Press, e[n - 1].action);
}

// ---------------------------------------------------------------- IK mode --

void test_ik_mode_moves_front_right_foot_with_sticks() {
  TEST_ASSERT_TRUE(h->bootAndArm());
  standUp();
  h->tap(Button::Up);
  TEST_ASSERT_EQUAL(Mode::IK, h->robot.mode());
  TEST_ASSERT_TRUE(h->runUntil([] { return h->robot.motion().state() == MotionState::IkControl; }, 3000));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(LegId::FrontRight), h->robot.motion().ikLeg());

  h->pad.sticks(0.0f, 1.0f, 0.0f, 1.0f);  // forward + up
  h->runMs(500);
  h->pad.sticks(0, 0, 0, 0);
  h->tick();
  TEST_ASSERT_TRUE(h->robot.motion().ikOffset().x > 10.0f);
  TEST_ASSERT_TRUE(h->robot.motion().ikOffset().z > 10.0f);
  TEST_ASSERT_TRUE(h->movementContains("IK FR foot"));

  h->tap(Button::Triangle);  // reset target
  h->runMs(500);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, h->robot.motion().ikOffset().x);
}

// ------------------------------------------------------------ calibration --

void test_calibration_moves_to_reference_and_saves_trims() {
  TEST_ASSERT_TRUE(h->bootAndArm());
  h->submit(CommandType::SetMode, static_cast<int32_t>(Mode::Calibration));
  TEST_ASSERT_EQUAL(Mode::Calibration, h->robot.mode());
  h->runMs(2000);
  for (int s = 0; s < kServoCount; ++s) TEST_ASSERT_FLOAT_WITHIN(0.01f, 90.0f, h->robot.servos().target(s));

  h->tap(Button::R1);
  TEST_ASSERT_EQUAL_INT(1, h->robot.calSelected());
  h->pad.sticks(0.0f, 1.0f);
  h->runMs(460);  // ~4 nudges of 0.5 deg
  h->pad.sticks(0.0f, 0.0f);
  h->tick();
  const float trim = h->robot.servos().trim(1);
  TEST_ASSERT_TRUE(trim >= 1.0f && trim <= 2.5f);
  TEST_ASSERT_TRUE(h->robot.snapshot().calDirty);

  h->tap(Button::Cross);  // save
  TEST_ASSERT_EQUAL_INT(1, h->store.saves);
  TEST_ASSERT_EQUAL_FLOAT(trim, h->store.data.trimDeg[1]);
  TEST_ASSERT_FALSE(h->robot.snapshot().calDirty);

  // Trims are applied on the next boot.
  CalibrationData saved = h->store.data;
  h.reset(new RobotHarness(makeRobotConfig()));
  h->store.data = saved;
  h->store.hasData = true;
  h->boot();
  TEST_ASSERT_EQUAL_FLOAT(trim, h->robot.servos().trim(1));
}

void test_calibration_commands_refused_outside_calibration_mode() {
  TEST_ASSERT_TRUE(h->bootAndArm());
  h->submit(CommandType::CalSetTrim, 3, 5.0f);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, h->robot.servos().trim(3));
}

void test_leaving_calibration_returns_to_stand() {
  TEST_ASSERT_TRUE(h->bootAndArm());
  h->submit(CommandType::SetMode, static_cast<int32_t>(Mode::Calibration));
  h->runMs(2000);
  h->submit(CommandType::SetMode, static_cast<int32_t>(Mode::Normal));
  TEST_ASSERT_TRUE(h->waitIdle());
  TEST_ASSERT_EQUAL(Posture::Standing, h->robot.motion().posture());
}

// ---------------------------------------------------------------- battery --

void test_low_battery_sits_down_then_goes_limp() {
  h.reset(new RobotHarness(batteryConfig()));
  h->power.volts = 7.8f;
  TEST_ASSERT_TRUE(h->bootAndArm());
  standUp();
  // Discharge gradually (a sudden step would be reported as irregular power instead).
  for (float v = 7.8f; v > 6.3f; v -= 0.05f) {
    h->power.volts = v;
    h->runMs(100);
  }
  TEST_ASSERT_TRUE(h->runUntil([] { return h->robot.lifecycle() == Lifecycle::Stopping; }, 6000));
  TEST_ASSERT_TRUE(h->fake.anyOn());  // still powered while sitting down
  TEST_ASSERT_TRUE(h->runUntil([] { return h->robot.mode() == Mode::EmergencyStop; }, 5000));
  TEST_ASSERT_TRUE(h->robot.hasFault(Fault::BatteryCritical));
  TEST_ASSERT_FALSE(h->fake.anyOn());
  TEST_ASSERT_TRUE(h->movementContains("SIT (shutdown"));

  // Cannot be cleared while the battery is still low.
  h->hold(Button::PlayStation, 2100);
  TEST_ASSERT_EQUAL(Mode::EmergencyStop, h->robot.mode());
  TEST_ASSERT_TRUE(h->logContains("Cannot arm"));
}

void test_irregular_power_stops_immediately() {
  h.reset(new RobotHarness(batteryConfig()));
  h->power.volts = 7.8f;
  TEST_ASSERT_TRUE(h->bootAndArm());
  standUp();
  for (int i = 0; i < 10 && h->robot.mode() != Mode::EmergencyStop; ++i) {
    h->power.volts = (i % 2) ? 8.2f : 6.4f;
    h->runMs(100);
  }
  TEST_ASSERT_EQUAL(Mode::EmergencyStop, h->robot.mode());
  TEST_ASSERT_TRUE(h->robot.hasFault(Fault::BatteryUnstable));
  TEST_ASSERT_FALSE(h->fake.anyOn());
}

void test_low_battery_at_boot_refuses_to_arm() {
  h.reset(new RobotHarness(batteryConfig()));
  h->power.volts = 6.8f;
  h->boot();
  h->connectController();
  h->runMs(2000);
  TEST_ASSERT_EQUAL(Lifecycle::WaitingForController, h->robot.lifecycle());
  TEST_ASSERT_FALSE(h->fake.anyOn());
  TEST_ASSERT_TRUE(h->logContains("Cannot arm"));
}

void test_battery_disabled_reports_unavailable() {
  TEST_ASSERT_TRUE(h->bootAndArm());
  TEST_ASSERT_EQUAL(BatteryStatus::Unavailable, h->robot.snapshot().batteryStatus);
  TEST_ASSERT_TRUE(h->logContains("Battery monitoring disabled"));
}

// ----------------------------------------------------- servo driver faults --

void test_missing_servo_driver_at_boot_is_critical() {
  h->fake.present = false;
  h->boot();
  h->connectController();
  h->runMs(1000);
  TEST_ASSERT_TRUE(h->robot.hasFault(Fault::ServoDriverInit));
  TEST_ASSERT_EQUAL(Mode::EmergencyStop, h->robot.mode());
}

void test_i2c_write_failures_trigger_estop() {
  TEST_ASSERT_TRUE(h->bootAndArm());
  h->fake.failWrites = true;
  h->runMs(200);
  TEST_ASSERT_EQUAL(Mode::EmergencyStop, h->robot.mode());
  TEST_ASSERT_TRUE(h->robot.hasFault(Fault::ServoDriverComm));
}

void test_driver_reset_detected_and_recoverable() {
  TEST_ASSERT_TRUE(h->bootAndArm());
  h->fake.configLost = true;  // e.g. brown-out on the servo rail
  h->runMs(1500);
  TEST_ASSERT_EQUAL(Mode::EmergencyStop, h->robot.mode());
  TEST_ASSERT_TRUE(h->robot.hasFault(Fault::ServoDriverReset));

  h->hold(Button::PlayStation, 2100);  // re-initialises the driver
  TEST_ASSERT_FALSE(h->robot.hasFault(Fault::ServoDriverReset));
  TEST_ASSERT_TRUE(h->runUntil([] { return h->robot.lifecycle() == Lifecycle::Active; }, 3000));
}

// ------------------------------------------------------- controller loss --

void test_controller_loss_sits_then_shuts_down() {
  TEST_ASSERT_TRUE(h->bootAndArm());
  standUp();
  h->pad.sticks(0, 1);
  h->runMs(1000);
  h->pad.disconnect();
  h->tick();
  TEST_ASSERT_TRUE(h->robot.hasFault(Fault::ControllerLost));
  h->waitIdle();
  TEST_ASSERT_EQUAL(Posture::Sitting, h->robot.motion().posture());
  TEST_ASSERT_TRUE(h->fake.anyOn());
  h->runMs(h->cfg.safety.controllerLostShutdownMs + 5000);
  TEST_ASSERT_EQUAL(Lifecycle::Shutdown, h->robot.lifecycle());
  TEST_ASSERT_FALSE(h->fake.anyOn());
}

void test_stale_controller_reports_count_as_lost() {
  TEST_ASSERT_TRUE(h->bootAndArm());
  h->pad.clock = nullptr;  // reports stop arriving, "connected" flag stays set
  h->runMs(h->cfg.input.staleTimeoutMs + 200);
  TEST_ASSERT_TRUE(h->robot.hasFault(Fault::ControllerLost));
}

// ----------------------------------------------------------------- timing --

void test_loop_overruns_raise_a_warning() {
  TEST_ASSERT_TRUE(h->bootAndArm());
  for (int i = 0; i < 60; ++i) {
    h->tick();
    h->robot.reportTiming(45000, 30000);
  }
  TEST_ASSERT_TRUE(h->robot.hasFault(Fault::LoopOverrun));
  TEST_ASSERT_EQUAL(Mode::Normal, h->robot.mode());  // warning only
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_outputs_stay_off_until_controller_connects);
  RUN_TEST(test_arming_enables_one_leg_at_a_time_into_sit_pose);
  RUN_TEST(test_options_hold_shuts_down_and_ps_hold_wakes);
  RUN_TEST(test_invalid_config_blocks_startup);
  RUN_TEST(test_left_dpad_estop_cuts_outputs_within_one_tick);
  RUN_TEST(test_estop_ignores_movement_and_mode_inputs);
  RUN_TEST(test_estop_clear_requires_ps_hold_of_2_seconds);
  RUN_TEST(test_dashboard_estop_and_clear);
  RUN_TEST(test_estop_is_never_dropped_from_a_full_queue);
  RUN_TEST(test_cross_toggles_sit_and_stand);
  RUN_TEST(test_left_stick_walks_and_deadzone_stops);
  RUN_TEST(test_full_stick_is_capped_at_90_percent_speed);
  RUN_TEST(test_directions_backward_left_right_and_turning);
  RUN_TEST(test_up_dpad_cycles_modes);
  RUN_TEST(test_right_dpad_switches_gait_in_normal_and_dance_in_dance_mode);
  RUN_TEST(test_triangle_resets_movement_defaults);
  RUN_TEST(test_circle_dances_and_square_attacks);
  RUN_TEST(test_dance_mode_circle_loops_until_pressed_again);
  RUN_TEST(test_sticks_and_buttons_are_logged);
  RUN_TEST(test_ik_mode_moves_front_right_foot_with_sticks);
  RUN_TEST(test_calibration_moves_to_reference_and_saves_trims);
  RUN_TEST(test_calibration_commands_refused_outside_calibration_mode);
  RUN_TEST(test_leaving_calibration_returns_to_stand);
  RUN_TEST(test_low_battery_sits_down_then_goes_limp);
  RUN_TEST(test_irregular_power_stops_immediately);
  RUN_TEST(test_low_battery_at_boot_refuses_to_arm);
  RUN_TEST(test_battery_disabled_reports_unavailable);
  RUN_TEST(test_missing_servo_driver_at_boot_is_critical);
  RUN_TEST(test_i2c_write_failures_trigger_estop);
  RUN_TEST(test_driver_reset_detected_and_recoverable);
  RUN_TEST(test_controller_loss_sits_then_shuts_down);
  RUN_TEST(test_stale_controller_reports_count_as_lost);
  RUN_TEST(test_loop_overruns_raise_a_warning);
  return UNITY_END();
}
