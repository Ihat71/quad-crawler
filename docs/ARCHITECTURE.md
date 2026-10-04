# QuadSpider architecture

## Layers

```
┌──────────────────────────────────────────────────────────────────────┐
│ src/main.cpp            composition root: builds drivers + Robot      │
├──────────────────────────────────────────────────────────────────────┤
│ lib/quad_esp32          hardware & platform (Arduino / FreeRTOS)      │
│   Pca9685ServoDriver  Ps5Gamepad  AdcPowerSensor  NvsCalibrationStore │
│   DashboardServer  SerialConsole  RobotRuntime (tasks)                │
├────────────── implements hal/ interfaces ────────────────────────────┤
│ lib/quad_core           pure C++17, no Arduino; runs on PC and ESP32  │
│   robot/      Robot (lifecycle, modes, commands, supervision)         │
│   motion/     MotionController, GaitEngine, Animations (dances)       │
│   kinematics/ LegKinematics (3-DOF IK/FK), BodyKinematics             │
│   servo/      ServoBus (trim → 10..170 guard → slew → channel map)    │
│   safety/     BatteryMonitor, Faults                                  │
│   input/      InputMapper (edges, long-press, deadzone)               │
│   telemetry/  EventLog (ring buffers), TelemetryJson, JsonWriter      │
│   hal/        IServoDriver IGamepad IPowerSensor ICalibrationStore    │
├──────────────────────────────────────────────────────────────────────┤
│ config/robot_config.h   every tunable value (wiring, geometry, gaits, │
│                         mapping, thresholds, timing)                  │
└──────────────────────────────────────────────────────────────────────┘
```

- **Hardware code is separate from robot logic.** `quad_core` only knows the four `hal/` interfaces. That is why the whole robot runs in PC unit tests against fakes (`lib/quad_test_support`).
- **Configuration is separate from logic.** `RobotConfig` (in core) only declares the structure. `config/robot_config.h` holds every value, and logic only reads it. Calibration trims are runtime data in NVS.
- The namespace is `qspider`. `quad` cannot be used because ESP32 newlib `#define`s it.

## One control tick (20 ms, `Robot::tick`)

1. Read the gamepad, then `InputMapper` produces press, release and long-press edges and deadzone-filtered sticks. Presses go into the button history.
2. The **E-stop button is checked first**, before any other processing.
3. Commands queued by the dashboard or serial console are drained. Both use the same command language as the controller path.
4. Supervision: controller link, battery (every 100 ms when enabled) and driver health.
5. Mode-specific input handling: NORMAL, IK, DANCE or CALIBRATION.
6. Lifecycle sequencing: arming leg by leg, stopping, shutdown.
7. `MotionController` produces body-frame foot targets. `BodyKinematics` converts them to joint and servo angles.
8. `ServoBus` applies trim, then the angle guard, then the slew limit, and writes all 16 channels in **one I2C burst**. Disabled channels are written fully off.
9. Driver checks: write errors every tick; register verify and readback of one channel per second.
10. A snapshot for telemetry is copied under a mutex.

## Threads (ESP32)

| Task | Core | Priority | Work |
|---|---|---|---|
| `control` | 1 | high | `vTaskDelayUntil` every 20 ms → `Robot::tick`; timing is reported for overrun detection; supervised by the task watchdog |
| `service` | 0 | 1 (low) | serial log sink + console, WiFi/HTTP dashboard, lightbar and rumble feedback |
| BT / WiFi stacks | 0 | system | ps5-esp32 callbacks, lwIP |

The tasks share only three things, all mutex-protected with short critical sections and no I/O while locked:
- `Robot::submit` (command inbox; an E-stop is never dropped)
- `Robot::snapshot` (copy of the state)
- the `EventLog` rings (fixed-size, no heap)

Logging in the control loop is just a `vsnprintf` into a ring slot. Printing and HTTP happen on the other core at low priority, so the dashboard and logging cannot delay servo updates (the *Performance* requirement).

## Motion

- **Postures:** sit and stand are neutral foot positions (`reach`, `height`) per leg. Transitions are smoothstep tweens of all four feet.
- **Gaits:** `GaitEngine` is phase-based, and any `GaitConfig` works: duty factor, per-leg phase offsets, stride, step height and turn. In stance a foot moves from +D/2 to −D/2; in swing it moves back with a sine lift. D combines translation and rotation about the body centre. Creep adds a body sway away from the leg that is about to lift.
  - Speed is the stick magnitude × `maxSpeedFraction` (0.9) of the gait's maximum cycle rate.
  - Commands are rate-limited. When the stick returns to the deadzone, the feet settle back to the stand pose.
- **Animations** are keyframes of `BodyPose` (translation + roll/pitch/yaw) plus per-foot offsets relative to the stand pose. To add a dance, add a table in `motion/Animations.cpp` and list it in `kDances`. The dashboard and command parser pick it up automatically, and `native/test_kinematics` checks that every keyframe is reachable.
- **IK mode:** the body first shifts away from the selected leg, then the foot lifts. The sticks then move the foot target inside a configured box. Each target is checked for reachability and servo limits before it is accepted.

## Extending

| Change | Where |
|---|---|
| New gait | Add a `GaitConfig` in `config/robot_config.h` and increase `gaitCount` |
| New dance | `lib/quad_core/src/motion/Animations.cpp` |
| Remap buttons | `input.map` in the config |
| Different controller library (e.g. Bluepad32) | Implement `IGamepad` in `lib/quad_esp32`, then swap it in `src/main.cpp` |
| Current sensor (INA219) for real servo-stall detection | Implement `IPowerSensor` (or a new interface) and feed it into the supervision in `Robot` |
| Different servo board | Implement `IServoDriver` |
| Wider servo limits | `servoLimits.minDeg/maxDeg` + `allowLimitOverride = true` |

## Requirement traceability

| Requirement | Implementation | Verified by |
|---|---|---|
| 4 legs × 3 DOF | `kLegCount`, `kJointsPerLeg`, `LegKinematics` | `test_kinematics`, `hardware/test_legs` |
| Emergency stop | `Robot::emergencyStop`, D-pad ←, dashboard, OE pin | `test_robot`, `hardware/test_estop` |
| PlatformIO | `platformio.ini` | – |
| Forward, backward, left/right, turning, stop | `GaitEngine`, `Robot::sticksToWalk` | `test_motion`, `test_robot` |
| Stop on irregular battery power | `BatteryMonitor` (low/critical/ripple/over-voltage) | `test_safety`, `test_robot`, `hardware/test_battery` |
| Stop on servo malfunction | I2C failures, PCA9685 reset detection, channel readback | `test_robot`, `hardware/test_errors` |
| Stop on D-pad left | `ControllerMapping::emergencyStop` | `test_robot`, `hardware/test_estop` |
| Servo angles limited to 10°–170° unless overridden | `AngleGuard`, `ServoBus`, `ConfigValidator` | `test_safety`, `test_telemetry` |
| Servo calibration | CALIBRATION mode, `NvsCalibrationStore`, dashboard | `test_robot` |
| Sit, stand, up-and-down, rocking/dancing | `MotionController`, `Animations.cpp` | `test_motion`, `hardware/test_poses` |
| Modular and evolvable | layered libs, HAL interfaces, data-driven gaits and dances | – |
| PS5 mapped to movement commands | `InputMapper`, `Robot::handle*` | `test_robot` |
| Left stick speed up to about 90% | `motion.maxSpeedFraction = 0.9` | `test_robot` |
| IK mode on the front-right leg | `MotionController::enterIk/moveIk`, `motion.ik.leg` | `test_motion`, `test_robot` |
| Observability (health, status, mode, movements, button history) | `EventLog`, `RobotSnapshot`, dashboard, serial | `test_telemetry` |
| Dashboard | `DashboardServer` + `dashboard/index.html` | `test_telemetry` (JSON) |
| Safe start-up and shutdown | lifecycle, staggered arming, sit-then-off | `test_robot`, `hardware/test_poses` |
| Config separate from movement logic | `config/robot_config.h` | – |
| Errors detected, safe state on critical failure | `Faults`, `raiseCritical` | `test_robot`, `hardware/test_errors` |
| Consistent servo rate, low latency, logging isolated | `RobotRuntime` tasks, a single I2C burst per tick | `hardware/test_errors` (tick timing) |
| Separate test firmware folder | `test/hardware/*` | – |
