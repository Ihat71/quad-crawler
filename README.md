# QuadSpider

Firmware for a 12-servo quadruped robot (4 legs × 3 DOF). A PS5 controller connects over Bluetooth to an ESP32, which drives SG90 servos through a PCA9685 PWM board. Built with PlatformIO (Arduino framework).

```
PS5 controller ──Bluetooth──▶ ESP32 ──I2C──▶ PCA9685 ──PWM──▶ 12 × SG90
                                 │
                                 └──WiFi──▶ web dashboard (phone / PC)
```

- **Movement:** creep and trot gaits. Forward, backward, left/right strafe, turning and stop. The left stick sets speed, capped at 90% of the maximum.
- **Poses and dances:** sit, stand, up-and-down, left/right rocking, twist, bounce, wave, and an attack animation.
- **Modes:** NORMAL, IK (the front-right leg is driven by inverse kinematics), DANCE, CALIBRATION and EMERGENCY_STOP.
- **Safety:** a latched emergency stop, a hard 10°–170° servo angle limit, servo-driver fault detection, optional battery supervision, controller-loss failsafe, staggered safe start-up and a controlled shutdown.
- **Observability:** a WiFi dashboard and the serial log show health, status, mode, faults, movement history and PS5 button history.
- **Tests:** 77 unit tests run on the PC, and 6 hardware test firmwares run on the ESP32 (servos, legs, poses, battery, E-stop, error handling).

Design details are in [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

---

## 1. Hardware

| Part | Notes |
|---|---|
| ESP32 DevKit (**classic ESP32 / WROOM-32**) | Required. The DualSense uses Bluetooth Classic, which ESP32-S2/S3/C3 do not have. |
| PCA9685 16-ch PWM board | I2C address 0x40 |
| 12 × SG90 | Channels 0–11 (see the table below) |
| 2S Li-ion pack (7.4 V) + buck converter (5–6 V, ≥ 3 A) | Use a bench supply while testing |
| *(optional)* 100 kΩ / 33 kΩ voltage divider | Measures battery voltage on GPIO34 |
| *(optional)* wire from PCA9685 **OE** to a GPIO | Hardware kill switch for the E-stop (recommended) |

### Wiring (defaults in `config/robot_config.h`)

| Signal | ESP32 | PCA9685 |
|---|---|---|
| SDA | GPIO21 | SDA |
| SCL | GPIO22 | SCL |
| 3V3 / GND | 3V3 / GND | VCC / GND |
| Servo power | – | V+ ← buck converter output; GND shared with the ESP32 |
| OE (optional) | any free GPIO → set `pca.outputEnablePin` | OE |
| Battery divider (optional) | GPIO34 (ADC1) | – |

Battery divider: connect battery + → 100 kΩ → GPIO34 → 33 kΩ → GND, then set `battery.enabled = true`.

| Leg | Coxa | Femur | Tibia |
|---|---|---|---|
| FR front-right | ch 0 | ch 1 | ch 2 |
| FL front-left | ch 3 | ch 4 | ch 5 |
| RL rear-left | ch 6 | ch 7 | ch 8 |
| RR rear-right | ch 9 | ch 10 | ch 11 |

---

## 2. Getting started

1. Install [PlatformIO](https://platformio.org/install) (the VS Code extension or `pip install platformio`).
2. Copy `config/secrets.example.h` to `config/secrets.h` and fill it in:
   - `QUAD_WIFI_SSID` / `QUAD_WIFI_PASSWORD`: your WiFi network. Leave them empty to use the robot's own access point.
   - `QUAD_PS5_MAC`: see *Pairing the PS5 controller* below.
3. Measure your legs and set `geometry.*` in `config/robot_config.h`.
4. Flash the firmware and open the serial monitor:
   ```bash
   pio run -e esp32dev -t upload
   pio device monitor
   ```
5. Press **PS** on the controller. The robot arms one leg at a time into the sit pose. Press **X** to stand.

### Pairing the PS5 controller

The `ps5-esp32` library makes the ESP32 open the connection to the controller, so `QUAD_PS5_MAC` is the **controller's own** Bluetooth MAC address, written with colons (`aa:bb:cc:dd:ee:ff`).

1. Pair the DualSense with a phone or PC: hold **Create + PS** until the light flashes fast, then pair it in the Bluetooth settings.
2. Read the controller's MAC from the paired device's details (Android: Bluetooth → ⚙ next to "Wireless Controller"; Windows: Device Manager → the controller → Properties → Details → "Bluetooth device address") and put it in `QUAD_PS5_MAC`.
3. Unpair the controller from the phone or PC, or switch its Bluetooth off, then press **PS**. The controller connects to the robot.

The lightbar shows the mode: green NORMAL, blue IK, magenta DANCE, yellow CALIBRATION, red E-STOP, dim white while the outputs are off, and orange for low battery.

---

## 3. Controls

| Input | NORMAL | IK (front-right leg) | DANCE | CALIBRATION |
|---|---|---|---|---|
| **D-pad ↑** | next mode | next mode | next mode | next mode |
| **D-pad ←** | **EMERGENCY STOP** (any mode) | ← | ← | ← |
| **D-pad →** | next gait (creep / trot) | – | next dance | – |
| **✕** | sit ↔ stand | – | sit ↔ stand | save calibration |
| **△** | reset movement defaults | reset foot target | reset defaults | zero the selected trim |
| **○** | play the selected dance once | – | start/stop the dance loop | wiggle the selected servo |
| **□** | attack animation | – | attack animation | – |
| **Left stick** | walk direction + speed (deadzone = stop) | foot forward/back, left/right | – | ↑/↓ trims the selected servo |
| **Right stick** | X: turn | Y: foot up/down | – | – |
| **L1 / R1** | – | – | – | previous / next servo |
| **Hold PS 2 s** | clear E-stop / wake from shutdown | | | |
| **Hold Options 2 s** | controlled shutdown (sit, then outputs off) | | | |

Mode cycle: NORMAL → IK → DANCE → CALIBRATION → NORMAL. Every mapping is set in `input.map` in the config.

---

## 4. Lifecycle and safety

```
BOOTING → SELF_TEST → WAITING_FOR_CONTROLLER → ARMING → ACTIVE ⇄ STOPPING → SHUTDOWN
                         (outputs OFF)          (leg by leg)          (sit)     (outputs OFF)
```

**Safe start-up**
- The PCA9685 powers up with every output off.
- The self-test checks the configuration (channel map, 10–170° limits, reachability of every pose and gait), the stored calibration, the PCA9685 and the battery.
- The servos stay off until a controller connects, or until you send `arm` from the dashboard.
- Arming enables one leg every 250 ms into the sit pose. This limits inrush current, because SG90s jump to their first command.

**Safe shutdown**
- Hold **Options** for 2 s, or use the dashboard or the `shutdown` command.
- The robot stops, sits, then switches all outputs off and puts the PCA9685 to sleep. It is then safe to switch the power off.
- Hold **PS** to wake it.

**Emergency stop**
- Triggers: D-pad ←, the dashboard button (or the **Esc** key on the dashboard page), the `estop` command, or any critical fault.
- All PWM outputs are cut within one 20 ms tick, plus the OE pin if it is wired. The robot goes limp.
- The E-stop is latched. Clear it by holding **PS** for 2 s or with *Clear E-stop* on the dashboard. Clearing re-initialises the servo driver, re-checks faults and re-arms into the sit pose.

| Fault | Severity | Detection | Action |
|---|---|---|---|
| `SERVO_DRIVER_INIT` | critical | PCA9685 does not answer at boot or on re-init | E-stop, refuse to arm |
| `SERVO_DRIVER_COMM` | critical | 3 consecutive I2C write failures | E-stop |
| `SERVO_DRIVER_RESET` | critical | Register readback shows the chip reset (brown-out on the servo rail) | E-stop |
| `SERVO_READBACK` | critical | A channel does not hold the value that was written | E-stop |
| `BATTERY_CRITICAL` | critical | Below 6.6 V for 2 s | Sit, then E-stop |
| `BATTERY_UNSTABLE` | critical | Swings of more than 1.2 V within 1 s ("irregular power") | Immediate E-stop |
| `BATTERY_OVERVOLTAGE` | critical | Above 8.8 V | Immediate E-stop |
| `CONFIG_INVALID` | critical | Self-test | E-stop, cannot be cleared |
| `BATTERY_LOW` | warning | Below 7.0 V | Orange lightbar; refuses to arm below 7.0 V |
| `BATTERY_SENSOR` | warning | Implausible ADC reading | Reported |
| `CONTROLLER_LOST` | warning | Disconnected, or no reports for 1 s | Stop and sit; outputs off after 60 s |
| `KINEMATICS` | warning | Unreachable foot target | Holds the last joint angles |
| `LOOP_OVERRUN` | warning | Control loop misses its 20 ms deadline | Reported |

The battery checks are active only when `battery.enabled = true`. The divider is optional and disabled by default. Without it, the dashboard shows the battery as "not monitored".

Servo angles are clamped to **10°–170°** in software after the calibration trim is applied. Configuring wider limits requires `servoLimits.allowLimitOverride = true`.

---

## 5. Dashboard and serial console

The dashboard is at `http://quadspider.local/`, or at the IP printed on the serial log. Without WiFi credentials, join the access point **QuadSpider** (password `quadspider`) and open `http://192.168.4.1/`.

It shows:
- lifecycle, mode, posture, gait and dance
- health: battery, controller, loop timing, I2C errors, angle clamps, heap and WiFi
- active faults and faults seen since boot
- a live top view of the legs
- each servo's output, target and trim, with calibration controls
- the event log, movement history and PS5 button history
- mode, posture, dance, arm/clear/shutdown buttons and a command line

The serial monitor (115200 baud) prints the same logs and accepts the same commands. Type `help`:

```
estop | clear | arm | shutdown
mode <normal|ik|dance|cal>
sit | stand | toggle | stop | reset | attack
dance [next|<name>|<index>] [loop]      gait <next|index>
cal select <servo> | cal trim <servo> <deg> | cal nudge <servo> <deg> | cal save | cal reset [<servo>|all]
log <debug|info|warn|error> | status | help
```

---

## 6. Calibration

1. Put the robot on a stand. Arm it, then switch to **CALIBRATION** with D-pad ↑ ×3 or the dashboard.
2. Every joint moves to its reference pose: coxa straight out, femur horizontal, tibia vertical (servo 90° + trim).
3. Select a servo with **L1/R1** or the dashboard row. It wiggles so you can identify it.
4. Trim it with left stick ↑/↓ (0.5° steps) or the ± buttons on the dashboard until the joint matches the reference.
5. Press **✕** (or *Save calibration*) to store the trims in flash. They load automatically at boot.

If a joint moves the wrong way in `test_legs`, flip its `direction` in `config/robot_config.h`. Calibration only corrects offsets, not direction.

---

## 7. Tests

```bash
pio test -e native                                 # 77 logic tests on the PC, no hardware needed
pio test -e esp32dev                               # every hardware suite on the ESP32
pio test -e esp32dev -f hardware/test_estop        # one hardware suite
```

| Suite | Covers |
|---|---|
| `native/test_kinematics` | IK/FK round trips, transforms, servo mapping, reachability of every animation keyframe |
| `native/test_safety` | 10–170° guard and override, trim and slew limits, battery thresholds, hold times, ripple and over-voltage |
| `native/test_motion` | creep/trot phasing, smooth start and stop, sit/stand, all dances, IK leg workspace |
| `native/test_robot` | the full robot with fake hardware: start-up, E-stop, controller mapping, modes, calibration, battery, driver faults, controller loss, overruns |
| `native/test_telemetry` | shipped config validity, log ring buffers, dashboard JSON, command parser |
| `hardware/test_servos` | **each servo** moved alone, verified by PCA9685 readback; pulse math; all-off |
| `hardware/test_legs` | **each leg** traces up/out/forward with IK (use it to check joint directions) |
| `hardware/test_poses` | **standing and sitting** (plus up-down and rocking) on the real servos |
| `hardware/test_battery` | **low battery**: simulated discharge (sit, then outputs off); bench-supply undervoltage with the real divider |
| `hardware/test_estop` | **emergency stop**: outputs off within one tick, inputs ignored, 2 s PS hold to clear |
| `hardware/test_errors` | **error handling**: PCA9685 reset (brown-out) detection and recovery, I2C failures, controller loss, loop timing on the target |

The hardware suites live in `test/hardware/`. Each one is its own test firmware that PlatformIO uploads to the ESP32. Put the robot on a stand for `test_servos` and `test_legs`. The PS5 controller is not needed: button presses are scripted.

---

## 8. Project layout

```
config/            robot_config.h (all tunables), secrets.example.h
dashboard/         index.html (embedded into the firmware at build time)
lib/quad_core/     platform-independent logic (kinematics, gaits, dances, safety, modes, telemetry)
lib/quad_esp32/    ESP32 layer: PCA9685, PS5, ADC, NVS, WiFi dashboard, serial console, FreeRTOS runtime
lib/quad_test_support/  fake hardware and a scriptable robot harness for tests
src/main.cpp       composition root
test/native/       PC unit tests
test/hardware/     ESP32 test firmware
docs/              architecture notes
```

## 9. Before the first real run

The defaults in `config/robot_config.h` are **placeholders until you measure your robot**:
- leg segment lengths and mount positions
- the servo direction of each joint (check with `test_legs`)
- the SG90 pulse range, 500–2400 µs
- the battery divider ratio, if you fit one

Start on a bench supply with the robot on a stand. Run `test_servos` and `test_legs`, calibrate, then try `test_poses` on the floor.
