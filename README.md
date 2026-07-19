# Self-Balancing Robot — ESP32 + MPU6050 + TB6612FNG + GA25-370

## Project structure (PlatformIO)

```
self_balancing_robot/
├── platformio.ini      # build config (board, framework, monitor speed)
├── src/
│   └── main.cpp        # the full sketch
└── README.md
```

### Building/uploading

1. Install [PlatformIO](https://platformio.org/) (either the standalone core, or the VS Code extension).
2. Open this folder as a PlatformIO project (VS Code: "Open Folder", it auto-detects `platformio.ini`).
3. In `platformio.ini`, `board` is set to `esp32doit-devkit-v1`. If your board is different, change it — common alternatives:
   - `esp32dev` (generic DevKitC-style board)
   - `esp32-s3-devkitc-1` (ESP32-S3)
   - `nodemcu-32s`
   - `wemos_d1_mini32`
   Run `pio boards esp32` to list all supported ESP32 boards if you're not sure which matches yours.

   Note: the code uses the *channel-based* LEDC PWM API (`ledcSetup`/`ledcAttachPin`/`ledcWrite(channel, ...)`), which matches Arduino-ESP32 core 2.x (what PlatformIO's `espressif32` platform currently installs by default). If you ever upgrade to core 3.x, that version uses a different pin-based API (`ledcAttach`/`ledcWrite(pin, ...)`) and the PWM setup lines would need updating.
4. Build and upload:
   ```bash
   pio run              # build
   pio run -t upload    # build + flash
   pio device monitor   # serial monitor (115200 baud, matches platformio.ini)
   ```
   Or in VS Code, use the PlatformIO toolbar icons (checkmark = build, arrow = upload, plug = monitor).

## Parts list

- ESP32 dev board (any variant with enough GPIO)
- 2x GA25-370 DC gear motors with Hall-effect quadrature encoders
- TB6612FNG dual motor driver breakout
- MPU6050 (accelerometer + gyroscope), I2C
- 2S or 3S LiPo (7.4–11.1V) sized for the motors, or a battery pack matching your GA25-370's rated voltage
- 5V regulator/BEC for ESP32 + logic (unless powering ESP32 via USB during bench testing)
- Chassis, wheels, wiring, switch, fuse

## Wiring

### MPU6050 (I2C)
| MPU6050 | ESP32 |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO21 |
| SCL | GPIO22 |

### TB6612FNG
| TB6612FNG | ESP32 | Notes |
|---|---|---|
| PWMA | GPIO25 | Left motor speed |
| AIN1 | GPIO27 | Left motor direction |
| AIN2 | GPIO26 | Left motor direction |
| PWMB | GPIO14 | Right motor speed |
| BIN1 | GPIO33 | Right motor direction |
| BIN2 | GPIO32 | Right motor direction |
| STBY | GPIO13 | Must be HIGH to enable driver |
| VM | Battery+ | Motor supply (7–15V, matches your battery) |
| VCC | 3V3 or 5V | Logic supply |
| GND | GND | Common ground with ESP32 and battery |
| AO1/AO2 | Left motor leads | |
| BO1/BO2 | Right motor leads | |

### Encoders
| Encoder | ESP32 |
|---|---|
| Left channel A | GPIO34 |
| Left channel B | GPIO35 |
| Right channel A | GPIO36 |
| Right channel B | GPIO39 |
| Encoder VCC | 3V3 (check your encoder's rated voltage — GA25-370 hall encoders are usually 3.3–5V tolerant, confirm on your datasheet) |
| Encoder GND | GND |

GPIO34–39 are input-only on the ESP32 and have no internal pull resistors. GA25-370 hall sensors are push-pull outputs so this is normally fine; add external 10k pull-ups to 3.3V if you see erratic counts.

**Important:** All grounds (ESP32, TB6612FNG, battery, encoders) must be tied together (common ground), or the MPU6050/logic signals will be noisy or unreliable.

## Mechanical notes

- Mount the MPU6050 as close to the robot's pitch axis (the axle) as possible, and keep it rigid — any flex or vibration corrupts the angle reading.
- The two wheels/motors should be on the same axle line so the robot's balance point is symmetric.
- Keep the center of mass as high as reasonably possible above the wheel axle — this is counterintuitive but makes balancing physically easier (slower fall dynamics), similar to balancing a broomstick vs. a pencil.

## How the code works

The sketch uses a cascaded PID structure:

1. **Angle loop (inner, ~200 Hz):** A complementary filter combines the MPU6050 accelerometer angle (accurate long-term, noisy short-term) with the gyroscope rate (accurate short-term, drifts long-term) into a stable pitch estimate. A PID controller compares this to a setpoint and outputs a motor PWM command to correct any tilt.
2. **Velocity loop (outer, ~20 Hz):** Reads encoder ticks from both wheels, estimates the robot's actual speed, and runs a second PID whose output is a *small trim added to the angle setpoint*. This is what stops the robot from slowly rolling away — without it, a pure angle PID will balance but drift because the "zero angle" point isn't exactly where the robot is stationary.
3. **Fall detection:** If the pitch angle exceeds ±35°, the robot considers itself fallen, cuts motor power, and waits until you stand it back up (angle returns under 5°) before resuming.

## First-time bring-up checklist

1. Upload the sketch and open Serial Monitor at 115200 baud.
2. Lay the robot on its side and confirm you see `Self-balancing robot ready`.
3. Prop the robot upright by hand and check the wheels don't run away immediately — if they spin at full speed instantly, either the accel sign is flipped (see `mpu6050Read`, flip the `-ax` sign) or a wire is swapped on the driver.

## PID tuning procedure

Tune the **angle loop first**, with `velKp/Ki/Kd = 0` (comment out or zero them) and the robot held/propped upright:

1. Set `angleKi = 0`, `angleKd = 0`. Slowly raise `angleKp` until the robot pushes back against a tilt and starts to oscillate around vertical. Back off to about 60–70% of that value.
2. Slowly raise `angleKd` to damp the oscillation (it should stop wobbling and settle faster). Too much `Kd` makes it twitchy/jittery.
3. Slowly raise `angleKi` until the robot holds a fixed lean angle without slowly sagging. Too much causes a slow oscillation.
4. Only once the robot balances but drifts across the floor, enable the **velocity loop** (`velKp/Ki`) to correct the drift. Start with small values (`velKp` ~0.5–1.0) and increase gradually — too high makes the robot rock back and forth.

Use `Serial.println` (uncomment the plotting line in `innerLoop`) with the Arduino Serial Plotter to visualize `pitchAngle` vs `setpoint` while tuning — it makes oscillation/overshoot much easier to see than watching the robot.

## Calibrating `balanceAngleOffset`

Because of how the MPU6050 sits and the robot's center of mass, "vertical" (0°) on the sensor usually isn't the exact angle where the robot balances. After the angle loop is stable:
1. Balance the robot by hand and note the `pitchAngle` printed over serial when it's stationary and upright.
2. Set `balanceAngleOffset` to that value and re-upload.

## Known simplifications / next steps

- Encoder decoding is x2 (edge on channel A, direction from B), not full x4 quadrature — plenty for velocity feedback, but note if you want higher-resolution odometry.
- No turning/remote control input yet — `targetSpeed` in `outerLoop()` is hardcoded to 0 (hold position). To drive it, feed a target speed/turn command from a joystick, Bluetooth, or Wi-Fi input and differentiate left/right PWM for turning.
- No motor deadband compensation — very small PWM values may not overcome motor static friction. If the robot "buzzes" near zero without moving, add a minimum PWM threshold.