/*
  =====================================================================
  SELF-BALANCING ROBOT — ESP32 + MPU6050 + TB6612FNG + 2x GA25-370 (encoders)
  =====================================================================

  Control architecture:
    - Inner loop (fast, ~every 5 ms): complementary filter fuses MPU6050
      accel + gyro into a pitch angle estimate. An angle PID converts
      angle error into a motor PWM command that drives the robot to
      stay upright.
    - Outer loop (slower, every ~50 ms): reads wheel encoder ticks,
      estimates robot speed, and runs a velocity PID whose output is a
      small *target angle offset*. This is what keeps the robot from
      slowly drifting off and lets it hold position / creep to stop
      instead of running away.

  This is the same "cascaded PID" architecture used by most hobby
  self-balancing robots (angle loop nested inside a velocity loop).

  ---------------------------------------------------------------------
  WIRING (see README.md for the full table)
  ---------------------------------------------------------------------
  MPU6050:      SDA -> GPIO21   SCL -> GPIO22   VCC -> 3V3   GND -> GND
  TB6612FNG:    PWMA -> GPIO25  AIN1 -> GPIO27  AIN2 -> GPIO26 (Left motor)
                PWMB -> GPIO14  BIN1 -> GPIO33  BIN2 -> GPIO32 (Right motor)
                STBY -> GPIO13
  Left encoder:  A -> GPIO34    B -> GPIO35
  Right encoder: A -> GPIO36    B -> GPIO39

  NOTE: GPIO34-39 are input-only on ESP32 and have NO internal pull
  resistors. GA25-370 hall encoders have push-pull outputs so this is
  fine, but if you see noisy counts, add external 10k pull-ups.
  ---------------------------------------------------------------------
*/

#include <Arduino.h>
#include <Wire.h>

// ---------------------- USER-TUNABLE PARAMETERS ----------------------

// Angle (inner) PID gains — tune these FIRST, robot propped upright
float angleKp = 18.0;
float angleKi = 140.0;
float angleKd = 0.55;

// Velocity (outer) PID gains — tune AFTER angle loop is stable
float velKp = 0.9;
float velKi = 3.0;
float velKd = 0.0;

// The angle (degrees) at which the robot balances when perfectly
// still. Depends on MPU6050 mounting and center-of-mass offset.
// Calibrate by hand-balancing the robot and reading Serial Plotter,
// or use the auto zero routine below.
float balanceAngleOffset = 0.0;

// Safety cutoff: if the estimated tilt exceeds this, robot has
// fallen — motors are stopped so it doesn't spin its wheels.
const float FALL_ANGLE = 35.0;

// Complementary filter blend (closer to 1.0 = trust gyro more)
const float COMP_FILTER_ALPHA = 0.98;

// Encoder: pulses per output-shaft revolution for GA25-370.
// Common variants: 11 PPR magnet x gearbox ratio x 4 (quadrature).
// Adjust GEAR_RATIO / PPR_PER_MOTOR_REV to match your exact motor.
const float PPR_PER_MOTOR_REV = 11.0;   // hall pulses per motor-shaft rev
const float GEAR_RATIO = 90.0;          // GA25-370 common ratios: 30/90/150/210...
const float COUNTS_PER_WHEEL_REV = PPR_PER_MOTOR_REV * GEAR_RATIO * 4.0; // x4 quadrature

// ---------------------- PIN DEFINITIONS ----------------------

// MPU6050
#define SDA_PIN 21
#define SCL_PIN 22
#define MPU6050_ADDR 0x68

// TB6612FNG
#define PWMA 25
#define AIN1 27
#define AIN2 26
#define PWMB 14
#define BIN1 33
#define BIN2 32
#define STBY 13

// Encoders
#define ENCA_A 34
#define ENCA_B 35
#define ENCB_A 36
#define ENCB_B 39

// PWM (LEDC) config
const int PWM_FREQ = 20000;   // 20 kHz, above audible range
const int PWM_RES  = 8;       // 8-bit -> 0-255

// ---------------------- STATE VARIABLES ----------------------

float pitchAngle = 0.0;
float gyroRateY = 0.0;

volatile long encoderCountLeft = 0;
volatile long encoderCountRight = 0;

unsigned long lastInnerLoopTime = 0;
unsigned long lastOuterLoopTime = 0;

// Angle PID state
float angleIntegral = 0.0;
float lastAngleError = 0.0;

// Velocity PID state
float velIntegral = 0.0;
float lastVelError = 0.0;
float targetAngleTrim = 0.0; // output of velocity loop, added to setpoint

bool robotFallen = false;

// ---------------------- ENCODER ISRs ----------------------
// Simple x2 quadrature decode (count on A edges, direction from B).
// Good enough for velocity estimation on a balancing robot.

void IRAM_ATTR isrEncoderLeft() {
  bool a = digitalRead(ENCA_A);
  bool b = digitalRead(ENCA_B);
  if (a == b) encoderCountLeft++;
  else encoderCountLeft--;
}

void IRAM_ATTR isrEncoderRight() {
  bool a = digitalRead(ENCB_A);
  bool b = digitalRead(ENCB_B);
  // Right motor is mounted mirrored, so direction sense is flipped
  if (a == b) encoderCountRight--;
  else encoderCountRight++;
}

// ---------------------- MPU6050 LOW-LEVEL I/O ----------------------

void mpu6050Write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void mpu6050Init() {
  mpu6050Write(0x6B, 0x00); // wake up (clear sleep bit)
  mpu6050Write(0x1A, 0x03); // DLPF ~44Hz, reduces vibration noise
  mpu6050Write(0x1B, 0x00); // gyro full scale +-250 deg/s
  mpu6050Write(0x1C, 0x00); // accel full scale +-2g
}

// Reads accel + gyro, returns pitch estimate from accel (deg) and
// gyro Y rate (deg/s) via reference parameters.
void mpu6050Read(float &accelPitch, float &gyroY) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // temp, skip
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // gz, skip (not needed for pitch)

  // Convert accel to pitch angle (deg). Adjust axes/sign to match
  // how the MPU6050 is physically mounted on your chassis.
  float accelPitchRad = atan2(-ax, sqrt((float)ay * ay + (float)az * az));
  accelPitch = accelPitchRad * 180.0 / PI;

  // Gyro Y in deg/s (sensitivity 131 LSB/(deg/s) at +-250 scale)
  gyroY = gy / 131.0;
}

// ---------------------- MOTOR DRIVER ----------------------

// speed: -255..255 (negative = reverse)
void setMotor(int pwmPin, int in1Pin, int in2Pin, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    speed = -speed;
  }
  ledcWrite(pwmPin, speed);
}

void stopMotors() {
  ledcWrite(PWMA, 0);
  ledcWrite(PWMB, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

// ---------------------- SETUP ----------------------

void setup() {
  Serial.begin(115200);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  mpu6050Init();

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH); // enable driver

  ledcAttach(PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PWMB, PWM_FREQ, PWM_RES);

  pinMode(ENCA_A, INPUT);
  pinMode(ENCA_B, INPUT);
  pinMode(ENCB_A, INPUT);
  pinMode(ENCB_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA_A), isrEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB_A), isrEncoderRight, CHANGE);

  // Initialize pitch angle with a few accel-only readings so we don't
  // start at 0 and cause a jerk.
  float ap, gy;
  mpu6050Read(ap, gy);
  pitchAngle = ap;

  lastInnerLoopTime = micros();
  lastOuterLoopTime = millis();

  Serial.println("Self-balancing robot ready. Format: angle,targetAngle,pwmOut");
}

// ---------------------- MAIN LOOP ----------------------

void loop() {
  unsigned long now = micros();
  float dt = (now - lastInnerLoopTime) / 1000000.0;

  // Run inner loop at ~200 Hz (every 5 ms)
  if (dt >= 0.005) {
    lastInnerLoopTime = now;
    innerLoop(dt);
  }

  // Run outer (velocity) loop at ~20 Hz (every 50 ms)
  if (millis() - lastOuterLoopTime >= 50) {
    float outerDt = (millis() - lastOuterLoopTime) / 1000.0;
    lastOuterLoopTime = millis();
    outerLoop(outerDt);
  }
}

// ---------------------- INNER LOOP: angle estimate + angle PID ----------------------

void innerLoop(float dt) {
  float accelPitch, gyroY;
  mpu6050Read(accelPitch, gyroY);
  gyroRateY = gyroY;

  // Complementary filter: combine gyro (fast, drifts) with accel (slow, noisy)
  pitchAngle = COMP_FILTER_ALPHA * (pitchAngle + gyroY * dt)
             + (1.0 - COMP_FILTER_ALPHA) * accelPitch;

  // Fall detection / safety cutoff
  if (fabs(pitchAngle) > FALL_ANGLE) {
    if (!robotFallen) {
      robotFallen = true;
      stopMotors();
      angleIntegral = 0; // reset so it doesn't wind up while down
      velIntegral = 0;
      Serial.println("FALLEN — motors stopped. Stand it back up to resume.");
    }
    return;
  } else if (robotFallen && fabs(pitchAngle) < 5.0) {
    robotFallen = false; // recovered, resume balancing
  }
  if (robotFallen) return;

  // Setpoint = calibrated balance angle + trim from the velocity loop
  float setpoint = balanceAngleOffset + targetAngleTrim;
  float error = setpoint - pitchAngle;

  angleIntegral += error * dt;
  angleIntegral = constrain(angleIntegral, -300, 300); // anti-windup clamp
  float derivative = (error - lastAngleError) / dt;
  lastAngleError = error;

  float output = angleKp * error + angleKi * angleIntegral * 0.001 + angleKd * derivative;
  output = constrain(output, -255, 255);

  int pwmOut = (int)output;
  setMotor(PWMA, AIN1, AIN2, pwmOut);
  setMotor(PWMB, BIN1, BIN2, pwmOut);

  // Uncomment for tuning with Serial Plotter:
  // Serial.print(pitchAngle); Serial.print(",");
  // Serial.print(setpoint); Serial.print(",");
  // Serial.println(pwmOut);
}

// ---------------------- OUTER LOOP: encoder velocity PID ----------------------

void outerLoop(float dt) {
  if (robotFallen) return;

  noInterrupts();
  long left = encoderCountLeft;
  long right = encoderCountRight;
  encoderCountLeft = 0;
  encoderCountRight = 0;
  interrupts();

  // Wheel revolutions this interval -> average speed in rev/s
  float leftRevs = left / COUNTS_PER_WHEEL_REV;
  float rightRevs = right / COUNTS_PER_WHEEL_REV;
  float avgRevPerSec = ((leftRevs + rightRevs) / 2.0) / dt;

  // We want the robot to hold position, so target speed = 0.
  // (To drive it remotely, set targetSpeed from a joystick/serial command.)
  float targetSpeed = 0.0;
  float velError = targetSpeed - avgRevPerSec;

  velIntegral += velError * dt;
  velIntegral = constrain(velIntegral, -10, 10);
  float velDerivative = (velError - lastVelError) / dt;
  lastVelError = velError;

  // Output becomes a small trim added to the angle setpoint. Keep it
  // small — this loop should nudge, not dominate, the angle loop.
  targetAngleTrim = velKp * velError + velKi * velIntegral + velKd * velDerivative;
  targetAngleTrim = constrain(targetAngleTrim, -8.0, 8.0);
}
