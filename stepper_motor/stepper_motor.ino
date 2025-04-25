/*************************************************************
 * Project:  Motorized Capo System for Guitar Fretboard
 * Hardware: Arduino Uno + Adafruit Motor Shield V2
 * Control:  Analog Joystick (X-axis)
 * Motion:   Stepper motor + GT2 belt for linear movement
 * Authors:  Matthew & Pat
 *************************************************************/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// ----------------------------------------------------
// CONFIGURATION SECTION
// ----------------------------------------------------

// Joystick
#define JOYSTICK_X A0                // Analog input pin for X-axis
#define JOYSTICK_CENTER 512          // Center value of joystick
#define DEADZONE_THRESHOLD 100       // Wider deadzone to prevent drift

// Motion Limits (in stepper motor steps)
#define MOTION_LOWER_BOUND 0         // Leftmost position (e.g., fret 0)
#define MOTION_UPPER_BOUND 2000      // Rightmost position (e.g., max fret)

// Motor Behavior
#define MOTOR_MIN_SPEED 40           // Slowest RPM (below this may stall)
#define MOTOR_MAX_SPEED 100          // Fastest RPM (more speed = less torque)
#define STEP_DELAY_MS 5              // Delay between steps (fine-tune for smoothness)
#define STEP_MODE DOUBLE             // Options: SINGLE, DOUBLE, INTERLEAVE, MICROSTEP

// Debugging
#define ENABLE_DEBUG true            // Set to false to disable Serial prints

// Auto-Sleep (inactivity) Settings
#define AUTO_SLEEP_TIMEOUT 5000      // Time in ms after last activity to enter sleep mode

// ----------------------------------------------------
// RUNTIME STATE
// ----------------------------------------------------
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor* stepper = AFMS.getStepper(200, 1); // 200 steps/rev motor on M1+M2

long currentMotorPosition = 0;         // Position tracking (in steps)
unsigned long lastMotionTime = 0;      // Timestamp of last movement
bool motorSleeping = false;            // Motor sleep state flag

// ----------------------------------------------------
// SETUP
// ----------------------------------------------------
void setup() {
  Serial.begin(9600);

  #if defined(ARDUINO_ARCH_AVR)
    while (!Serial);  // Prevent hang on boards that require serial
  #endif

  AFMS.begin();                        // Initialize motor shield
  stepper->setSpeed(MOTOR_MIN_SPEED); // Initial motor speed

  if (ENABLE_DEBUG) {
    Serial.println("Capo Motion System Booted");
    Serial.println("Configuration loaded.");
  }
}

// ----------------------------------------------------
// MAIN LOOP
// ----------------------------------------------------
void loop() {
  // Read joystick X-axis analog value
  int xVal = analogRead(JOYSTICK_X);
  int delta = xVal - JOYSTICK_CENTER;
  unsigned long now = millis();

  // Deadzone check: no motion
  if (abs(delta) < DEADZONE_THRESHOLD) {
    // Check auto-sleep condition
    if (!motorSleeping && (now - lastMotionTime > AUTO_SLEEP_TIMEOUT)) {
      stepper->release();  // Disable motor holding torque
      motorSleeping = true;
      if (ENABLE_DEBUG) {
        Serial.println("Motor has entered sleep mode to reduce heat.");
      }
    }
    return;
  }

  // Wake motor if it's sleeping
  if (motorSleeping) {
    motorSleeping = false;
    stepper->setSpeed(MOTOR_MIN_SPEED);  // Reset speed
    if (ENABLE_DEBUG) {
      Serial.println("Motor waking up from sleep.");
    }
  }

  // Determine direction
  int direction = (delta > 0) ? FORWARD : BACKWARD;

  // Boundary check
  if ((direction == FORWARD && currentMotorPosition >= MOTION_UPPER_BOUND) ||
      (direction == BACKWARD && currentMotorPosition <= MOTION_LOWER_BOUND)) {
    if (ENABLE_DEBUG) {
      Serial.println("Motion limit reached — command ignored.");
    }
    return;
  }

  // Map joystick distance to motor speed
  int speed = map(abs(delta), DEADZONE_THRESHOLD, 512, MOTOR_MIN_SPEED, MOTOR_MAX_SPEED);
  speed = constrain(speed, MOTOR_MIN_SPEED, MOTOR_MAX_SPEED);
  stepper->setSpeed(speed);

  // Step motor one step
  stepper->step(1, direction, STEP_MODE);
  currentMotorPosition += (direction == FORWARD) ? 1 : -1;
  lastMotionTime = now;  // Update last motion time

  // Debug output
  if (ENABLE_DEBUG) {
    Serial.print("Joystick: ");
    Serial.print(xVal);
    Serial.print(" | Direction: ");
    Serial.print((direction == FORWARD) ? "Right" : "Left");
    Serial.print(" | Position: ");
    Serial.print(currentMotorPosition);
    Serial.print(" | Speed: ");
    Serial.println(speed);
  }

  delay(STEP_DELAY_MS);  // Step smoothness
}
