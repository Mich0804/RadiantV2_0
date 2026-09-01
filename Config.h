#pragma once

#include <Arduino.h>

// Translation steppers, DRV8825 drivers.
#define OUTER_STEP 16
#define OUTER_DIR  17
#define INNER_STEP 18
#define INNER_DIR  19
#define STEPPER_EN 23

// NC switches. HIGH means switch active or wiring fault.
// GPIO32, GPIO33 and GPIO5 use INPUT_PULLUP.
// GPIO36 and GPIO39 have no internal pull-ups and require external ~10 kOhm pull-ups to 3.3 V.
#define OUTER_HOME_SW 32
#define INNER_HOME_SW 33
#define OUTER_INSERT_LIMIT_SW 36
#define INNER_INSERT_LIMIT_SW 39
#define PLATFORM_COLLISION_SW 5

// Spindle AS5600 magnetic encoders. Both sensors use fixed address 0x36,
// so they are connected to separate ESP32 I2C controllers.
#define OUTER_AS5600_SDA 21
#define OUTER_AS5600_SCL 22
#define INNER_AS5600_SDA 4
#define INNER_AS5600_SCL 13

// TB6612FNG channel for Maxon rotation motor.
#define ROT_PWM   25
#define ROT_IN1   26
#define ROT_IN2   27
#define ROT_STBY  14

// Maxon quadrature encoder.
#define ENC_ROT_A 34
#define ENC_ROT_B 35

#define DRV_ENABLE_ACTIVE   LOW
#define DRV_ENABLE_INACTIVE HIGH

const char BT_DEVICE_NAME[] = "Radiant1.1";

// Calibration: new_steps_per_mm = old_steps_per_mm * commanded_mm / measured_mm.
const float OUTER_STEPS_PER_MM = 400.0f;
const float INNER_STEPS_PER_MM = 800.0f;
const float INNER_MAX_RETRACTION_MM = 30.0f;
const float SPINDLE_LEAD_MM_PER_REV = 2.0f;

const float OUTER_MAX_SPEED_STEPS_S = 1200.0f;
const float INNER_MAX_SPEED_STEPS_S = 2400.0f;
const float OUTER_ACCEL_STEPS_S2 = 400.0f;
const float INNER_ACCEL_STEPS_S2 = 800.0f;
const long TRANS_POSITION_TOLERANCE_STEPS = 3;

const float OUTER_HOMING_SPEED_MM_S = 0.8f;
const float INNER_HOMING_SPEED_MM_S = 0.4f;
const float OUTER_HOMING_SPEED_STEPS_S = OUTER_HOMING_SPEED_MM_S * OUTER_STEPS_PER_MM;
const float INNER_HOMING_SPEED_STEPS_S = INNER_HOMING_SPEED_MM_S * INNER_STEPS_PER_MM;

const float HOMING_BACKOFF_MM = 2.0f;
const float SWITCH_CLEAR_MM = 3.0f;
const float MAX_HOMING_TRAVEL_MM = 50.0f;

const long OUTER_BACKOFF_STEPS = (long)(HOMING_BACKOFF_MM * OUTER_STEPS_PER_MM + 0.5f);
const long INNER_BACKOFF_STEPS = (long)(HOMING_BACKOFF_MM * INNER_STEPS_PER_MM + 0.5f);
const long OUTER_SWITCH_CLEAR_STEPS = (long)(SWITCH_CLEAR_MM * OUTER_STEPS_PER_MM + 0.5f);
const long INNER_SWITCH_CLEAR_STEPS = (long)(SWITCH_CLEAR_MM * INNER_STEPS_PER_MM + 0.5f);
const long OUTER_MAX_HOMING_STEPS = (long)(MAX_HOMING_TRAVEL_MM * OUTER_STEPS_PER_MM + 0.5f);
const long INNER_MAX_HOMING_STEPS = (long)(MAX_HOMING_TRAVEL_MM * INNER_STEPS_PER_MM + 0.5f);

const bool OUTER_HOME_DIR_POSITIVE = true;
const bool INNER_HOME_DIR_POSITIVE = true;
const bool OUTER_INSERT_DIR_POSITIVE = !OUTER_HOME_DIR_POSITIVE;
const bool INNER_INSERT_DIR_POSITIVE = !INNER_HOME_DIR_POSITIVE;

const uint32_t RMT_TICK_HZ = 10000000;
const uint16_t STEP_PULSE_US = 4;
const uint32_t RMT_REFRESH_INTERVAL_MS = 20;
const float RMT_SPEED_REFRESH_DELTA = 50.0f;

const int PWM_FREQ_HZ = 20000;
const int PWM_RES_BITS = 10;
const int PWM_MIN_EFFECTIVE = 120;
const int PWM_MAX_EFFECTIVE = 300;

const int ENCODER_CPT = 512;
const int QUAD_COUNTS_PER_REV = ENCODER_CPT * 4;

const float MOTOR_GEAR_TEETH = 24.0f;
const float NEEDLE_GEAR_TEETH = 30.0f;
const float GEAR_DIRECTION_SIGN = -1.0f;
const float NEEDLE_DEG_PER_ENCODER_DEG =
    GEAR_DIRECTION_SIGN * MOTOR_GEAR_TEETH / NEEDLE_GEAR_TEETH;

const float ROT_PID_KP = 0.1f;
const float ROT_PID_KI = 0.01f;
const float ROT_PID_KD = 0.0015f;
const float ROT_PID_I_LIMIT = 100.0f;
const float ROT_PID_OUT_LIMIT_PWM = 300.0f;
const float ROT_PID_DEADBAND_COUNTS = 3.0f;

const uint32_t STATUS_INTERVAL_MS = 250;
const uint32_t AS5600_UPDATE_INTERVAL_MS = 10;
const uint32_t AS5600_I2C_FREQUENCY_HZ = 400000;
