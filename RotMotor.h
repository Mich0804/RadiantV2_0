#pragma once

#include "driver/pcnt.h"
#include "Config.h"

struct RotPidState {
  float kp;
  float ki;
  float kd;
  float ePrev;
  float eInt;
  uint32_t prevUs;
  float iLimit;
  float outLimitPwm;
  float deadbandCounts;
  bool initialized;
};

static const pcnt_unit_t ROT_PCNT_UNIT = PCNT_UNIT_0;
static const uint16_t ROT_PCNT_FILTER = 1000;
static const uint32_t ROT_ACCUM_INTERVAL_MS = 5;
static const int ROT_SIGN = -1;

volatile int32_t rotEncoderCount = 0;

// Final goal and moving reference for the constant-speed trajectory.
float rotTargetCount = 0.0f;
float rotReferenceCount = 0.0f;
float rotTrajectoryStartCount = 0.0f;
uint32_t rotTrajectoryStartUs = 0;
bool rotTrajectoryActive = false;

bool rotPositionMode = false;
int rotLastPwmCommand = 0;
float rotLastPTermPwm = 0.0f;
float rotLastITermPwm = 0.0f;
float rotLastDTermPwm = 0.0f;

RotPidState rotPid = {
  .kp = ROT_PID_KP,
  .ki = ROT_PID_KI,
  .kd = ROT_PID_KD,
  .ePrev = 0.0f,
  .eInt = 0.0f,
  .prevUs = 0,
  .iLimit = ROT_PID_I_LIMIT,
  .outLimitPwm = ROT_PID_OUT_LIMIT_PWM,
  .deadbandCounts = ROT_PID_DEADBAND_COUNTS,
  .initialized = false
};

void rotMotorCoast() {
  ledcWrite(ROT_PWM, 0);
  digitalWrite(ROT_IN1, LOW);
  digitalWrite(ROT_IN2, LOW);
}

void rotMotorBrake() {
  ledcWrite(ROT_PWM, 0);
  digitalWrite(ROT_IN1, HIGH);
  digitalWrite(ROT_IN2, HIGH);
}

void rotMotorDriveSigned(int pwmSigned) {
  if (pwmSigned > PWM_MAX_EFFECTIVE) pwmSigned = PWM_MAX_EFFECTIVE;
  if (pwmSigned < -PWM_MAX_EFFECTIVE) pwmSigned = -PWM_MAX_EFFECTIVE;

  int pwmAbs = abs(pwmSigned);
  if (pwmAbs == 0) {
    rotMotorCoast();
    return;
  }

  if (pwmAbs < PWM_MIN_EFFECTIVE) {
    pwmAbs = PWM_MIN_EFFECTIVE;
  }

  if (pwmSigned > 0) {
    digitalWrite(ROT_IN1, HIGH);
    digitalWrite(ROT_IN2, LOW);
  } else {
    digitalWrite(ROT_IN1, LOW);
    digitalWrite(ROT_IN2, HIGH);
  }

  ledcWrite(ROT_PWM, pwmAbs);
  //ledcWrite(ROT_PWM, 500);
}

void rotPcntSetupQuadrature(pcnt_unit_t unit, int pinA, int pinB) {
  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);

  pcnt_config_t channel0 = {};
  channel0.pulse_gpio_num = pinA;
  channel0.ctrl_gpio_num = pinB;
  channel0.channel = PCNT_CHANNEL_0;
  channel0.unit = unit;
  channel0.pos_mode = PCNT_COUNT_INC;
  channel0.neg_mode = PCNT_COUNT_DEC;
  channel0.lctrl_mode = PCNT_MODE_REVERSE;
  channel0.hctrl_mode = PCNT_MODE_KEEP;
  channel0.counter_h_lim = 32767;
  channel0.counter_l_lim = -32768;
  pcnt_unit_config(&channel0);

  pcnt_config_t channel1 = {};
  channel1.pulse_gpio_num = pinB;
  channel1.ctrl_gpio_num = pinA;
  channel1.channel = PCNT_CHANNEL_1;
  channel1.unit = unit;
  channel1.pos_mode = PCNT_COUNT_INC;
  channel1.neg_mode = PCNT_COUNT_DEC;
  channel1.lctrl_mode = PCNT_MODE_KEEP;
  channel1.hctrl_mode = PCNT_MODE_REVERSE;
  channel1.counter_h_lim = 32767;
  channel1.counter_l_lim = -32768;
  pcnt_unit_config(&channel1);

  pcnt_set_filter_value(unit, ROT_PCNT_FILTER);
  pcnt_filter_enable(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
}

int16_t rotPcntReadAndClear(pcnt_unit_t unit) {
  int16_t value = 0;
  pcnt_get_counter_value(unit, &value);
  pcnt_counter_clear(unit);
  return value;
}

void rotMotorAccumulateEncoder() {
  static uint32_t lastMs = 0;
  uint32_t now = millis();

  if (now - lastMs < ROT_ACCUM_INTERVAL_MS) {
    return;
  }

  lastMs = now;
  int16_t delta = rotPcntReadAndClear(ROT_PCNT_UNIT);
  rotEncoderCount += (int32_t)delta * ROT_SIGN;
}

int32_t rotMotorGetCount() {
  noInterrupts();
  int32_t count = rotEncoderCount;
  interrupts();
  return count;
}

float rotMotorCountsToMotorDeg(float counts) {
  return counts * 360.0f / (float)QUAD_COUNTS_PER_REV;
}

float rotMotorMotorDegToNeedleDeg(float motorDeg) {
  return motorDeg * NEEDLE_DEG_PER_ENCODER_DEG;
}

float rotMotorNeedleDegToMotorDeg(float needleDeg) {
  return needleDeg / NEEDLE_DEG_PER_ENCODER_DEG;
}

float rotMotorCountsToDeg(float counts) {
  return rotMotorMotorDegToNeedleDeg(rotMotorCountsToMotorDeg(counts));
}

float rotMotorDegToCounts(float needleDeg) {
  return rotMotorNeedleDegToMotorDeg(needleDeg) * (float)QUAD_COUNTS_PER_REV / 360.0f;
}

float rotMotorGetDeg() {
  return rotMotorCountsToDeg((float)rotMotorGetCount());
}

void rotMotorResetCount(int32_t value = 0) {
  noInterrupts();
  rotEncoderCount = value;
  interrupts();

  pcnt_counter_pause(ROT_PCNT_UNIT);
  pcnt_counter_clear(ROT_PCNT_UNIT);
  pcnt_counter_resume(ROT_PCNT_UNIT);
}

float rotMotorPidPosition(float position, float target, RotPidState &state) {
  uint32_t nowUs = micros();

  if (!state.initialized) {
    state.prevUs = nowUs;
    state.ePrev = 0.0f;
    state.eInt = 0.0f;
    state.initialized = true;
    rotLastPTermPwm = 0.0f;
    rotLastITermPwm = 0.0f;
    rotLastDTermPwm = 0.0f;
    return 0.0f;
  }

  float dt = (nowUs - state.prevUs) * 1e-6f;
  state.prevUs = nowUs;

  if (dt <= 0.0f || dt > 0.2f) {
    dt = 0.01f;
  }

  float error = target - position;
  if (fabsf(error) <= state.deadbandCounts) {
    state.ePrev = error;
    rotLastPTermPwm = 0.0f;
    rotLastITermPwm = 0.0f;
    rotLastDTermPwm = 0.0f;
    return 0.0f;
  }

  state.eInt += error * dt;
  if (state.eInt > state.iLimit) state.eInt = state.iLimit;
  if (state.eInt < -state.iLimit) state.eInt = -state.iLimit;

  float derivative = (error - state.ePrev) / dt;
  state.ePrev = error;

  rotLastPTermPwm = state.kp * error;
  rotLastITermPwm = state.ki * state.eInt;
  rotLastDTermPwm = state.kd * derivative;

  float output =
      rotLastPTermPwm +
      rotLastITermPwm +
      rotLastDTermPwm;

  if (output > state.outLimitPwm) output = state.outLimitPwm;
  if (output < -state.outLimitPwm) output = -state.outLimitPwm;
  return output;
}

bool rotMotorBegin() {
  pinMode(ENC_ROT_A, INPUT);
  pinMode(ENC_ROT_B, INPUT);
  rotPcntSetupQuadrature(ROT_PCNT_UNIT, ENC_ROT_A, ENC_ROT_B);
  rotMotorResetCount(0);

  pinMode(ROT_IN1, OUTPUT);
  pinMode(ROT_IN2, OUTPUT);
  pinMode(ROT_STBY, OUTPUT);
  digitalWrite(ROT_STBY, HIGH);

  ledcAttach(ROT_PWM, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcWrite(ROT_PWM, 0);
  rotMotorCoast();
  return true;
}

void rotMotorSetTargetCounts(float targetCounts) {
  const float currentCount =
      (float)rotMotorGetCount();

  rotTrajectoryStartCount = currentCount;
  rotReferenceCount = currentCount;
  rotTargetCount = targetCounts;
  rotTrajectoryStartUs = micros();

  rotTrajectoryActive =
      fabsf(rotTargetCount - rotTrajectoryStartCount) >
      ROT_PID_DEADBAND_COUNTS;

  rotPositionMode = true;
  rotPid.initialized = false;
  rotLastPTermPwm = 0.0f;
  rotLastITermPwm = 0.0f;
  rotLastDTermPwm = 0.0f;
}

void rotMotorSetTargetDeg(float targetDeg) {
  rotMotorSetTargetCounts(rotMotorDegToCounts(targetDeg));
}

void rotMotorUpdateTrajectory() {
  if (!rotTrajectoryActive) {
    rotReferenceCount = rotTargetCount;
    return;
  }

  const float speedCountsS =
      fabsf(rotMotorDegToCounts(
          ROT_TRAJECTORY_SPEED_DEG_S));

  if (speedCountsS <= 0.0f) {
    rotReferenceCount = rotTargetCount;
    rotTrajectoryActive = false;
    return;
  }

  const uint32_t elapsedUs =
      micros() - rotTrajectoryStartUs;

  const float maxTravelCounts =
      speedCountsS *
      (float)elapsedUs *
      1e-6f;

  const float trajectoryDelta =
      rotTargetCount -
      rotTrajectoryStartCount;

  if (maxTravelCounts >= fabsf(trajectoryDelta)) {
    rotReferenceCount = rotTargetCount;
    rotTrajectoryActive = false;
    return;
  }

  rotReferenceCount =
      rotTrajectoryStartCount +
      ((trajectoryDelta >= 0.0f)
          ? maxTravelCounts
          : -maxTravelCounts);
}

void rotMotorZero() {
  rotMotorResetCount(0);
  rotTargetCount = 0.0f;
  rotReferenceCount = 0.0f;
  rotTrajectoryStartCount = 0.0f;
  rotTrajectoryStartUs = micros();
  rotTrajectoryActive = false;
  rotPid.initialized = false;
  rotLastPTermPwm = 0.0f;
  rotLastITermPwm = 0.0f;
  rotLastDTermPwm = 0.0f;
}

void rotMotorStop() {
  const float currentCount =
      (float)rotMotorGetCount();

  rotTargetCount = currentCount;
  rotReferenceCount = currentCount;
  rotTrajectoryStartCount = currentCount;
  rotTrajectoryStartUs = micros();
  rotTrajectoryActive = false;
  rotPositionMode = false;
  rotPid.initialized = false;
  rotLastPwmCommand = 0;
  rotLastPTermPwm = 0.0f;
  rotLastITermPwm = 0.0f;
  rotLastDTermPwm = 0.0f;
  rotMotorCoast();
}

bool rotMotorAtTarget() {
  if (rotTrajectoryActive) {
    return false;
  }

  return fabsf(
      rotTargetCount -
      (float)rotMotorGetCount())
      <= ROT_PID_DEADBAND_COUNTS;
}

void rotMotorUpdate() {
  rotMotorAccumulateEncoder();

  if (!rotPositionMode) {
    return;
  }

  rotMotorUpdateTrajectory();

  float output = rotMotorPidPosition(
      (float)rotMotorGetCount(),
      rotReferenceCount,
      rotPid);
  rotLastPwmCommand = (int)lroundf(output);

  if (rotLastPwmCommand == 0) {
    rotMotorBrake();
  } else {
    rotMotorDriveSigned(rotLastPwmCommand);
  }
}
