#pragma once

#include "esp32-hal-rmt.h"
#include "Config.h"

class RmtStepperAxis {
public:
  RmtStepperAxis(int step, int dir, float stepsPerMm, bool insertDirPositive, float maxSpeedStepsS, float accelStepsS2)
      : stepPin(step),
        dirPin(dir),
        stepsPerMillimeter(stepsPerMm),
        insertionDirPositive(insertDirPositive),
        maxSpeed(maxSpeedStepsS),
        accel(accelStepsS2) {}

  bool begin() {
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    digitalWrite(stepPin, LOW);
    digitalWrite(dirPin, HIGH);

    if (!rmtInit(stepPin, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, RMT_TICK_HZ)) {
      Serial.print("RMT init failed on pin ");
      Serial.println(stepPin);
      return false;
    }

    rmtSetEOT(stepPin, 0);
    stop();
    return true;
  }

  void setCurrentPosition(long pos) {
    currentSteps = pos;
    posFloat = (float)pos;
    targetSteps = pos;
    currentSpeed = 0.0f;
    commandedSpeed = 0.0f;
  }

  void setMeasuredPositionMm(float measuredMm)
  {
    currentSteps = mmToSteps(measuredMm);
    posFloat = (float)currentSteps;
  }

  void setZero() {
    setCurrentPosition(0);
  }

  void setTarget(long target) {
    targetSteps = target;
  }

  void setTargetMm(float mm) {
    setTarget(mmToSteps(mm));
  }

  long mmToSteps(float mm) const {
    return lroundf(mm * stepsPerMillimeter);
  }

  float stepsToMm(long steps) const {
    return (float)steps / stepsPerMillimeter;
  }

  float currentMm() const {
    return stepsToMm(currentSteps);
  }

  float targetMm() const {
    return stepsToMm(targetSteps);
  }

  long distanceToGo() const {
    return targetSteps - currentSteps;
  }

  bool atTarget() const {
    return labs(distanceToGo()) <= TRANS_POSITION_TOLERANCE_STEPS && fabsf(currentSpeed) < 1.0f;
  }

  bool motionTowardInsertion() const {
    if (rmtRunning) {
      return dirPositive == insertionDirPositive;
    }

    return distanceToGo() > TRANS_POSITION_TOLERANCE_STEPS;
  }

  bool motionTowardRetraction() const {
    if (rmtRunning) {
      return dirPositive != insertionDirPositive;
    }

    return distanceToGo() < -TRANS_POSITION_TOLERANCE_STEPS;
  }

  void setDirection(bool positive) {
    dirPositive = positive;
    digitalWrite(dirPin, dirPositive ? HIGH : LOW);
    delayMicroseconds(5);
  }

  bool startConstantSpeed(float stepsPerSec, bool positive) {
    if (stepsPerSec < 1.0f) {
      stopRMT();
      return false;
    }

    setDirection(positive);
    commandedSpeed = (dirPositive == insertionDirPositive) ? stepsPerSec : -stepsPerSec;
    currentSpeed = commandedSpeed;
    writeRmtSpeed(stepsPerSec);
    lastAppliedAbsSpeed = stepsPerSec;
    lastSpeedRefreshMs = millis();
    return rmtRunning;
  }

  void stopRMT() {
    rmtWriteLooping(stepPin, nullptr, 0);
    rmtRunning = false;
    digitalWrite(stepPin, LOW);
  }

  void stop() {
    stopRMT();
    currentSpeed = 0.0f;
    commandedSpeed = 0.0f;
    targetSteps = currentSteps;
  }

  void updatePositionEstimate(float speedStepsPerSec) {
    uint32_t nowUs = micros();
    if (lastUpdateUs == 0) {
      lastUpdateUs = nowUs;
      return;
    }

    float dt = (nowUs - lastUpdateUs) * 1e-6f;
    lastUpdateUs = nowUs;

    if (dt <= 0.0f || dt > 0.1f) {
      dt = 0.001f;
    }

    if (rmtRunning) {
      float signedSpeed = (dirPositive == insertionDirPositive) ? speedStepsPerSec : -speedStepsPerSec;
      posFloat += signedSpeed * dt;
      currentSteps = lroundf(posFloat);
    }
  }

  void update() {
    uint32_t nowUs = micros();
    if (lastUpdateUs == 0) {
      lastUpdateUs = nowUs;
      return;
    }

    float dt = (nowUs - lastUpdateUs) * 1e-6f;
    lastUpdateUs = nowUs;

    if (dt <= 0.0f || dt > 0.05f) {
      dt = 0.001f;
    }

    long dist = targetSteps - currentSteps;
    if (labs(dist) <= TRANS_POSITION_TOLERANCE_STEPS) {
      currentSpeed = 0.0f;
      commandedSpeed = 0.0f;
      stopRMT();
      return;
    }

    int direction = (dist > 0) ? 1 : -1;
    float absSpeed = fabsf(currentSpeed);
    float decelDist = (absSpeed * absSpeed) / (2.0f * accel);
    float targetAbsSpeed = maxSpeed;

    if ((float)labs(dist) <= decelDist) {
      targetAbsSpeed = 0.0f;
    }

    if (absSpeed < targetAbsSpeed) {
      absSpeed += accel * dt;
      if (absSpeed > targetAbsSpeed) absSpeed = targetAbsSpeed;
    } else if (absSpeed > targetAbsSpeed) {
      absSpeed -= accel * dt;
      if (absSpeed < targetAbsSpeed) absSpeed = targetAbsSpeed;
    }

    commandedSpeed = direction * absSpeed;
    currentSpeed = commandedSpeed;

    bool wantInsertion = commandedSpeed >= 0.0f;
    bool wantDirPositive = wantInsertion ? insertionDirPositive : !insertionDirPositive;
    if (rmtRunning && wantDirPositive != dirPositive) {
      stopRMT();
      currentSpeed = 0.0f;
      commandedSpeed = 0.0f;
      return;
    }

    setDirection(wantDirPositive);

    float appliedAbsSpeed = fabsf(commandedSpeed);
    uint32_t nowMs = millis();
    bool refreshRmt =
        (!rmtRunning && appliedAbsSpeed >= 1.0f) ||
        (fabsf(appliedAbsSpeed - lastAppliedAbsSpeed) > RMT_SPEED_REFRESH_DELTA) ||
        (nowMs - lastSpeedRefreshMs >= RMT_REFRESH_INTERVAL_MS);

    if (appliedAbsSpeed < 1.0f) {
      stopRMT();
    } else if (refreshRmt) {
      writeRmtSpeed(appliedAbsSpeed);
      lastAppliedAbsSpeed = appliedAbsSpeed;
      lastSpeedRefreshMs = nowMs;
    }
  }

  int stepPin;
  int dirPin;
  float stepsPerMillimeter;
  bool insertionDirPositive;
  float maxSpeed;
  float accel;
  long currentSteps = 0;
  long targetSteps = 0;
  float posFloat = 0.0f;
  float currentSpeed = 0.0f;
  float commandedSpeed = 0.0f;
  bool rmtRunning = false;
  bool dirPositive = true;

private:
  void writeRmtSpeed(float stepsPerSec) {
    float periodUs = 1000000.0f / stepsPerSec;
    if (periodUs <= (float)STEP_PULSE_US + 1.0f) {
      periodUs = (float)STEP_PULSE_US + 1.0f;
    }

    const float ticksPerUs = (float)RMT_TICK_HZ / 1000000.0f;
    uint32_t highTicks = (uint32_t)(STEP_PULSE_US * ticksPerUs);
    uint32_t lowTicks = (uint32_t)((periodUs - STEP_PULSE_US) * ticksPerUs);

    if (highTicks < 1) highTicks = 1;
    if (lowTicks < 1) lowTicks = 1;
    if (highTicks > 32767) highTicks = 32767;
    if (lowTicks > 32767) lowTicks = 32767;

    stepSymbol[0].duration0 = highTicks;
    stepSymbol[0].level0 = 1;
    stepSymbol[0].duration1 = lowTicks;
    stepSymbol[0].level1 = 0;

    stopRMT();
    rmtRunning = rmtWriteLooping(stepPin, stepSymbol, 1);
  }

  rmt_data_t stepSymbol[1];
  uint32_t lastUpdateUs = 0;
  uint32_t lastSpeedRefreshMs = 0;
  float lastAppliedAbsSpeed = 0.0f;
};

RmtStepperAxis outerAxis(OUTER_STEP, OUTER_DIR, OUTER_STEPS_PER_MM, OUTER_INSERT_DIR_POSITIVE, OUTER_MAX_SPEED_STEPS_S, OUTER_ACCEL_STEPS_S2);
RmtStepperAxis innerAxis(INNER_STEP, INNER_DIR, INNER_STEPS_PER_MM, INNER_INSERT_DIR_POSITIVE, INNER_MAX_SPEED_STEPS_S, INNER_ACCEL_STEPS_S2);
