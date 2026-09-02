#pragma once

#include "BluetoothSerial.h"
#include "Config.h"
#include "AS5600Encoder.h"
#include "HomingController.h"
#include "RotMotor.h"

extern BluetoothSerial SerialBT;

enum SystemState {
  STARTUP_HOMING,
  NORMAL_RUN,
  RETURN_TO_ZERO_ON_BT_LOSS,
  STOPPED,
  TRANSLATION_FAULT,
  ERROR_STATE
};

SystemState systemState = STARTUP_HOMING;
bool stageBtWasConnected = false;

const char* stageStateName(SystemState state) {
  switch (state) {
    case STARTUP_HOMING: return "STARTUP_HOMING";
    case NORMAL_RUN: return "NORMAL_RUN";
    case RETURN_TO_ZERO_ON_BT_LOSS: return "RETURN_TO_ZERO_ON_BT_LOSS";
    case STOPPED: return "STOPPED";
    case TRANSLATION_FAULT: return "TRANSLATION_FAULT";
    case ERROR_STATE: return "ERROR_STATE";
    default: return "UNKNOWN_STATE";
  }
}

void stageChangeState(int newState) {
  SystemState nextState = (SystemState)newState;
  if (systemState != nextState) {
    Serial.print("STATE: ");
    Serial.print(stageStateName(systemState));
    Serial.print(" -> ");
    Serial.println(stageStateName(nextState));
    systemState = nextState;
  }
}

bool stageCanAcceptMotion() {
  return systemState == NORMAL_RUN ||
         systemState == RETURN_TO_ZERO_ON_BT_LOSS ||
         (systemState == STOPPED && homingIsDone());
}

void stageResumeForMotion() {
  if (systemState == STOPPED) {
    stageChangeState(NORMAL_RUN);
  }
}

float stageClampInnerMm(float innerMm, float outerMm) {
  if (innerMm < 0.0f) {
    innerMm = 0.0f;
  }

  if (innerMm < outerMm) {
    return outerMm;
  }

  float maxInnerMm = outerMm + INNER_MAX_RETRACTION_MM;
  if (innerMm > maxInnerMm) {
    return maxInnerMm;
  }

  return innerMm;
}

void stageApplyInnerSafety() {
  float outerTargetMm = outerAxis.targetMm();
  float innerTargetMm = innerAxis.targetMm();
  float clampedInnerMm = stageClampInnerMm(innerTargetMm, outerTargetMm);

  if (fabsf(clampedInnerMm - innerTargetMm) > 0.0001f) {
    innerAxis.setTargetMm(clampedInnerMm);
  }
}

void stageHome();

void stageStopTranslation() {
  outerAxis.stop();
  innerAxis.stop();
}

void stageEnterTranslationFault(const char *message) {
  if (systemState != TRANSLATION_FAULT) {
    Serial.println(message);
  }

  stageStopTranslation();
  if (homingIsRunning()) {
    homingAbortToError("ERROR: Homing aborted by translation fault.");
  }
  stageChangeState(TRANSLATION_FAULT);
}

bool stageCheckPlatformCollision() {
  if (!platformCollisionActive()) {
    return false;
  }

  stageEnterTranslationFault("SAFETY: PLATFORM collision switch active. Translation fault.");
  return true;
}

bool stageTargetWouldViolateOuterLimit(float targetMm) {
  const long targetSteps = outerAxis.mmToSteps(targetMm);

  if (targetSteps < outerAxis.currentSteps - TRANS_POSITION_TOLERANCE_STEPS && outerHomeActive()) {
    Serial.println("SAFETY: OUTER home switch active; rejecting further outer retraction.");
    return true;
  }

  if (targetSteps > outerAxis.currentSteps + TRANS_POSITION_TOLERANCE_STEPS && outerInsertLimitActive()) {
    Serial.println("SAFETY: OUTER insert limit active; rejecting further outer insertion.");
    return true;
  }

  return false;
}

bool stageTargetWouldViolateInnerLimit(float targetMm) {
  const long targetSteps = innerAxis.mmToSteps(targetMm);

  if (targetSteps < innerAxis.currentSteps - TRANS_POSITION_TOLERANCE_STEPS && innerHomeActive()) {
    Serial.println("SAFETY: INNER home switch active; rejecting further inner retraction.");
    return true;
  }

  if (targetSteps > innerAxis.currentSteps + TRANS_POSITION_TOLERANCE_STEPS && innerInsertLimitActive()) {
    Serial.println("SAFETY: INNER insert limit active; rejecting further inner insertion.");
    return true;
  }

  return false;
}

bool stageApplyDirectionalLimitStops() {
  bool stopped = false;

  if (outerHomeActive() && outerAxis.motionTowardRetraction()) {
    Serial.println("SAFETY: OUTER home switch active; stopping outer retraction.");
    outerAxis.stop();
    stopped = true;
  }

  if (outerInsertLimitActive() && outerAxis.motionTowardInsertion()) {
    Serial.println("SAFETY: OUTER insert limit active; stopping outer insertion.");
    outerAxis.stop();
    stopped = true;
  }

  if (innerHomeActive() && innerAxis.motionTowardRetraction()) {
    Serial.println("SAFETY: INNER home switch active; stopping inner retraction.");
    innerAxis.stop();
    stopped = true;
  }

  if (innerInsertLimitActive() && innerAxis.motionTowardInsertion()) {
    Serial.println("SAFETY: INNER insert limit active; stopping inner insertion.");
    innerAxis.stop();
    stopped = true;
  }

  return stopped;
}

bool stageSetOuterMm(float outerMm) {
  if (!stageCanAcceptMotion()) {
    return false;
  }

  if (stageCheckPlatformCollision()) {
    return false;
  }

  stageResumeForMotion();

  if (outerMm < 0.0f) {
    outerMm = 0.0f;
  }

  // An OUTER command moves both translation axes to the same target.
  // Check both limits before changing either target. 
  if (stageTargetWouldViolateOuterLimit(outerMm) ||
      stageTargetWouldViolateInnerLimit(outerMm)) {
    return false;
  }

  outerAxis.setTargetMm(outerMm);
  innerAxis.setTargetMm(outerMm);
  return true;
  }

bool stageSetInnerMm(float innerMm) {
  if (!stageCanAcceptMotion()) {
    return false;
  }

  if (stageCheckPlatformCollision()) {
    return false;
  }

  stageResumeForMotion();

  if (innerMm < 0.0f) {
    innerMm = 0.0f;
  }

  const float clampedInnerMm = stageClampInnerMm(innerMm, outerAxis.targetMm());
  if (stageTargetWouldViolateInnerLimit(clampedInnerMm)) {
    return false;
  }

  innerAxis.setTargetMm(clampedInnerMm);
  return true;
}

float stageWrapAngle360(float deg) {
  while (deg < 0.0f) deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;
  return deg;
}

float stageWrapAngle180(float deg) {
  while (deg <= -180.0f) deg += 360.0f;
  while (deg > 180.0f) deg -= 360.0f;
  return deg;
}

bool stageSetRotDeg(float targetAbsDeg) {
  if (!stageCanAcceptMotion()) {
    return false;
  }

  stageResumeForMotion();

  float currentDegContinuous = rotMotorGetDeg();
  float currentDegWrapped = stageWrapAngle360(currentDegContinuous);
  float deltaDeg = stageWrapAngle180(stageWrapAngle360(targetAbsDeg) - currentDegWrapped);
  float newTargetDegContinuous = currentDegContinuous + deltaDeg;
  rotMotorSetTargetDeg(newTargetDegContinuous);
  return true;
}

void stageStopAll() {
  stageStopTranslation();
  rotMotorStop();
}

void stageReturnToZero() {
  outerAxis.setTarget(0);
  innerAxis.setTarget(0);
  rotMotorSetTargetCounts(0.0f);
}

bool stageAllAtTarget() {
  return outerAxis.atTarget() && innerAxis.atTarget() && rotMotorAtTarget();
}

void stageHome() {
  stageStopAll();
  rotMotorStop();
  homingStart();
  stageChangeState(STARTUP_HOMING);
}

void stageZeroRot() {
  rotMotorZero();
}

void stageBegin() {
  pinMode(STEPPER_EN, OUTPUT);
  digitalWrite(STEPPER_EN, DRV_ENABLE_INACTIVE);

  bool okOuter = outerAxis.begin();
  bool okInner = innerAxis.begin();
  homingBegin();
  as5600Begin();
  bool okRot = rotMotorBegin();

  if (!okOuter || !okInner || !okRot) {
    stageChangeState(ERROR_STATE);
    return;
  }

  digitalWrite(STEPPER_EN, DRV_ENABLE_ACTIVE);
  stageHome();
}

void stageUpdate() {
  static unsigned long lastAS5600UpdateMs = 0;

  rotMotorUpdate();

  const unsigned long nowMs = millis();
  if (nowMs - lastAS5600UpdateMs >= AS5600_UPDATE_INTERVAL_MS) {
    lastAS5600UpdateMs = nowMs;
    updateAS5600(outerSpindleEncoder);
    updateAS5600(innerSpindleEncoder);
  }

  const float outerMeasuredMm = as5600PositionMm(outerSpindleEncoder, OUTER_AS5600_POSITION_SIGN);

  const float innerMeasuredMm = as5600PositionMm(innerSpindleEncoder, INNER_AS5600_POSITION_SIGN);

  if (stageCheckPlatformCollision()) {
    return;
  }

  if (homingIsRunning()) {
    homingUpdate();
  }

  if (homingHasError()) {
    stageStopAll();
    stageChangeState(ERROR_STATE);
    return;
  }

  switch (systemState) {
    case STARTUP_HOMING:
      if (homingIsDone()) {
        outerAxis.setZero();
        innerAxis.setZero();
        as5600SetZero(outerSpindleEncoder);
        as5600SetZero(innerSpindleEncoder);
        rotTargetCount = (float)rotMotorGetCount();
        rotPositionMode = false;
        stageBtWasConnected = false;
        stageChangeState(NORMAL_RUN);
      }
      break;

    case NORMAL_RUN:
      stageApplyInnerSafety();
      stageApplyDirectionalLimitStops();

      if (!as5600ValidForControl(outerSpindleEncoder) ||
          !as5600ValidForControl(innerSpindleEncoder))
      {
        stageEnterTranslationFault(
            "SAFETY: Translation encoder invalid.");
        return;
      }

      outerAxis.setMeasuredPositionMm(outerMeasuredMm);
      innerAxis.setMeasuredPositionMm(innerMeasuredMm);

      outerAxis.update();
      innerAxis.update();

      stageApplyDirectionalLimitStops();

      if (SerialBT.hasClient()) {
        stageBtWasConnected = true;
      } else if (stageBtWasConnected) {
        stageReturnToZero();
        stageBtWasConnected = false;
        stageChangeState(RETURN_TO_ZERO_ON_BT_LOSS);
      }
      break;

    case RETURN_TO_ZERO_ON_BT_LOSS:
      stageApplyInnerSafety();
      stageApplyDirectionalLimitStops();

      if (!as5600ValidForControl(outerSpindleEncoder) ||
          !as5600ValidForControl(innerSpindleEncoder))
      {
        stageEnterTranslationFault(
            "SAFETY: Translation encoder invalid.");
        return;
      }

      outerAxis.setMeasuredPositionMm(outerMeasuredMm);
      innerAxis.setMeasuredPositionMm(innerMeasuredMm);

      outerAxis.update();
      innerAxis.update();

      stageApplyDirectionalLimitStops();

      if (stageAllAtTarget()) {
        stageChangeState(NORMAL_RUN);
      }
      break;

    case STOPPED:
      break;

    case TRANSLATION_FAULT:
      stageStopTranslation();
      break;

    case ERROR_STATE:
      stageStopAll();
      break;
  }
}

void stagePrintAS5600Status(Print &out, const AS5600State &encoder) {
  out.print(" AS5600 connected=");
  out.print(encoder.connected ? "YES" : "NO");
  out.print(" raw=");
  out.print(encoder.rawAngle);
  out.print(" angle=");
  out.print(as5600RawToDegrees(encoder.rawAngle), 2);
  out.print(" deg counts=");
  out.print(encoder.accumulatedCounts);
  out.print(" revolutions=");
  out.print(as5600CountsToRevolutions(encoder.accumulatedCounts), 4);
  out.print(" position=");
  out.print(as5600CountsToMm(encoder.accumulatedCounts), 3);
  out.print(" mm magnet=");
  out.print(as5600MagnetStatusName(encoder));
}

void stagePrintStatus(Print &out) {
  out.print("state=");
  out.print(stageStateName(systemState));
  out.print(" homing=");
  out.print(homingStateName(homingState));
  out.print(" outer_mm=");
  out.print(outerAxis.currentMm(), 3);
  out.print(" outer_target_mm=");
  out.print(outerAxis.targetMm(), 3);
  out.print(" inner_mm=");
  out.print(innerAxis.currentMm(), 3);
  out.print(" inner_target_mm=");
  out.print(innerAxis.targetMm(), 3);
  out.print(" rot_deg=");
  out.print(rotMotorGetDeg(), 2);
  out.print(" rot_target_deg=");
  out.print(rotMotorCountsToDeg(rotTargetCount), 2);
  out.print(" bt=");
  out.println(SerialBT.hasClient() ? "connected" : "disconnected");

  out.print("OUTER: HOME=");
  out.print(safetySwitchStatusName(outerHomeActive()));
  out.print(" INSERT=");
  out.print(safetySwitchStatusName(outerInsertLimitActive()));
  stagePrintAS5600Status(out, outerSpindleEncoder);
  out.println();

  out.print("INNER: HOME=");
  out.print(safetySwitchStatusName(innerHomeActive()));
  out.print(" INSERT=");
  out.print(safetySwitchStatusName(innerInsertLimitActive()));
  stagePrintAS5600Status(out, innerSpindleEncoder);
  out.println();

  out.print("PLATFORM COLLISION=");
  out.println(safetySwitchStatusName(platformCollisionActive()));
}
