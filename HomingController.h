#pragma once

#include "Config.h"
#include "RmtStepperAxis.h"
#include "SafetySwitches.h"

enum HomingState {
  HOMING_IDLE,
  HOME_OUTER_PRECHECK,
  HOME_OUTER_CLEAR,
  HOME_OUTER_START,
  HOME_OUTER_SEARCH,
  HOME_OUTER_BACKOFF,
  HOME_INNER_PRECHECK,
  HOME_INNER_CLEAR,
  HOME_INNER_START,
  HOME_INNER_SEARCH,
  HOME_INNER_BACKOFF,
  HOMING_DONE,
  HOMING_ERROR
};

HomingState homingState = HOMING_IDLE;
unsigned long homingStateStartMs = 0;
long homingStateStartSteps = 0;

const char* homingStateName(HomingState state) {
  switch (state) {
    case HOMING_IDLE: return "HOMING_IDLE";
    case HOME_OUTER_PRECHECK: return "HOME_OUTER_PRECHECK";
    case HOME_OUTER_CLEAR: return "HOME_OUTER_CLEAR";
    case HOME_OUTER_START: return "HOME_OUTER_START";
    case HOME_OUTER_SEARCH: return "HOME_OUTER_SEARCH";
    case HOME_OUTER_BACKOFF: return "HOME_OUTER_BACKOFF";
    case HOME_INNER_PRECHECK: return "HOME_INNER_PRECHECK";
    case HOME_INNER_CLEAR: return "HOME_INNER_CLEAR";
    case HOME_INNER_START: return "HOME_INNER_START";
    case HOME_INNER_SEARCH: return "HOME_INNER_SEARCH";
    case HOME_INNER_BACKOFF: return "HOME_INNER_BACKOFF";
    case HOMING_DONE: return "HOMING_DONE";
    case HOMING_ERROR: return "HOMING_ERROR";
    default: return "UNKNOWN_HOMING";
  }
}

void homingChangeState(int newState) {
  homingState = (HomingState)newState;
  homingStateStartMs = millis();
  Serial.print("HOMING -> ");
  Serial.println(homingStateName(homingState));
}

void homingBegin() {
  safetySwitchesBegin();
  homingChangeState(HOMING_IDLE);
}

void homingStart() {
  outerAxis.stop();
  innerAxis.stop();
  outerAxis.setZero();
  innerAxis.setZero();
  homingChangeState(HOME_INNER_PRECHECK);
}

bool homingIsDone() {
  return homingState == HOMING_DONE;
}

bool homingHasError() {
  return homingState == HOMING_ERROR;
}

bool homingIsRunning() {
  return homingState != HOMING_IDLE && homingState != HOMING_DONE && homingState != HOMING_ERROR;
}

void homingAbortToError(const char *message) {
  outerAxis.stopRMT();
  innerAxis.stopRMT();
  Serial.println(message);
  homingChangeState(HOMING_ERROR);
}

void homingUpdate() {
  outerAxis.updatePositionEstimate(OUTER_HOMING_SPEED_STEPS_S);
  innerAxis.updatePositionEstimate(INNER_HOMING_SPEED_STEPS_S);

  switch (homingState) {
    case HOMING_IDLE:
      break;

    case HOME_OUTER_PRECHECK:
      Serial.println("Checking OUTER switch...");
      if (outerHomeActive()) {
        Serial.println("OUTER switch active. Clearing by moving away...");
        homingStateStartSteps = outerAxis.currentSteps;
        outerAxis.startConstantSpeed(OUTER_HOMING_SPEED_STEPS_S, !OUTER_HOME_DIR_POSITIVE);
        homingChangeState(HOME_OUTER_CLEAR);
      } else {
        homingChangeState(HOME_OUTER_START);
      }
      break;

    case HOME_OUTER_CLEAR:
      if (outerInsertLimitActive()) {
        homingAbortToError("ERROR: OUTER insert limit active while clearing home switch.");
        break;
      }

      if (!outerHomeActive()) {
        outerAxis.stopRMT();
        Serial.println("OUTER switch cleared.");
        delay(100);
        homingChangeState(HOME_OUTER_START);
      }

      if (labs(outerAxis.currentSteps - homingStateStartSteps) > OUTER_SWITCH_CLEAR_STEPS) {
        outerAxis.stopRMT();
        Serial.println("ERROR: OUTER switch stays active. Possible wiring fault or stuck switch.");
        homingChangeState(HOMING_ERROR);
      }
      break;

    case HOME_OUTER_START:
      Serial.println("Homing OUTER...");
      homingStateStartSteps = outerAxis.currentSteps;
      outerAxis.startConstantSpeed(OUTER_HOMING_SPEED_STEPS_S, OUTER_HOME_DIR_POSITIVE);
      homingChangeState(HOME_OUTER_SEARCH);
      break;

    case HOME_OUTER_SEARCH:
      if (outerHomeActive()) {
        outerAxis.stopRMT();
        Serial.println("OUTER switch hit.");
        delay(100);
        homingStateStartSteps = outerAxis.currentSteps;
        outerAxis.startConstantSpeed(OUTER_HOMING_SPEED_STEPS_S, !OUTER_HOME_DIR_POSITIVE);
        homingChangeState(HOME_OUTER_BACKOFF);
      }

      if (labs(outerAxis.currentSteps - homingStateStartSteps) > OUTER_MAX_HOMING_STEPS) {
        outerAxis.stopRMT();
        Serial.println("ERROR: OUTER homing travel exceeded.");
        homingChangeState(HOMING_ERROR);
      }
      break;

    case HOME_OUTER_BACKOFF:
      if (outerInsertLimitActive()) {
        homingAbortToError("ERROR: OUTER insert limit active during homing backoff.");
        break;
      }

      if (labs(outerAxis.currentSteps - homingStateStartSteps) >= OUTER_BACKOFF_STEPS) {
        outerAxis.stopRMT();

        if (outerHomeActive()) {
          Serial.println("ERROR: OUTER switch still active after backoff.");
          homingChangeState(HOMING_ERROR);
          break;
        }

        outerAxis.setZero();
        Serial.println("OUTER zero set after backoff.");
        delay(200);
        homingChangeState(HOMING_DONE);
      }
      break;

    case HOME_INNER_PRECHECK:
      Serial.println("Checking INNER switch...");
      if (innerHomeActive()) {
        Serial.println("INNER switch active. Clearing by moving away...");
        homingStateStartSteps = innerAxis.currentSteps;
        innerAxis.startConstantSpeed(INNER_HOMING_SPEED_STEPS_S, !INNER_HOME_DIR_POSITIVE);
        homingChangeState(HOME_INNER_CLEAR);
      } else {
        homingChangeState(HOME_INNER_START);
      }
      break;

    case HOME_INNER_CLEAR:
      if (innerInsertLimitActive()) {
        homingAbortToError("ERROR: INNER insert limit active while clearing home switch.");
        break;
      }

      if (!innerHomeActive()) {
        innerAxis.stopRMT();
        Serial.println("INNER switch cleared.");
        delay(100);
        homingChangeState(HOME_INNER_START);
      }

      if (labs(innerAxis.currentSteps - homingStateStartSteps) > INNER_SWITCH_CLEAR_STEPS) {
        innerAxis.stopRMT();
        Serial.println("ERROR: INNER switch stays active. Possible wiring fault or stuck switch.");
        homingChangeState(HOMING_ERROR);
      }
      break;

    case HOME_INNER_START:
      Serial.println("Homing INNER...");
      homingStateStartSteps = innerAxis.currentSteps;
      innerAxis.startConstantSpeed(INNER_HOMING_SPEED_STEPS_S, INNER_HOME_DIR_POSITIVE);
      homingChangeState(HOME_INNER_SEARCH);
      break;

    case HOME_INNER_SEARCH:
      if (innerHomeActive()) {
        innerAxis.stopRMT();
        Serial.println("INNER switch hit.");
        delay(100);
        homingStateStartSteps = innerAxis.currentSteps;
        innerAxis.startConstantSpeed(INNER_HOMING_SPEED_STEPS_S, !INNER_HOME_DIR_POSITIVE);
        homingChangeState(HOME_INNER_BACKOFF);
      }

      if (labs(innerAxis.currentSteps - homingStateStartSteps) > INNER_MAX_HOMING_STEPS) {
        innerAxis.stopRMT();
        Serial.println("ERROR: INNER homing travel exceeded.");
        homingChangeState(HOMING_ERROR);
      }
      break;

    case HOME_INNER_BACKOFF:
      if (innerInsertLimitActive()) {
        homingAbortToError("ERROR: INNER insert limit active during homing backoff.");
        break;
      }

      if (labs(innerAxis.currentSteps - homingStateStartSteps) >= INNER_BACKOFF_STEPS) {
        innerAxis.stopRMT();

        if (innerHomeActive()) {
          Serial.println("ERROR: INNER switch still active after backoff.");
          homingChangeState(HOMING_ERROR);
          break;
        }

        innerAxis.setZero();
        Serial.println("INNER zero set after backoff.");
        delay(200);
        homingChangeState(HOME_OUTER_PRECHECK);
      }
      break;

    case HOMING_DONE:
      outerAxis.stopRMT();
      innerAxis.stopRMT();
      break;

    case HOMING_ERROR:
      outerAxis.stopRMT();
      innerAxis.stopRMT();
      break;
  }
}
