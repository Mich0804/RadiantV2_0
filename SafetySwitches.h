#pragma once

#include "Config.h"

bool safetySwitchActive(uint8_t pin) {
  return digitalRead(pin) == HIGH;
}

bool outerHomeActive() {
  return safetySwitchActive(OUTER_HOME_SW);
}

bool innerHomeActive() {
  return safetySwitchActive(INNER_HOME_SW);
}

bool outerInsertLimitActive() {
  return safetySwitchActive(OUTER_INSERT_LIMIT_SW);
}

bool innerInsertLimitActive() {
  return safetySwitchActive(INNER_INSERT_LIMIT_SW);
}

bool platformCollisionActive() {
  return safetySwitchActive(PLATFORM_COLLISION_SW);
}

const char* safetySwitchStatusName(bool active) {
  return active ? "ACTIVE" : "CLEAR";
}

void safetySwitchesBegin() {
  pinMode(OUTER_HOME_SW, INPUT_PULLUP);
  pinMode(INNER_HOME_SW, INPUT_PULLUP);
  pinMode(PLATFORM_COLLISION_SW, INPUT_PULLUP);

  pinMode(OUTER_INSERT_LIMIT_SW, INPUT);
  pinMode(INNER_INSERT_LIMIT_SW, INPUT);
}
