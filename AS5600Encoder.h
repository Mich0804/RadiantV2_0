#pragma once

#include <Wire.h>

#include "Config.h"

const uint8_t AS5600_ADDRESS = 0x36;
const uint8_t AS5600_STATUS_REGISTER = 0x0B;
const uint8_t AS5600_RAW_ANGLE_REGISTER = 0x0C;
const int32_t AS5600_COUNTS_PER_REV = 4096;

TwoWire outerSpindleEncoderBus = TwoWire(0);
TwoWire innerSpindleEncoderBus = TwoWire(1);

struct AS5600State {
  const char *name;
  TwoWire *bus;
  uint8_t sdaPin;
  uint8_t sclPin;

  uint16_t rawAngle;
  uint16_t previousRawAngle;
  int32_t accumulatedCounts;

  bool firstReading;
  bool connected;
  bool magnetDetected;
  bool magnetTooWeak;
  bool magnetTooStrong;
  bool weakWarningPrinted;
};

AS5600State outerSpindleEncoder = {
  "OUTER",
  &outerSpindleEncoderBus,
  OUTER_AS5600_SDA,
  OUTER_AS5600_SCL,
  0,
  0,
  0,
  true,
  false,
  false,
  false,
  false,
  false
};

AS5600State innerSpindleEncoder = {
  "INNER",
  &innerSpindleEncoderBus,
  INNER_AS5600_SDA,
  INNER_AS5600_SCL,
  0,
  0,
  0,
  true,
  false,
  false,
  false,
  false,
  false
};

float as5600RawToDegrees(uint16_t rawAngle) {
  return ((float)(rawAngle & 0x0FFF) * 360.0f) / (float)AS5600_COUNTS_PER_REV;
}

float as5600CountsToRevolutions(int32_t counts) {
  return (float)counts / (float)AS5600_COUNTS_PER_REV;
}

float as5600CountsToMm(int32_t counts) {
  return as5600CountsToRevolutions(counts) * SPINDLE_LEAD_MM_PER_REV;
}

float as5600PositionMm(
    const AS5600State &encoder,
    float positionSign)
{
  return positionSign *
         as5600CountsToMm(encoder.accumulatedCounts);
}

bool as5600ValidForControl(const AS5600State &encoder)
{
  return encoder.connected &&
         encoder.magnetDetected;
}

bool readAS5600StatusAndRawAngle(TwoWire &bus, uint8_t &status, uint16_t &rawAngle) {
  bus.beginTransmission(AS5600_ADDRESS);
  bus.write(AS5600_STATUS_REGISTER);

  if (bus.endTransmission(false) != 0) {
    return false;
  }

  if (bus.requestFrom(AS5600_ADDRESS, (uint8_t)3) != 3) {
    return false;
  }

  status = bus.read();
  const uint8_t highByte = bus.read();
  const uint8_t lowByte = bus.read();
  rawAngle = (((uint16_t)highByte << 8) | lowByte) & 0x0FFF;
  return true;
}

void updateAS5600AccumulatedCounts(AS5600State &encoder, uint16_t newRawAngle) {
  if (encoder.firstReading) {
    encoder.rawAngle = newRawAngle;
    encoder.previousRawAngle = newRawAngle;
    encoder.firstReading = false;
    return;
  }

  int32_t difference = (int32_t)newRawAngle - (int32_t)encoder.previousRawAngle;

  if (difference < -2048) {
    difference += AS5600_COUNTS_PER_REV;
  } else if (difference > 2048) {
    difference -= AS5600_COUNTS_PER_REV;
  }

  encoder.accumulatedCounts += difference;
  encoder.previousRawAngle = newRawAngle;
  encoder.rawAngle = newRawAngle;
}

bool updateAS5600(AS5600State &encoder) {
  uint8_t status = 0;
  uint16_t rawAngle = 0;

  if (!readAS5600StatusAndRawAngle(*encoder.bus, status, rawAngle)) {
    encoder.connected = false;
    encoder.firstReading = true;
    return false;
  }

  encoder.connected = true;
  encoder.magnetDetected = (status & (1 << 5)) != 0;
  encoder.magnetTooWeak = (status & (1 << 4)) != 0;
  encoder.magnetTooStrong = (status & (1 << 3)) != 0;

  if (encoder.magnetTooWeak && !encoder.weakWarningPrinted) {
    Serial.print("WARNING: ");
    Serial.print(encoder.name);
    Serial.println(" AS5600 magnet weak; continuing supervisory tracking.");
    encoder.weakWarningPrinted = true;
  } else if (!encoder.magnetTooWeak) {
    encoder.weakWarningPrinted = false;
  }

  updateAS5600AccumulatedCounts(encoder, rawAngle);
  return true;
}

void as5600SetZero(AS5600State &encoder) {
  encoder.accumulatedCounts = 0;
  encoder.previousRawAngle = encoder.rawAngle;
  encoder.firstReading = !encoder.connected;
}

const char* as5600MagnetStatusName(const AS5600State &encoder) {
  if (!encoder.connected) {
    return "NOT_CONNECTED";
  }

  if (!encoder.magnetDetected) {
    return "NOT_DETECTED";
  }

  if (encoder.magnetTooWeak) {
    return "WEAK";
  }

  if (encoder.magnetTooStrong) {
    return "STRONG";
  }

  return "OK";
}

void as5600Begin() {
  outerSpindleEncoder.bus->begin(
      outerSpindleEncoder.sdaPin,
      outerSpindleEncoder.sclPin,
      AS5600_I2C_FREQUENCY_HZ);

  innerSpindleEncoder.bus->begin(
      innerSpindleEncoder.sdaPin,
      innerSpindleEncoder.sclPin,
      AS5600_I2C_FREQUENCY_HZ);

  delay(100);

  updateAS5600(outerSpindleEncoder);
  updateAS5600(innerSpindleEncoder);

  Serial.print("Outer AS5600 startup detection: ");
  Serial.println(outerSpindleEncoder.connected ? "OK" : "FAILED");
  Serial.print("Inner AS5600 startup detection: ");
  Serial.println(innerSpindleEncoder.connected ? "OK" : "FAILED");
}
