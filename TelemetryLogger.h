#pragma once

#include "Config.h"
#include "RmtStepperAxis.h"
#include "AS5600Encoder.h"
#include "RotMotor.h"

struct TelemetrySample {
  uint32_t timeMs;

  float outerReferenceMm;
  float outerMeasuredMm;
  float outerReferenceVelocityMmS;

  float innerReferenceMm;
  float innerMeasuredMm;
  float innerReferenceVelocityMmS;

  float rotationTargetDeg;
  float rotationMeasuredDeg;
  float rotationPidOutputPwm;
  float rotationPTermPwm;
  float rotationITermPwm;
  float rotationDTermPwm;
};

TelemetrySample telemetryBuffer[LOG_CAPACITY];

size_t telemetryWriteIndex = 0;
size_t telemetrySampleCount = 0;

bool telemetryRecording = false;
uint32_t telemetryLastSampleMs = 0;

void telemetryClear()
{
  telemetryWriteIndex = 0;
  telemetrySampleCount = 0;
  telemetryLastSampleMs = 0;
}

void telemetryStart()
{
  telemetryClear();
  telemetryRecording = true;
}

void telemetryStop()
{
  telemetryRecording = false;
}

void telemetryUpdate()
{
  if (!telemetryRecording) {
    return;
  }

  const uint32_t nowMs = millis();

  if (nowMs - telemetryLastSampleMs <
      LOG_SAMPLE_INTERVAL_MS)
  {
    return;
  }

  telemetryLastSampleMs = nowMs;

  TelemetrySample &sample =
      telemetryBuffer[telemetryWriteIndex];

  sample.timeMs = nowMs;

  sample.outerReferenceMm =
      outerAxis.currentMm();

  sample.outerMeasuredMm =
      OUTER_AS5600_POSITION_SIGN *
      as5600CountsToMm(
          outerSpindleEncoder.accumulatedCounts);

  sample.outerReferenceVelocityMmS =
      outerAxis.referenceSpeedMmS();

  sample.innerReferenceMm =
      innerAxis.currentMm();

  sample.innerMeasuredMm =
      INNER_AS5600_POSITION_SIGN *
      as5600CountsToMm(
          innerSpindleEncoder.accumulatedCounts);

  sample.innerReferenceVelocityMmS =
      innerAxis.referenceSpeedMmS();

  sample.rotationTargetDeg =
      rotMotorCountsToDeg(rotReferenceCount);

  sample.rotationMeasuredDeg =
      rotMotorGetDeg();

  sample.rotationPidOutputPwm =
      (float)rotLastPwmCommand;

  sample.rotationPTermPwm =
      rotLastPTermPwm;

  sample.rotationITermPwm =
      rotLastITermPwm;

  sample.rotationDTermPwm =
      rotLastDTermPwm;

  telemetryWriteIndex =
      (telemetryWriteIndex + 1) % LOG_CAPACITY;

  if (telemetrySampleCount < LOG_CAPACITY) {
    telemetrySampleCount++;
  }
}

void telemetryDump(Print &out)
{
  out.println(
      "LOG_BEGIN,"
      "time_ms,"
      "outer_x_ref_mm,"
      "outer_x_meas_mm,"
      "outer_v_ref_mm_s,"
      "inner_x_ref_mm,"
      "inner_x_meas_mm,"
      "inner_v_ref_mm_s,"
      "rot_target_deg,"
      "rot_meas_deg,"
      "rot_pid_out_pwm,"
      "rot_p_term_pwm,"
      "rot_i_term_pwm,"
      "rot_d_term_pwm");

  const size_t oldestIndex =
      telemetrySampleCount < LOG_CAPACITY
          ? 0
          : telemetryWriteIndex;

  for (size_t i = 0;
       i < telemetrySampleCount;
       i++)
  {
    const size_t index =
        (oldestIndex + i) % LOG_CAPACITY;

    const TelemetrySample &sample =
        telemetryBuffer[index];

    out.print("LOG,");
    out.print(sample.timeMs);

    out.print(",");
    out.print(sample.outerReferenceMm, 4);

    out.print(",");
    out.print(sample.outerMeasuredMm, 4);

    out.print(",");
    out.print(
        sample.outerReferenceVelocityMmS,
        4);

    out.print(",");
    out.print(sample.innerReferenceMm, 4);

    out.print(",");
    out.print(sample.innerMeasuredMm, 4);

    out.print(",");
    out.print(
        sample.innerReferenceVelocityMmS,
        4);

    out.print(",");
    out.print(sample.rotationTargetDeg, 4);

    out.print(",");
    out.print(sample.rotationMeasuredDeg, 4);

    out.print(",");
    out.print(sample.rotationPidOutputPwm, 0);

    out.print(",");
    out.print(sample.rotationPTermPwm, 4);

    out.print(",");
    out.print(sample.rotationITermPwm, 4);

    out.print(",");
    out.println(sample.rotationDTermPwm, 4);
  }

  out.print("LOG_END,");
  out.println(telemetrySampleCount);
}