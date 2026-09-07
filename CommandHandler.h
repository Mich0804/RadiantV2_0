#pragma once

#include "BluetoothSerial.h"
#include "StageStateMachine.h"

extern BluetoothSerial SerialBT;

void commandPrintHelp(Print &out) {
  out.println("Commands:");
  out.println("  OUTER <mm>");
  out.println("  INNER <mm>");
  out.println("  ROT <deg>");
  out.println("  HOME");
  out.println("  STOP");
  out.println("  ZERO_ROT");
  out.println("  STATUS");
  out.println("  LOG_START");
  out.println("  LOG_STOP");
  out.println("  LOG_DUMP");
  out.println("  LOG_CLEAR");
}

void commandReply(Print &out, const char *message) {
  out.println(message);
}

void commandHandleLine(String line, Print &out) {
  line.trim();
  if (line.length() == 0) {
    return;
  }

  char cmdBuffer[20] = {0};
  float value = 0.0f;
  int parsed = sscanf(line.c_str(), "%19s %f", cmdBuffer, &value);

  String command = String(cmdBuffer);
  command.toUpperCase();

  if (command == "HOME") {
    stageHome();
    commandReply(out, "OK HOME");
    return;
  }

  if (command == "STOP") {
    stageStopAll();
    stageChangeState(STOPPED);
    commandReply(out, "OK STOP");
    return;
  }

  if (command == "ZERO_ROT") {
    stageZeroRot();
    commandReply(out, "OK ZERO_ROT");
    return;
  }

  if (command == "STATUS") {
    stagePrintStatus(out);
    return;
  }

  if (command == "HELP" || command == "?") {
    commandPrintHelp(out);
    return;
  }

  if (command == "LOG_START") {
    telemetryStart();
    commandReply(out, "OK LOG_START");
    return;
  }

  if (command == "LOG_STOP") {
    telemetryStop();
    commandReply(out, "OK LOG_STOP");
    return;
  }

  if (command == "LOG_CLEAR") {
    telemetryClear();
    commandReply(out, "OK LOG_CLEAR");
    return;
  }

  if (command == "LOG_DUMP") {
    if (telemetryRecording) {
      commandReply(
          out,
          "ERR stop logging before dump");
      return;
    }

    if (!outerAxis.atTarget() ||
        !innerAxis.atTarget())
    {
      commandReply(
          out,
          "ERR motion active; dump rejected");
      return;
    }

    telemetryDump(out);
    return;
  }

  if (parsed != 2) {
    commandReply(out, "ERR expected command value");
    commandPrintHelp(out);
    return;
  }

  if (!stageCanAcceptMotion()) {
    commandReply(out, "ERR homing required before motion");
    return;
  }

  if (command == "OUTER") {
    if (stageSetOuterMm(value)) {
      commandReply(out, "OK OUTER");
    } else {
      commandReply(out, "ERR OUTER rejected");
    }
    return;
  }

  if (command == "INNER") {
    if (stageSetInnerMm(value)) {
      commandReply(out, "OK INNER");
    } else {
      commandReply(out, "ERR INNER rejected");
    }
    return;
  }

  if (command == "ROT") {
    if (stageSetRotDeg(value)) {
      commandReply(out, "OK ROT");
    } else {
      commandReply(out, "ERR ROT rejected");
    }
    return;
  }

  commandReply(out, "ERR unknown command");
  commandPrintHelp(out);
}

void commandPollStream(Stream &stream, Print &out) {
  while (stream.available()) {
    String line = stream.readStringUntil('\n');
    commandHandleLine(line, out);
  }
}

void commandUpdate() {
  commandPollStream(Serial, Serial);
  commandPollStream(SerialBT, SerialBT);
}
