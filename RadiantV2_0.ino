#include "BluetoothSerial.h"

#include "Config.h"
#include "RmtStepperAxis.h"
#include "RotMotor.h"
#include "HomingController.h"
#include "AS5600Encoder.h"
#include "StageStateMachine.h"
#include "CommandHandler.h"

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  delay(1000);

  if (!SerialBT.begin(BT_DEVICE_NAME)) {
    Serial.println("Bluetooth init failed");
    while (true) {
      delay(1000);
    }
  }

  Serial.print("Bluetooth ready: ");
  Serial.println(BT_DEVICE_NAME);
  commandPrintHelp(Serial);

  stageBegin();
}

void loop() {
  static unsigned long lastStatusMs = 0;

  commandUpdate();
  stageUpdate();

  if (millis() - lastStatusMs >= STATUS_INTERVAL_MS) {
    lastStatusMs = millis();
    stagePrintStatus(Serial);
  }
}
