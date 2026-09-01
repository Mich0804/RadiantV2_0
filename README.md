# Radiant V2.0 Firmware and Demo Scripts

This folder contains the main ESP32-WROOM / ESP32-WROOM-DA firmware for the Radiant V2 translation and rotation platform.

The firmware keeps translation step generation on ESP32 RMT peripherals, controls the Maxon rotation motor with PID and PCNT encoder feedback, monitors fail-safe normally-closed switches, and reads two AS5600 spindle encoders as independent supervisory sensors.

This is a research/demo control stack. Always test motion with the mechanism unloaded first, keep an emergency stop available, and supervise all hardware execution.

## Main Files

- `RadiantV2.0.ino` - Arduino entry point; starts Bluetooth, prints command help, starts the stage state machine, and periodically prints status.
- `Config.h` - pin map, calibration constants, homing speeds, RMT timing, PID gains, gear ratio, AS5600 timing.
- `RmtStepperAxis.h` - non-blocking translation-axis control using RMT step pulses and software position estimates.
- `HomingController.h` - existing homing logic for outer and inner translation axes.
- `SafetySwitches.h` - fail-safe NC switch helpers and pin-mode setup.
- `AS5600Encoder.h` - dual-I2C AS5600 spindle encoder readout and multi-turn tracking.
- `RotMotor.h` - Maxon/TB6612FNG rotation motor, quadrature encoder PCNT, PID position control, and needle gear-ratio conversion.
- `StageStateMachine.h` - high-level system state, command acceptance, safety checks, homing coordination, and status output.
- `CommandHandler.h` - serial/Bluetooth command parser.

## Hardware Pin Map

### Translation Steppers

| Function | GPIO |
| --- | ---: |
| Outer STEP | 16 |
| Outer DIR | 17 |
| Inner STEP | 18 |
| Inner DIR | 19 |
| Stepper enable | 23 |

`DRV_ENABLE_ACTIVE = LOW`, `DRV_ENABLE_INACTIVE = HIGH`.

### Fail-Safe NC Switches

All switches are normally closed:

- `LOW` = normal, switch closed, wiring connected.
- `HIGH` = active, pressed, unplugged, or broken/open circuit.

| Switch | GPIO | ESP32 Mode | Behavior |
| --- | ---: | --- | --- |
| Outer home | 32 | `INPUT_PULLUP` | Prevents further outer retraction |
| Inner home | 33 | `INPUT_PULLUP` | Prevents further inner retraction |
| Outer inserted limit | 36 | `INPUT` | Prevents further outer insertion |
| Inner inserted limit | 39 | `INPUT` | Prevents further inner insertion |
| Platform collision | 5 | `INPUT_PULLUP` | Stops both translation axes and enters `TRANSLATION_FAULT` |

GPIO36 and GPIO39 are input-only and have no internal pull-ups. Wire each with an external pull-up.

### Maxon Rotation Motor

| Function | GPIO |
| --- | ---: |
| TB6612 PWM | 25 |
| TB6612 IN1 | 26 |
| TB6612 IN2 | 27 |
| TB6612 STBY | 14 |
| Encoder A | 34 |
| Encoder B | 35 |

The firmware applies the external gear ratio from the rotation characterization test:

- Motor/encoder gear: `24` teeth.
- Needle/output gear: `30` teeth.
- External gears reverse direction by default: `GEAR_DIRECTION_SIGN = -1.0`.

Therefore `ROT <deg>` commands the needle/output angle, not the raw motor shaft angle.

### AS5600 Spindle Encoders

Both AS5600 sensors use fixed I2C address `0x36`, so they are placed on separate ESP32 I2C controllers.

| Encoder | SDA | SCL | I2C Controller |
| --- | ---: | ---: | ---: |
| Outer spindle AS5600 | 21 | 22 | 0 |
| Inner spindle AS5600 | 4 | 13 | 1 |

Module wiring:

- `VCC -> 3.3 V`
- `GND -> GND`
- `DIR -> GND`
- `OUT` unused
- Breakout-board I2C pull-ups are already present and used.

The firmware reads `STATUS 0x0B` and `RAW_ANGLE 0x0C/0x0D` in one 3-byte transaction at `400 kHz`, every `10 ms`.

AS5600 readout is supervisory only. It is not yet used for closed-loop stepper control.

## Mechanical Calibration

Current translation calibration:

```cpp
OUTER_STEPS_PER_MM = 400.0f;
INNER_STEPS_PER_MM = 800.0f;
```

Both stepper drivers are connected similarly, however, somewhere in the hardware there must be something wrong as the microstepping is clearly not similar. With 400 and 800, they go equally fast. 

Spindle encoder conversion:

```cpp
SPINDLE_LEAD_MM_PER_REV = 2.0f;
AS5600_COUNTS_PER_REV = 4096;
linear_mm = accumulatedCounts / 4096.0f * 2.0f;
```

Outer validation showed `3` commanded revolutions = `2400` outer step pulses and `12288` AS5600 counts, so `OUTER_STEPS_PER_MM = 400.0f` is kept.

## Firmware Commands

Commands are accepted over USB serial and Bluetooth serial.

| Command | Meaning |
| --- | --- |
| `HOME` | Start homing sequence |
| `OUTER <mm>` | Move outer translation axis to absolute mm |
| `INNER <mm>` | Move inner translation axis to absolute mm |
| `ROT <deg>` | Move needle/output rotation to absolute degrees |
| `STOP` | Stop all axes and enter `STOPPED` |
| `ZERO_ROT` | Zero the current rotation encoder count |
| `STATUS` | Print system state, targets, switches, AS5600 readouts, and Bluetooth state |
| `HELP` or `?` | Print command list |

Example:

```text
STATUS
OUTER 25
INNER 37
ROT 45
INNER 25
```

Important rotation rule for scripted demos: rotate only while the inner cannula is fully retracted to the current outer depth.

## State and Safety Behavior

Startup sequence:

1. Bluetooth starts as `Radiant1.1`.
2. Hardware drivers and sensors initialize.
3. Firmware starts homing.
4. When homing finishes, translation positions and AS5600 accumulated counts are zeroed.
5. State changes to `NORMAL_RUN`.

Runtime safety:

- Outer home switch active stops/rejects further outer retraction.
- Inner home switch active stops/rejects further inner retraction.
- Outer inserted limit active stops/rejects further outer insertion.
- Inner inserted limit active stops/rejects further inner insertion.
- Platform collision active stops both translation axes and enters `TRANSLATION_FAULT`.
- Open switch wiring is treated exactly like an active switch.
- AS5600 `magnetTooWeak` is logged as a warning, not a blocking fault.

Status output includes lines like:

```text
state=NORMAL_RUN homing=HOMING_DONE outer_mm=... outer_target_mm=... inner_mm=... inner_target_mm=... rot_deg=... rot_target_deg=... bt=connected
OUTER: HOME=CLEAR INSERT=CLEAR AS5600 connected=YES raw=... angle=... deg counts=... revolutions=... position=... mm magnet=WEAK
INNER: HOME=CLEAR INSERT=CLEAR AS5600 connected=YES raw=... angle=... deg counts=... revolutions=... position=... mm magnet=OK
PLATFORM COLLISION=CLEAR
```

## Uploading the Firmware

Using Arduino IDE:

1. Open `Firmware/radiantfirmware/RadiantV2.0/RadiantV2.0.ino`.
2. Select `ESP32-WROOM-DA Module`.
3. Select the ESP32 COM port.
4. Verify/compile.
5. Upload.
6. Open Serial Monitor at `115200 baud`.

Install Espressif ESP32 Arduino board support if the board is not listed.


