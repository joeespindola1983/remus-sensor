# Remus Computer headless operation plan

Status: implementation plan for operating the Remus Computer without the
GMT024/ST7789 TFT. This document does not claim that display-presence detection
exists in the current hardware.

## 1. Hardware constraint

The current TFT connection exposes clock, data, data/command, reset and chip
select from the ESP32 to the display. It has no readable identification or
health signal. A disconnected ST7789 can therefore accept SPI writes from the
firmware's perspective without returning proof that the panel is present.

`displayDevice.begin()` must not be treated as physical presence detection.
Headless operation must be selected explicitly rather than inferred from the
absence of a response that this wiring cannot provide.

## 2. Desired behavior

Introduce a persisted `displayMode` configuration with these values:

- `enabled`: initialize and update the configured display;
- `disabled`: never initialize the display bus or run presentation-only work.

The default remains `enabled` for existing assembled units. A Remus Computer
whose TFT has been disconnected is configured once as `disabled`. The selected
mode survives restart and is reported to the phone and USB diagnostics.

`displayMode` describes configuration, not detected hardware presence. Runtime
status must separately report whether display initialization was attempted and
whether the driver initialized. Neither field may claim that the panel is
physically present.

## 3. Firmware work

1. Add a small shared display-runtime policy that combines the hardware
   profile's `hasDisplay` capability with the persisted `displayMode`.
2. Store the user selection in its own Preferences/NVS namespace with a
   versioned default.
3. Add idempotent control and query commands over BLE and USB recovery:
   `DISPLAY_MODE,ENABLED`, `DISPLAY_MODE,DISABLED` and `DISPLAY_MODE?`.
4. Apply a mode change on the next boot. Do not reconfigure display GPIOs while
   a recording is active.
5. When disabled, skip:
   - `displayDevice.begin()` and its visual self-test;
   - `updateDisplay()` calls and `DisplayTelemetry` assembly;
   - display-only median histograms and their periodic sampling.
6. Keep acquisition, live SPM estimation, GNSS, microSD recording, BLE
   telemetry, Blade traffic and file transfer unchanged.
7. Report the configured mode in boot logs, USB status and the versioned device
   diagnostics consumed by the recorder. A protocol extension must remain
   backward compatible with Recorder versions that do not know this field.

## 4. Recorder work

1. Show the Remus Computer display mode in device diagnostics.
2. Provide a guarded setting to enable or disable it, explaining that the
   physical TFT must be disconnected only while the unit is powered off.
3. Mark the setting as pending restart after a successful command.
4. Do not change recording readiness when the display is disabled; a display
   is not required evidence equipment.

## 5. Tests and acceptance

Add unit tests before firmware changes:

- existing units default to `enabled` when no preference exists;
- persisted `disabled` survives restart;
- a headless boot never calls display initialization or rendering;
- display-only histograms remain untouched in headless mode;
- recording, BLE snapshots and file transfer behave identically in both modes;
- invalid commands do not alter the persisted value;
- mode changes during recording are rejected or deferred explicitly;
- diagnostics distinguish configured mode from initialization outcome.

Bench acceptance with the TFT physically disconnected:

1. Boot without display initialization attempts or display GPIO traffic.
2. Start, monitor and stop a recording using only the phone.
3. Confirm 200 Hz acquisition and complete microSD evidence.
4. Confirm live SPM/GNSS telemetry and export over BLE.
5. Compare average current over at least 30 minutes against the same hardware
   with the TFT connected and enabled. Record voltage, battery, firmware,
   sampling configuration and radio conditions.

The battery benefit is accepted only from this current measurement. Reduced
CPU work is useful, but the principal expected saving comes from physically
removing the TFT and its backlight load.
