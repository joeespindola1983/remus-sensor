# Autonomous PC + dual-Blade field profile

Status: experimental field build (`2.3.0-field.1`), not a replacement for the
qualified `2.2.0` release.

## Behavior

Build and upload the `remus-proto1-autonomous` PlatformIO environment. It:

- requires a working MPU-6050, mounted MicroSD, recording buffer and Blade
  relay task before autostart;
- starts the Computer RBP2 recording automatically after boot;
- scans for up to two Remus Blades, commands each connected Blade to stream and
  stores its original notifications in a separate RBR1 sidecar;
- retains the Computer's 200 Hz IMU and coherent 5 Hz GNSS in RBP2;
- flushes the RBP2 and open Blade sidecars every five seconds;
- retains USB `s` as the clean manual START/STOP toggle.

When no slot assignment exists, discovery fills the two provisional channels.
The sidecar header preserves the actual Blade identity, but left/right labels
are not authoritative until the slots have been explicitly assigned.

The Blades are already connection-gated: after power-on they initialize the
IMU once and advertise their identity, but do not sample continuously or emit
200 Hz telemetry. Sampling begins only after a BLE central connects and sends
the accepted `StartStream` command. A disconnect clears the request and the
Blade returns to advertising. It is therefore appropriate to power the Blades
on at the paddles before launching and power the Computer on only after it is
installed in the boat.

## Bench acceptance before water

1. Insert the intended FAT32 MicroSD before power-on.
2. Power both Blades, then power the Remus Computer.
3. Confirm the serial log contains `Perfil autônomo pronto`, an RBP2 filename,
   two Blade connection messages and two Blade backup filenames.
4. Move all three devices for at least five minutes.
5. Send `s` over USB for a clean stop and confirm non-zero received/persisted
   counts for both Blade sidecars with zero queue/write failures.
6. Download and audit the RBP2 plus both RBR1 files before field use.

If the Computer reports an IMU, SD, queue or relay-task failure, autostart is
blocked instead of creating a misleading partial session.

## Power-off limitation

There is no unused physical button in the current P1 pinout. Disconnecting
power is therefore not a clean stop. The formats tolerate a truncated final
record, and the five-second flush limits the likely lost tail, but abrupt power
loss can still damage the FAT filesystem. For the field test, keep power stable
and use USB `s` whenever a clean stop is possible.

## Rollback

Build and upload `remus-proto1` (release firmware `2.2.0`). That environment
keeps Blade relay and autonomous start disabled and restores the phone-direct
capture profile.
