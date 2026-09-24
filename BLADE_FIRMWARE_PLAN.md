# Remus Blade firmware separation plan

Status: proposed implementation plan; no Blade firmware target exists yet  
Repositories in scope: `remus-sensor`, `remus-recorder`, and the canonical
contracts in `remus-app`  
Baseline inspected: `remus-sensor` `v0.3.2` / commit `262e804`

## Decision

Keep one firmware repository and one PlatformIO project, but build two isolated
ESP32 applications:

- the existing full ESP32-C3 application, currently built as `remus-proto1`;
- a new IMU + BLE Blade application, initially built as `remus-blade-dev` until
  its durable device model and acquisition-profile identifiers are approved.

The Blade must have its own `BladeApp`, hardware profile, build environment and
runtime state machine. It must not be implemented as conditional branches
throughout `RemusApp.cpp`, and it must not copy the legacy one-characteristic
CSV BLE implementation.

This gives the products separate binaries and release lifecycles while retaining
one source of truth for reusable sensor, protocol and test code.

Creating a second repository is not recommended now. It would make the shared
wire contract, MPU-6050 driver and golden fixtures harder to keep identical. A
repository split can be reconsidered only if the products acquire independent
teams or release governance; the shared code would first need to become a
versioned package.

## Findings from the current code

The repository already has the right first boundary:

```text
remus-sensor/
  lib/remus-core/             platform-neutral algorithms and contracts
  platforms/esp32/            Arduino/FreeRTOS hardware implementation
  platforms/raspberrypi/      separate Linux/CMake target
  platformio.ini              currently one ESP32 environment
```

Useful existing pieces are:

- `remus::hal::IImu` and its raw six-axis fields;
- `Mpu6050Imu`, including 400 kHz I2C, 200 Hz configuration, +/-8 g,
  +/-500 degrees/s and coherent 14-byte reads;
- the FreeRTOS acquisition/task separation and RAM buffering pattern in
  `RemusApp.cpp`;
- `LiveSpmEstimator`, if a later Blade capability explicitly enables it;
- hardware-profile validation and host-side smoke tests.

Current constraints that drive the split:

- `platformio.ini` selects only `REMUS_PROFILE_PROTO1`;
- `ActiveProfile.hpp` accepts only `Prototype1`;
- `RemusApp.cpp` owns GPS, MicroSD, display, file transfer, IMU, BLE and UI in
  one translation unit;
- GPS/display libraries and the SD download extra script are global to every
  PlatformIO environment;
- the existing full firmware is already approximately 89.8% of its configured
  application flash partition, so adding Blade behavior to that binary is the
  wrong direction;
- the current BLE path sends a one-second CSV snapshot through one
  read/write/notify characteristic; it is not the 200 Hz evidence path;
- the iOS, Android and TypeScript recorder layers each currently model one
  discovered/connected Blade, not a set of physical units;
- mobile currently uses an iOS peripheral UUID or Android MAC address as the
  device ID and contains hard-coded demo identity/capability metadata;
- the app-side source currently says `sensorPlacement: paddle`, while the
  accepted current RBP1 rowing profile says `sensorPlacement: hull`.

## Contract decision required before durable identifiers

There is a naming collision between current documentation and the new product
direction. The accepted `Remus Blade P1` / `rbp1` profile currently describes
the full hull-mounted ESP32-C3 unit with GNSS, MicroSD and TFT. The proposed new
Blade is an equipment-mounted IMU + BLE node without those peripherals.

Before shipping either identity, update the canonical dictionary, lifecycle
catalog and hardware profiles to distinguish:

1. the full hull unit (the product owner currently calls it Remus or Remus
   Computer); and
2. the new oar- or paddle-associated Blade unit.

Recommended product direction is `Remus Computer` for the full unit and
`Remus Blade` for the small IMU node, but the exact canonical `deviceFamily`,
`deviceModel` and acquisition-profile identifiers are a product/data contract
decision. Existing RBP1/RBP2 recordings remain immutable legacy evidence and
need an explicit migration mapping; their identity must not be silently
rewritten.

Until that decision is approved, use `remus-blade-dev` only as a PlatformIO
environment name. Do not persist it as a production `deviceModel`.

## Target source layout

```text
remus-sensor/
  lib/remus-core/
    include/remus/protocol/       explicit wire types and codecs
    include/remus/acquisition/    platform-neutral sample/gap/clock types
    src/                          platform-neutral implementations

  platforms/esp32/
    include/remus/apps/
      RemusApp.hpp
      BladeApp.hpp
    include/remus/profiles/
      Prototype1.hpp              existing full device, compatibility name
      BladeDev.hpp                new IMU + BLE hardware profile
    include/remus/drivers/
      Mpu6050Imu.hpp
    include/remus/transport/
      BladeBlePeripheral.hpp
    include/remus/identity/
      Esp32DeviceIdentity.hpp
    src/apps/
      RemusApp.cpp
      BladeApp.cpp
    src/drivers/
    src/transport/
    src/identity/
    src/main.cpp                   selects exactly one app at compile time

  tests/
    fixtures/protocol-v1/         golden byte fixtures shared with mobile
```

`remus-core` remains free of Arduino, FreeRTOS, BLE, NVS and GPIO headers. BLE
server calls, FreeRTOS queues/ring buffers and NVS provisioning remain ESP32
adapters.

## PlatformIO environments

Refactor `platformio.ini` so the common `[env]` contains only true ESP32-wide
settings. Target-specific dependencies, scripts, defines and source filters
belong to each environment.

Conceptually:

```ini
[platformio]
default_envs = remus-proto1
src_dir = platforms/esp32/src
include_dir = platforms/esp32/include

[env]
platform = espressif32
board = esp32-c3-devkitm-1
framework = arduino
build_flags =
    -std=gnu++17
    -I lib/remus-core/include

[env:remus-proto1]
build_flags =
    ${env.build_flags}
    -D REMUS_TARGET_FULL=1
    -D REMUS_PROFILE_PROTO1=1
extra_scripts = platforms/esp32/download_sd.py
lib_deps =
    mikalhart/TinyGPSPlus @ ^1.0.3
    moononournation/GFX Library for Arduino @ 1.5.5
build_src_filter =
    +<*>
    -<apps/BladeApp.cpp>
    -<transport/BladeBlePeripheral.cpp>

[env:remus-blade-dev]
build_flags =
    ${env.build_flags}
    -D REMUS_TARGET_BLADE=1
    -D REMUS_PROFILE_BLADE_DEV=1
build_src_filter =
    +<main.cpp>
    +<apps/BladeApp.cpp>
    +<drivers/Mpu6050Imu.cpp>
    +<transport/BladeBlePeripheral.cpp>
    +<identity/Esp32DeviceIdentity.cpp>
```

The final source filters must be verified against PlatformIO's compiled source
list. The Blade binary must not link GNSS, SD, TFT or the legacy BLE/file
transfer application.

Expected commands:

```bash
pio run -e remus-proto1
pio run -e remus-blade-dev
pio run -e remus-proto1 -e remus-blade-dev
pio run -e remus-blade-dev -t upload
pio device monitor -b 115200
```

A small `scripts/firmware` wrapper may later expose `build`, `upload` and
`monitor` with a `full|blade` argument, but it must only call these named
PlatformIO environments. The environments remain the source of truth, so the
project also works directly in the PlatformIO UI.

## Hardware profile and wiring

The current full prototype profile is:

- MPU SDA GPIO 5;
- MPU SCL GPIO 6.

The proposed Blade wiring reported by the product owner is the reverse:

- MPU SCL GPIO 5;
- MPU SDA GPIO 6.

`BladeDev.hpp` must encode the new Blade wiring as `{sda: 6, scl: 5}` without
changing `Prototype1.hpp`. Before the first upload, verify the labels on the
exact GY-521 and ESP32-C3 boards; a swapped logical profile will simply fail to
find addresses `0x68`/`0x69`.

Only SDA/SCL have been declared. The MPU-6050 data-ready interrupt design needs
an additional INT-to-GPIO wire and a selected safe GPIO. Therefore the first
bring-up may use the existing 5 ms acquisition task, but the profile must leave
the interrupt pin explicitly unassigned rather than inventing one. FIFO/data-
ready work starts only after the pin and electrical behavior are confirmed.

Battery percentage is unavailable until the actual battery, charger/protection
and measurement circuit are declared. Firmware must not advertise a fabricated
`batteryLevelPercent`.

## What is shared and what remains separate

| Concern | Share? | Boundary |
|---|---:|---|
| MPU-6050 register driver and raw sample type | Yes | Configuration becomes explicit and read back after boot |
| Raw units, timestamps, sequences and gap diagnostics | Yes | Platform-neutral acquisition contract |
| Binary message definitions and encoder/decoder | Yes | Explicit little-endian codec; never transmit a C++ struct by memory copy |
| Golden protocol fixtures and CRC/checksum code | Yes | Same bytes validated in C++, TypeScript, Swift and Kotlin |
| Live SPM estimator | Optional | Capability-gated; not part of the 200 Hz critical path |
| FreeRTOS queue/ring buffer | ESP32 only | Shared ESP32 utility is possible; never enters `remus-core` |
| NVS identity implementation | ESP32 only | Implements a shared identity contract |
| BLE GATT server | Blade-specific initially | Do not copy the legacy mixed CSV/file characteristic |
| GPS, MicroSD, TFT and file transfer | Full unit only | Excluded from the Blade build |
| Top-level lifecycle | Separate | `RemusApp` and `BladeApp` are independent applications |
| “Blade 01”, “Blade 02” aliases | App only | Persisted registry keyed by durable physical identity |

## Blade runtime pipeline

```text
MPU-6050 coherent read at 200 Hz
  -> RawImuFrame { sampleSequence, nativeTimestampUs, six int16 axes, status }
  -> bounded RAM ring buffer
  -> low-priority BLE batching task
  -> versioned binary notification frames
  -> phone persistence
```

The acquisition task must never call BLE. The BLE task may consume up to a
target of ten samples per logical batch, but batch size is negotiated/adapted to
the usable notification payload. The RAM buffer must declare its duration and
overflow behavior. Overflow or acquisition failure increments diagnostics and
creates observable sequence gaps; firmware never repeats the last sample to
hide a failure.

Initial state machine:

```text
boot -> advertising -> connected_idle -> streaming -> connected_idle
                     \-> fault/degraded -> reconnect/reinitialize
```

The 200 Hz stream runs only in `streaming`. Status/health may continue at a low
rate while connected idle.

Because the proposed Blade has no declared local persistence, its first
capability manifest must state `standaloneCapture: false` and
`storeAndForward: false`. A disconnect longer than the bounded RAM window
creates an explicit gap. If app-absent capture is a product requirement, add a
qualified flash/SD spool design rather than claiming that RAM provides durable
recording.

## BLE protocol v1 requirements

The study's ten raw samples per packet is a good throughput target, not yet a
wire contract. Freeze the wire format only after golden fixtures exist.

The protocol must provide:

- protocol version and message kind;
- durable physical identity and boot identity during handshake;
- `deviceFamily`, `deviceModel`, hardware revision, firmware version and real
  capability/configuration read-back;
- stream/batch sequence and first sample sequence;
- native monotonic acquisition time, not notification arrival time;
- raw `int16` accelerometer and gyroscope axes;
- nominal sample period plus enough per-sample timing/status information to
  expose jitter, failed reads and dropped samples;
- bounded payload length and explicit byte order;
- fragmentation/reassembly or adaptive batching for negotiated ATT payloads;
- idempotent start/stop requests with acknowledgements and source recording
  identity;
- status/health notifications and a reserved clock-synchronization exchange.

Recommended GATT responsibilities:

```text
Remus Device service
  Device Info / Capabilities     read
  Control                        write, acknowledged at protocol level
  IMU Stream                     notify
  Status                         read + notify
  Clock Sync                     write + notify
```

Advertising identifies the family, protocol major version, short public code
and capabilities needed for discovery. It does not stream IMU samples and does
not authorize control. BLE names are human hints only.

Do not require MTU 512. Test at least the default 23-byte MTU and typical larger
mobile MTUs. If one logical batch does not fit, a versioned fragment envelope
must allow deterministic reassembly and loss detection. Link-layer reliability
does not replace application sequences, bounded resend semantics or explicit
gaps.

## Identity and user-visible numbering

The firmware never decides that it is “Blade 01”. It exposes one durable
physical identity and a short public code. The app owns a persistent registry:

```text
deviceSerialNumber / internal identity -> short code -> user alias “Blade 01”
```

Production identity is provisioned and stored in NVS independently from BLE
name and public radio address. For development only, a deterministic identity
derived from the ESP32 eFuse may be used as a clearly marked fallback; it must
not be presented as a provisioned production serial number. A 16-bit MAC suffix
is insufficient as the durable key.

## Recorder changes required for a useful Blade target

Firmware-only success is not enough because the current recorder is single-
device and legacy-CSV oriented.

The `remus-recorder` track must:

1. add golden binary frame decoding before enabling the firmware stream;
2. replace single `discoveredPeripheral`, `bluetoothGatt` and adapter identity
   fields with maps keyed by the protocol's durable physical identity;
3. retain OS peripheral UUID/MAC only as a transport locator;
4. discover Device Info/Control/IMU/Status/Clock characteristics;
5. persist one `recording` and `sensorStream` per Blade with its own clock;
6. validate sequence continuity, deduplicate frames and record explicit gaps;
7. persist user aliases independently from recording/mounting assignments;
8. remove hard-coded demo serial, firmware, placement and capabilities;
9. retain `sensorPlacement` and `mountingConfiguration` as explicit session
   context rather than inferring them from the product name;
10. support two simultaneous physical Blades before any bilateral comparison.

Legacy full-unit CSV and RBP1/RBP2 file transfer stay behind a compatibility
adapter until deliberately retired. New binary live frames must not be parsed
as UTF-8 CSV.

## Delivery phases

### Phase 0 — resolve contracts and freeze the development hardware

- approve full-unit versus Blade product identity and migration mapping;
- update the canonical dictionary/catalog and hardware/acquisition profiles;
- record exact ESP32-C3 and MPU-6050 board markings, power design and Blade
  wiring revision;
- confirm SDA 6 / SCL 5 and decide whether an INT GPIO will be wired;
- decide whether connected-only capture is acceptable for the first Blade.

Exit: no ambiguous durable identifier or false capability is needed by code.

### Phase 1 — split the builds without changing behavior

- introduce target-specific PlatformIO environments and source filters;
- add `BladeDev.hpp`, `BladeApp` scaffold and compile-time app selection;
- move GPS/GFX dependencies and SD scripts to the full-unit environment;
- expand hardware-profile smoke tests for both GPIO maps and capabilities;
- keep `remus-proto1` behavior and binary size regression-visible.

Exit: both environments compile independently; Blade links no GPS/SD/TFT code.

### Phase 2 — protocol-first host tests

- define explicit message codecs, compatibility rules and golden byte fixtures;
- test truncation, unknown versions/types, invalid counts, CRC/checksum,
  fragmentation, duplicate/reordered frames and 64-bit timestamp handling;
- consume the same fixtures from firmware C++, TypeScript, Swift and Kotlin;
- add a transport simulator before physical multi-device work.

Exit: every language agrees on bytes, units, nullability and failure behavior.

### Phase 3 — one-Blade bring-up

- provision/read identity and advertise the development capability manifest;
- initialize MPU-6050 on Blade SDA 6 / SCL 5 and verify WHO_AM_I/config read-back;
- run acquisition, ring buffer and BLE tasks separately;
- implement connected-idle/start/stream/stop/status;
- record sequences, native timestamps, I2C failures, buffer high-water mark and
  overflow count.

Exit: one physical Blade streams raw 200 Hz evidence for a sustained bench run
with measured rate/jitter/gaps and no fabricated continuity.

### Phase 4 — recorder persistence and recovery

- decode/persist binary batches as an immutable acquisition artifact;
- convert raw axes only at the declared adapter boundary;
- implement app restart, disconnect and bounded-resend/gap behavior;
- expose health/readiness without claiming unsupported GPS, battery, storage or
  derived sporting metrics.

Exit: a recorded app artifact can be replayed to reproduce samples and gaps.

### Phase 5 — two Blades and clock mapping

- refactor native BLE bridges and TypeScript service for multiple peripherals;
- assign stable app aliases such as Blade 01/02 from durable identity;
- preserve independent `clockDomain` values;
- implement repeated clock-anchor exchanges and estimate offset/drift with
  uncertainty; never align by notification arrival time;
- test simultaneous start/stop, reconnect and one-unit failure.

Exit: two units stream concurrently and remain separate recordings; aligned
comparison is enabled only when a qualified clock mapping exists.

### Phase 6 — FIFO/data-ready and product qualification

- add the confirmed INT GPIO and move to data-ready/FIFO acquisition;
- characterize overflow, saturation, reset, brownout, radio pressure and power;
- qualify mounting orientation, repeatability, enclosure and safety;
- set measured acceptance thresholds before claiming dataset readiness or
  sporting outputs.

Exit: hardware-in-the-loop acceptance gates are documented and reproducible.

## Verification gates

Every phase keeps these commands green:

```bash
./tests/validate_project.sh
pio run -e remus-proto1
pio run -e remus-blade-dev
```

Additional Blade gates:

- compile-time validation rejects missing/duplicate GPIOs and impossible
  capability combinations;
- binary size report proves the Blade build excludes full-unit subsystems;
- a fixture corpus is byte-identical across all consumers;
- sustained tests report requested and observed sampling rates, interval
  distribution, sequence gaps, I2C failures, buffer high-water/overflow and BLE
  frame/fragment loss;
- tests cover MTU variation, delayed notifications, duplication, reordering,
  reconnect, reset and 64-bit clock precision;
- two-unit tests prove identity and samples never cross between sources;
- unavailable battery, GPS, storage and derived metrics remain unavailable,
  never zero or invented.

## Recommended first implementation slice

Implement only Phase 1 after the Phase 0 naming/wiring decisions. It is a small,
reversible change that creates the correct compile boundary without prematurely
freezing the BLE protocol. The following slice should be Phase 2 golden protocol
fixtures, followed by one-device hardware bring-up.
