# Remus Sensor 1.4.0

## Device identity and metadata

- Remus Computer now provisions and retains a persistent `RC-D-XXXXXXXXXXXX`
  serial number in non-volatile storage.
- Remus Computer now exposes the same versioned, read-only Device Info BLE
  characteristic as Remus Blade.
- Device Info v2 truthfully advertises sample rate, accelerometer range,
  gyroscope range and DLPF setting. The binary IMU packet protocol remains v1.
- Remus Blade now uses ±16 g and ±2000 dps to preserve the peaks that saturated
  the real-session capture. Remus Computer remains independently configured at
  ±8 g and ±500 dps.
- The shared protocol declares distinct Computer and Blade family identifiers.

## Versions

- Remus Computer firmware: `1.4.0`.
- Remus Blade firmware: `1.4.0`.
- Binary IMU packet protocol remains version 1.
