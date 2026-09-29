# Remus Sensor 2.2.0

Minor release adding phone-centralized, receiver-native GNSS capture from the
Remus Computer without changing MPU-6050 acquisition or pin assignments.

- Adds CRC-protected binary GNSS observation batches on characteristic
  `beb54845-36e1-4688-b7f5-ea07361b26a8`.
- Preserves ESP32 monotonic time, GPS time-of-week, position, Doppler speed,
  course, receiver accuracy, fix state, satellites and SNR at the receiver's
  actual cadence (5 Hz when the acknowledged UBX profile is active).
- Batches up to two observations and schedules GNSS after IMU so radio pressure
  cannot block the 200 Hz acquisition path.
- Adds independent GNSS sequences, queue-loss accounting and capability bit 5.
- Keeps the one-second textual snapshot and NMEA fallback compatible.
- Firmware and shared library version: 2.2.0.

No APK or IPA is produced by this repository.
