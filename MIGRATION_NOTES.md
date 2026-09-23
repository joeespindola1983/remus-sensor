# Migration notes

## What changed

1. Prototype 2 was removed from PlatformIO and redefined as Raspberry Pi Zero W v1.1.
2. The working ESP32 pin map is now explicitly Prototype 1.
3. `LiveSpmEstimator` moved to `lib/remus-core`; its algorithm is unchanged apart from the include path.
4. RBP1 structs moved into the shared `SessionFormat.hpp`.
5. GPS/IMU/storage interfaces no longer require Arduino headers.
6. Prototype 2 has native Linux drivers for MPU-6050, NEO-6M and SSD1306.
7. Prototype 2 records RBP1 `.bin` files to the Linux filesystem.
8. Prototype 2 runs IMU acquisition and Live SPM processing in separate threads.
9. A systemd service is included for autonomous recording on boot.

## Deliberately not ported yet

- Raspberry Pi BlueZ BLE GATT server / iOS-Android live protocol.
- Raspberry Pi BLE file transfer.

These remain platform adapters to implement, not reasons to duplicate the core.
