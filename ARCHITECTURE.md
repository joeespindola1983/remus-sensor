# REMUS architecture

```text
                         remus-core
       +-------------------------------------------+
       | LiveSpmEstimator                         |
       | BladeOrientationEstimator                |
       | Versioned RBP SessionFormat              |
       | SplitCalculator                          |
       | HAL data contracts                       |
       +----------------------+--------------------+
                              |
             +----------------+----------------+
             |                                 |
     Prototype 1 / ESP32                Prototype 2 / Linux
     +--------------------+              +---------------------+
     | FreeRTOS           |              | std::thread         |
     | Wire/SPI/SD        |              | /dev/i2c-1          |
     | HardwareSerial     |              | /dev/serial0        |
     | Arduino BLE        |              | filesystem          |
     | MicroSD            |              | SSD1306             |
     | ST7789 TFT         |              |                     |
     +--------------------+              +---------------------+
```

## Rule

The core never includes Arduino, FreeRTOS, Wire, SPI, BlueZ or Linux device headers.

A sensor algorithm receives canonical values, not GPIOs. A hardware target is responsible for turning its physical device into those canonical values.

## Dual-Blade orientation boundary

`BladeOrientationEstimator` consumes one source's canonical acceleration,
angular velocity and native timestamps. It produces
`equipmentOrientationQuaternion` plus `orientationQuality`; it never performs
BLE, display or storage work. A mounting reference is mandatory before a pair
can produce `relativeEquipmentAlignmentDegrees`.

Two estimators remain independent because each Blade has its own clock. The
Remus Computer may compare them only after mapping both samples into a common
clock with retained uncertainty. Notification arrival order is not clock
alignment. With the current six-axis MPU-6050, yaw remains drift-prone and the
result is relative/calibrated orientation, not absolute heading.

The relay implementation has explicit left/right channel state, but the current
field-development Computer build sets `REMUS_ENABLE_BLADE_RELAY=0`. In that
profile the Computer does not scan, connect, command, persist, relay or orient
Blade sources; the iPhone owns two direct Blade connections. The Computer keeps
its own RBP2 and app stream. The relay code remains build-selectable for
controlled rollback and must never be active simultaneously with direct mobile
capture of the same Blade.

## Why Prototype 2 is not a PlatformIO environment

A Raspberry Pi Zero W is a Linux computer, not another ESP32 board definition. Its executable is compiled with CMake and runs as a Linux process. PlatformIO remains the build system for embedded ESP32 targets only.

## Future Blade

The future Blade target can be added under `platforms/esp32/` as another ESP32 hardware profile with IMU + BLE and no GPS/SD. `LiveSpmEstimator` remains untouched.

The concrete build, protocol, identity, recorder and qualification sequence is
defined in [BLADE_FIRMWARE_PLAN.md](BLADE_FIRMWARE_PLAN.md).
