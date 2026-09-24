# REMUS Sensor — multi-platform firmware

This repository has one shared REMUS core and three hardware targets:

- **Prototype 1 — ESP32-C3**: MPU-6050 + NEO-6M + MicroSD + TFT + BLE, built with PlatformIO.
- **Remus Blade — ESP32-C3**: MPU-6050 + BLE live streaming, built with PlatformIO.
- **Prototype 2 — Raspberry Pi Zero W v1.1**: MPU-6050 + NEO-6M + SSD1306 + Linux filesystem, built with CMake.

The live SPM estimator and versioned RBP binary record definitions are shared. Platform code is isolated below `platforms/`.

## Layout

```text
remus-sensor/
├── lib/remus-core/                   # no Arduino/Linux dependency
│   ├── include/remus/core/
│   │   ├── LiveSpmEstimator.hpp
│   │   ├── SessionFormat.hpp
│   │   └── SplitCalculator.hpp
│   ├── include/remus/hal/
│   │   ├── IImu.hpp
│   │   ├── IGps.hpp
│   │   └── IStorage.hpp
│   └── src/LiveSpmEstimator.cpp
│
├── platforms/esp32/                  # Prototype 1 and Remus Blade
│   ├── include/remus/profiles/       # one explicit profile per hardware target
│   ├── include/remus/drivers/
│   └── src/
│
├── platforms/raspberrypi/            # Prototype 2
│   ├── include/remus/profiles/Prototype2.hpp
│   ├── include/remus/drivers/
│   ├── src/
│   ├── deploy/remus-proto2.service
│   └── scripts/
│
├── platformio.ini                    # ESP32 only
└── tests/core_smoke.cpp
```

## Prototype 1 — ESP32-C3

Working pin map currently encoded in `platforms/esp32/include/remus/profiles/Prototype1.hpp`:

| Component | Model | Module pin | ESP32-C3 GPIO | Interface | Function |
|---|---|---|---:|---|---|
| IMU | MPU-6050 / GY-521 | SDA | GPIO 5 | I2C | Data |
| IMU | MPU-6050 / GY-521 | SCL | GPIO 6 | I2C | Clock |
| GPS | u-blox NEO-6M / GY-NEO6MV2 | TX | GPIO 0 | UART | GPS TX -> ESP32 RX |
| GPS | u-blox NEO-6M / GY-NEO6MV2 | RX | GPIO 1 | UART | ESP32 TX -> GPS RX |
| MicroSD | MicroSD Card Module | MISO | GPIO 8 | SPI | SD -> ESP32 |
| MicroSD | MicroSD Card Module | MOSI | GPIO 10 | SPI | ESP32 -> SD |
| MicroSD | MicroSD Card Module | SCK | GPIO 20 | SPI | Clock |
| MicroSD | MicroSD Card Module | CS | GPIO 21 | SPI | Chip Select |
| TFT | GMT024-10 V2.1 | SCL / SCK | GPIO 4 | Software SPI | Clock |
| TFT | GMT024-10 V2.1 | SDA / MOSI | GPIO 2 | Software SPI | ESP32 -> display |
| TFT | GMT024-10 V2.1 | DC | GPIO 7 | GPIO | Data / Command |
| TFT | GMT024-10 V2.1 | RST | GPIO 3 | GPIO | Display reset |
| TFT | GMT024-10 V2.1 | CS | GPIO 9 | GPIO | Chip Select |

All devices must share GND. The TFT power rail is 3.3 V. Its data pins are on a
dedicated software-SPI bus, separate from the MicroSD hardware-SPI bus.

The current display profile selects the ST7789 controller at 240x320. Supplier
documentation for this exact board marking identifies ST7789, but the IC must
still be confirmed on the assembled unit. If the physical batch differs, change
`DisplayModel` and its driver rather than changing GPIOs in application code.
This module is initialized in `SPI_MODE3`. At boot it briefly shows red, green
and blue bands as a wiring/controller self-test before drawing the dashboard.

Build:

```bash
pio run -e remus-proto1
```

Upload:

```bash
pio run -e remus-proto1 -t upload
```

Instalação seguida do monitor serial:

```bash
./scripts/install_remus_computer.sh
```

Monitor:

```bash
pio device monitor -b 115200
```

**Do not edit GPIOs in `main.cpp` or `RemusApp.cpp`.** Change the profile only.

## Remus Blade — ESP32-C3 development profile

The Blade is a separate firmware target in the same repository. It shares the
versioned binary protocol and hardware abstractions with the full REMUS device,
but it does not compile GPS, MicroSD, TFT or standalone recording features.

| Component | Module pin | ESP32-C3 GPIO | Interface |
|---|---|---:|---|
| MPU-6050 / GY-521 | SCL | GPIO 5 | I2C clock |
| MPU-6050 / GY-521 | SDA | GPIO 6 | I2C data |

Build the full device and Blade independently:

```bash
pio run -e remus-proto1
pio run -e remus-blade-dev
```

Upload only the Blade image:

```bash
pio run -e remus-blade-dev -t upload
```

Instalação seguida do monitor serial:

```bash
./scripts/install_remus_blade.sh
```

Os scripts selecionam automaticamente a porta quando há somente uma. Se mais
de uma placa estiver conectada, escolha explicitamente para impedir upload no
dispositivo errado:

```bash
./scripts/install_remus_blade.sh --port /dev/cu.usbmodem101
```

Use `--no-monitor` para apenas instalar ou `--monitor-only` para abrir somente
o monitor serial. Pressione `Ctrl+C` para encerrar o monitor.

No PlatformIO IDE, os mesmos fluxos aparecem em **Project Tasks**:

- `remus-proto1 > Custom > Install & Monitor` instala o Remus Computer;
- `remus-blade-dev > Custom > Install & Monitor` instala o Remus Blade.

A tarefa compila, faz upload e abre o monitor a 115200 baud. As tarefas nativas
`Upload` e `Monitor` continuam disponíveis separadamente. Quando houver várias
portas, configure `upload_port` no ambiente correspondente ou use um dos scripts
com `--port`.

The development firmware samples raw MPU-6050 acceleration and gyroscope data
at 200 Hz into a two-second queue. A lower-priority BLE task sends versioned,
CRC-protected batches and fragments them when the negotiated MTU is smaller
than 185 bytes. Sampling never calls the BLE stack. Stable device serials are
derived once from the ESP32 identity and persisted in NVS; the app can assign
human aliases such as `Blade 01` without changing that identity.

This beta intentionally makes no battery, onboard-storage, GNSS or
store-and-forward claim. Those capabilities require matching hardware and
evidence before they may be advertised.

The TFT dashboard uses two full-width stacked metrics: live SPM above and
pace/500 m below. Their values use nearly all available screen area for maximum
readability while paddling. The bottom line combines workout state, IMU,
MicroSD, BLE, compact GPS satellite state and written-record count. GPS uses a
number for satellites in a qualified fix, `~N` while searching and `--` without
signal. With the UBX profile active, horizontal error comes from the receiver;
only the NMEA fallback uses an explicit HDOP-derived estimate.
The dashboard uses partial 1 Hz updates so display rendering does not require a
full-screen redraw during the 200 Hz acquisition path.

### GNSS behavior and bench diagnosis

The ESP32 profile configures the declared u-blox NEO-6M through UBX at 9600
baud. Coherent NAV-POSLLH, NAV-VELNED and NAV-STATUS solutions run at 5 Hz;
position and velocity are published only when their GPS time-of-week matches.
GGA/RMC remain at 1 Hz and GSV at low cadence for UTC and diagnostics. Boot
should report that the receiver acknowledged UBX; if it does not,
the firmware keeps parsing default NMEA but the module marking, wiring and
actual receiver family must be checked. Do not substitute PMTK/PCAS commands:
those target other receiver families.

“Satellites in view” means the antenna can see RF signals, not that navigation
is locked. In UBX mode the firmware uses NAV-STATUS fix validity and native
accuracy; the NMEA fallback retains the stricter GGA/GSA checks and rejects
stale HDOP.

For an antenna check, test outdoors with an unobstructed sky and allow 5–15
minutes for a true cold start. Repeatedly seeing satellites but no fix warrants
inspection of the antenna orientation, active-antenna supply if applicable,
connector/solder joints, module markings and separation from the TFT, ESP32 and
BLE wiring. Use the GGA/GSA state and GSV signal values together; satellite
count alone cannot validate the antenna.

The legacy app `AID,<lat>,<lon>` input is intentionally ignored: a position
without qualified time, uncertainty and ephemeris data is not valid u-blox
assistance. During BLE file transfer the firmware continues polling the GNSS
UART so its receive buffer does not overflow and make GPS appear to disappear.

## Prototype 2 — Raspberry Pi (Blade / Paddle IMU & Full Prototype)

Hardware profiles supported:

- **Blade / Paddle mode (No GPS)**: MPU-6050 at 200 Hz, autonomous recording to Linux filesystem on boot (`/var/lib/remus/sessions`), live SPM estimation.
- **Full Prototype**: MPU-6050 + NEO-6M GPS + SSD1306 OLED display.

Default hardware settings:
- I2C bus: `/dev/i2c-1` (configured at 400 kHz Fast-Mode)
- MPU-6050: `0x68`, 200 Hz, +/-8 g, +/-500 dps, DLPF 3
- SSD1306 (optional): `0x3C`, 128x64
- NEO-6M (optional): `/dev/serial0`, 9600 baud (disabled in blade mode via `--no-gps`)
- Session storage: `/var/lib/remus/sessions/` (RBP1 binary files)

### Deploy to Raspberry Pi (from development computer)

From your Mac/PC, deploy code directly to your Raspberry Pi (`gramulho-pi.local`):

```bash
cd platforms/raspberrypi
./scripts/deploy_to_pi.sh pi@gramulho-pi.local
```

This will:
1. Synchronize source files via `rsync`.
2. Compile `remus-proto2` with CMake on the Raspberry Pi.
3. Install the systemd service configured for blade capture (`--auto-start --no-gps`).
4. Start recording automatically on boot.

### Monitor live sensor logs

```bash
./scripts/monitor_pi.sh pi@gramulho-pi.local
```

### Download recorded sessions

After a workout, retrieve all `.bin` session files to your machine:

```bash
./scripts/fetch_sessions.sh pi@gramulho-pi.local ./sessions_downloaded
```

To automatically delete downloaded sessions from the Pi after transfer:
```bash
./scripts/fetch_sessions.sh pi@gramulho-pi.local ./sessions_downloaded --delete-remote
```

### Direct on-device commands

If working directly on the Raspberry Pi:

1. **First-time setup** (enables 400 kHz I2C):
   ```bash
   ./scripts/prepare_pi.sh
   sudo reboot
   ```

2. **Test sensor wiring**:
   ```bash
   ./scripts/test_sensor.sh
   ```

3. **Install / update auto-start service**:
   ```bash
   sudo ./scripts/install_service.sh
   ```

4. **Service control**:
   ```bash
   sudo systemctl status remus-proto2
   journalctl -u remus-proto2 -f
   ```

The service configuration is located at `/etc/default/remus-sensor`.
Available arguments for `remus-proto2`:
```text
--auto-start           start recording immediately on launch
--no-gps               disable GPS (blade/paddle IMU-only mode)
--no-oled              disable SSD1306 display
--session-dir PATH     session output directory (default: /var/lib/remus/sessions)
--i2c DEVICE           I2C bus (default: /dev/i2c-1)
--gps DEVICE           GPS serial port (default: /dev/serial0, or 'none')
```

## Versioned RBP sessions

Both targets understand the original RBP1 layout:

- `0x01`: IMU, 17 bytes
- `0x02`: GPS, 19 bytes
- `0x03`: accepted Live SPM result, 7 bytes

The reserved header padding is used to tag the producer without changing the RBP1 size. Existing parsers that ignore padding remain compatible.

ESP32 firmware 0.3.x writes RBP2. The header remains 32 bytes, IMU (`0x01`) and
SPM (`0x03`) stay byte-compatible, and GNSS uses `0x04` (41 bytes) with GPS
time-of-week, Doppler ground speed, course, native horizontal/speed/course
accuracy and fix status. Every coherent GNSS epoch is retained, including epochs
without a qualified fix (`flags = 0`), and its local timestamp is assigned when
the complete receiver solution arrives. This preserves lock dropouts and makes
iTOW cadence gaps measurable. Readers select the record layout from
magic/version; they must not infer it from file size.

## Live SPM

`LiveSpmEstimator` remains platform-independent. The 200 Hz acquisition path sends an 8-sample average to the estimator at approximately 25 Hz. Heavy autocorrelation does not run in the 200 Hz acquisition path on either target.

## BLE status

Prototype 1 keeps the existing ESP32 BLE protocol and file-transfer implementation.
`FILE_END` includes the byte count and a CRC32 of the exact RBP artifact. New
clients validate unique byte coverage, RBP magic and CRC; the optional CRC field
keeps older firmware and clients interoperable during migration.

Prototype 2 currently implements capture, GPS, OLED, split, Linux storage and shared Live SPM. A BlueZ GATT transport has **not** been added yet. This separation is intentional: BlueZ belongs in `platforms/raspberrypi/`, not in the shared core. Until that driver is added, the Pi target is controlled by auto-start/systemd or its console commands.
