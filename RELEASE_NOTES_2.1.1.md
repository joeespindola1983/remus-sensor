# Remus Sensor 2.1.1

Release date: 2026-09-29

- Exposes the versioned Clock Sync GATT characteristic on the Remus Computer,
  not only on Remus Blade firmware.
- Returns the Computer's monotonic receive/send timestamps and `computerBootId`
  so the recorder can build a bounded phone-to-Computer clock mapping.
- Keeps the phone-to-Computer mapping independent from the Computer-to-Blade
  mappings already maintained for port and starboard sources.

Firmware and shared library version: 2.1.1.
