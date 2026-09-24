# REMUS MicroSD downloader for PlatformIO
# Adds:
#   Custom -> List SD Files
#   Custom -> Download SD Sessions
#   Custom -> Download Latest SD Session
#
# Requires the matching REMUS firmware USB recovery protocol v7 (ACK/CRC text chunks).

Import("env")

import os
import struct
import time
import zlib
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    raise RuntimeError(
        "pyserial is required. PlatformIO normally includes it in its Python environment."
    ) from exc

BAUD = 115200
LINE_TIMEOUT_SECONDS = 6.0
BINARY_READ_TIMEOUT_SECONDS = 8.0
FRAME_HEADER_SIZE = 14
FRAME_MAGIC = b"RCH1"
END_MAGIC = b"REND"


def _port_candidates(preferred=None):
    ports = list(list_ports.comports())
    devices = [p.device for p in ports if "Bluetooth" not in p.device and "BTH" not in p.device]

    ordered = []
    if preferred and preferred in devices:
        ordered.append(preferred)

    # ESP32-C3 native USB-Serial/JTAG normally reports Espressif VID 0x303A.
    for p in ports:
        if getattr(p, "vid", None) == 0x303A and p.device not in ordered:
            ordered.append(p.device)

    # macOS fallback for native USB CDC / USB-Serial-JTAG.
    for p in ports:
        if "usbmodem" in p.device.lower() and p.device not in ordered:
            ordered.append(p.device)

    # Finally keep PlatformIO's configured/auto-detected port if it is present.
    configured = str(env.get("UPLOAD_PORT", "") or "")
    if configured and configured.lower() != "none" and configured in devices and configured not in ordered:
        ordered.append(configured)

    return ordered


def _initial_port():
    configured = str(env.get("UPLOAD_PORT", "") or "")
    if not configured or configured.lower() == "none":
        try:
            env.AutodetectUploadPort()
            configured = str(env.get("UPLOAD_PORT", "") or "")
        except Exception:
            configured = ""

    candidates = _port_candidates(configured if configured else None)
    if candidates:
        return candidates[0]
    
    if configured and configured.lower() != "none" and "Bluetooth" not in configured and "BTH" not in configured:
        return configured
        
    raise RuntimeError(
        "Could not find the ESP32 USB serial port. "
        "Please ensure the Remus PC is plugged into the computer via a data-capable USB cable."
    )


def _open_serial(preferred_port=None, overall_timeout=15.0):
    deadline = time.monotonic() + overall_timeout
    last_error = None
    preferred = preferred_port or _initial_port()

    while time.monotonic() < deadline:
        candidates = _port_candidates(preferred)
        if not candidates:
            time.sleep(0.25)
            continue

        for port in candidates:
            ser = None
            try:
                print(f"[REMUS SD] Opening {port} (DTR=off, RTS=off)")

                # The ESP32-C3 built-in USB Serial/JTAG CDC port does not need the
                # UART-style DTR/RTS auto-reset dance. Open it as a normal CDC port.
                # IMPORTANT for ESP32-C3 native USB-Serial/JTAG:
                # pySerial defaults DTR/RTS to asserted when Serial(port=...)
                # opens the device. The C3 USB-JTAG bridge uses these modem
                # control lines for reset/download-mode control, so opening
                # with the defaults can make /dev/cu.usbmodem* disappear.
                # Build the Serial object CLOSED, set both lines inactive,
                # and only then open the port.
                ser = serial.Serial(
                    port=None,
                    baudrate=BAUD,
                    timeout=0.25,
                    write_timeout=5,
                    rtscts=False,
                    dsrdtr=False,
                )
                ser.dtr = False
                ser.rts = False
                ser.port = port
                ser.open()

                # Do not pulse DTR/RTS and do not force an input-buffer ioctl
                # immediately after opening. Give HW CDC a moment to settle.
                time.sleep(0.20)
                return ser, port
            except (serial.SerialException, OSError) as exc:
                last_error = exc
                if ser is not None:
                    try:
                        ser.close()
                    except Exception:
                        pass
                time.sleep(0.35)

        time.sleep(0.25)

    raise RuntimeError(
        "ESP32-C3 serial port did not stay connected long enough to start the REMUS protocol. "
        f"Last error: {last_error}"
    )


def _write_command(ser, command):
    ser.write((command + "\n").encode("utf-8"))
    ser.flush()


def _read_complete_line(ser, timeout=LINE_TIMEOUT_SECONDS, max_bytes=4096):
    """Read exactly one newline-terminated control line.

    pySerial's readline()/read_until() may return a *partial* line when its
    per-read timeout expires. For this protocol a partial USB_DATA line must
    never be accepted, otherwise the following RCH1 binary frame can be
    mistaken for part of the filename.
    """
    deadline = time.monotonic() + timeout
    data = bytearray()
    while time.monotonic() < deadline:
        b = ser.read(1)
        if not b:
            continue
        if b == b"\n":
            return bytes(data).rstrip(b"\r")
        data += b
        if len(data) > max_bytes:
            raise RuntimeError("REMUS USB control line exceeded maximum length.")
    raise RuntimeError("Timed out waiting for a complete REMUS USB control line.")


def _readline_until(ser, accepted_prefixes, timeout=LINE_TIMEOUT_SECONDS):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        remaining = max(0.05, deadline - time.monotonic())
        raw = _read_complete_line(ser, timeout=remaining)
        line = raw.decode("utf-8", errors="replace").strip()
        if not line:
            continue

        # Firmware may have emitted ordinary telemetry just before entering
        # USB recovery mode. Ignore it until a protocol line is found.
        for prefix in accepted_prefixes:
            if line.startswith(prefix):
                return line

    raise RuntimeError(
        "Timed out waiting for a REMUS USB response. "
        "Close PlatformIO Serial Monitor and verify the updated firmware is running."
    )


def _list_files(ser):
    # Opening some serial adapters can reset the board. Retry the handshake
    # so a command sent during setup() is not permanently lost.
    first = None
    last_error = None
    for _attempt in range(3):
        ser.reset_input_buffer()
        _write_command(ser, "USB_LIST")
        try:
            first = _readline_until(
                ser, ("USB_LIST_BEGIN", "USB_ERR\t"), timeout=3.0
            )
            break
        except RuntimeError as exc:
            last_error = exc
            time.sleep(0.5)

    if first is None:
        raise RuntimeError(
            "REMUS did not answer USB_LIST after retries. "
            "Close Serial Monitor and make sure the USB-recovery firmware is running."
        ) from last_error

    if first.startswith("USB_ERR\t"):
        raise RuntimeError(first)

    files = []
    deadline = time.monotonic() + LINE_TIMEOUT_SECONDS

    while time.monotonic() < deadline:
        remaining = max(0.05, deadline - time.monotonic())
        raw = _read_complete_line(ser, timeout=remaining)
        line = raw.decode("utf-8", errors="replace").strip()
        if not line:
            continue

        if line == "USB_LIST_END":
            return files

        if line.startswith("USB_ERR\t"):
            raise RuntimeError(line)

        if line.startswith("USB_FILE\t"):
            parts = line.split("\t", 2)
            if len(parts) != 3:
                continue
            try:
                size = int(parts[1])
            except ValueError:
                continue
            files.append({"path": parts[2], "size": size})
            # Keep extending while valid protocol traffic arrives.
            deadline = time.monotonic() + LINE_TIMEOUT_SECONDS

    raise RuntimeError("Timed out while receiving the SD file list.")


def _list_files_resilient():
    last_error = None
    preferred = None

    for attempt in range(1, 7):
        ser = None
        try:
            ser, preferred = _open_serial(preferred_port=preferred, overall_timeout=8.0)
            files = _list_files(ser)
            return files
        except (serial.SerialException, OSError, RuntimeError) as exc:
            last_error = exc
            print(f"[REMUS SD] USB handshake interrupted ({attempt}/6): {exc}")
        finally:
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass

        # The ESP32-C3 USB-Serial/JTAG device can disappear briefly during reset.
        time.sleep(0.75)

    raise RuntimeError(
        "Could not complete USB_LIST after reconnecting to the ESP32-C3. "
        "Close Serial Monitor and verify the recovery firmware is running. "
        f"Last error: {last_error}"
    )


def _read_exact(ser, count, timeout=BINARY_READ_TIMEOUT_SECONDS):
    data = bytearray()
    deadline = time.monotonic() + timeout

    while len(data) < count:
        chunk = ser.read(count - len(data))
        if chunk:
            data.extend(chunk)
            deadline = time.monotonic() + timeout
            continue
        if time.monotonic() >= deadline:
            raise RuntimeError(
                f"USB transfer stalled while waiting for {count - len(data)} byte(s)."
            )

    return bytes(data)


def _safe_local_name(remote_path):
    name = Path(remote_path).name
    if not name or name in (".", ".."):
        raise RuntimeError(f"Invalid remote filename: {remote_path!r}")
    return name


def _send_ack(ser, next_offset):
    _write_command(ser, f"USB_ACK\t{next_offset}")


def _send_nack(ser, offset):
    _write_command(ser, f"USB_NACK\t{offset}")


def _download_one_connected(ser, remote, output_dir):
    remote_path = remote["path"]
    remote_size = int(remote["size"])
    local_name = _safe_local_name(remote_path)
    output_path = output_dir / local_name
    part_path = output_dir / (local_name + ".part")

    if output_path.exists() and output_path.stat().st_size == remote_size:
        print(f"[REMUS SD] Already complete: {local_name} ({remote_size:,} bytes)")
        return output_path

    if part_path.exists():
        resume_offset = part_path.stat().st_size
        if resume_offset > remote_size:
            print(f"[REMUS SD] Discarding oversized partial file: {part_path.name}")
            part_path.unlink()
            resume_offset = 0
    else:
        resume_offset = 0

    if resume_offset:
        print(
            f"[REMUS SD] Resuming {local_name} at "
            f"{resume_offset:,}/{remote_size:,} bytes"
        )
    else:
        print(f"[REMUS SD] Downloading {local_name} ({remote_size:,} bytes)")

    ser.reset_input_buffer()
    _write_command(ser, f"USB_GET\t{remote_path}\t{resume_offset}")

    response = _readline_until(ser, ("USB_DATA\t", "USB_ERR\t"), timeout=12.0)
    if response.startswith("USB_ERR\t"):
        raise RuntimeError(response)

    parts = response.split("\t", 3)
    if len(parts) != 4:
        raise RuntimeError(f"Malformed USB_DATA response: {response!r}")

    declared_size = int(parts[1])
    declared_offset = int(parts[2])
    declared_path = parts[3]

    if declared_size != remote_size:
        raise RuntimeError(
            f"Remote file size changed: listing={remote_size}, transfer={declared_size}"
        )
    if declared_offset != resume_offset:
        raise RuntimeError(
            f"Resume offset mismatch: requested={resume_offset}, device={declared_offset}"
        )
    if declared_path != remote_path:
        raise RuntimeError(
            f"Remote path mismatch: requested={remote_path!r}, device={declared_path!r}"
        )

    output_dir.mkdir(parents=True, exist_ok=True)

    mode = "ab" if resume_offset else "wb"
    current = resume_offset
    next_report = current + 1024 * 1024
    bad_chunk_retries = 0

    with open(part_path, mode) as out:
        while True:
            raw = _read_complete_line(ser, timeout=15.0, max_bytes=2048)
            line = raw.decode("ascii", errors="replace").strip()

            if line.startswith("USB_ERR\t"):
                raise RuntimeError(line)

            if line.startswith("USB_END\t"):
                end_parts = line.split("\t", 1)
                if len(end_parts) != 2:
                    raise RuntimeError(f"Malformed USB_END response: {line!r}")
                ended_size = int(end_parts[1])
                if ended_size != remote_size:
                    raise RuntimeError(
                        f"Device ended with size {ended_size}, expected {remote_size}."
                    )
                if current != remote_size:
                    raise RuntimeError(
                        f"Local partial size is {current}, expected {remote_size}."
                    )
                _write_command(ser, "USB_ACK_END")
                break

            if not line.startswith("USB_CHUNK\t"):
                # During recovery there should be no normal telemetry. If a line
                # is malformed/corrupted, request the current block again rather
                # than consuming more bytes and losing framing.
                bad_chunk_retries += 1
                _send_nack(ser, current)
                if bad_chunk_retries > 8:
                    raise RuntimeError(
                        f"Too many malformed USB chunk lines at offset {current}: {line[:120]!r}"
                    )
                continue

            parts = line.split("\t", 4)
            if len(parts) != 5:
                bad_chunk_retries += 1
                _send_nack(ser, current)
                if bad_chunk_retries > 8:
                    raise RuntimeError(f"Malformed USB_CHUNK at offset {current}.")
                continue

            try:
                chunk_offset = int(parts[1])
                chunk_len = int(parts[2])
                expected_crc = int(parts[3], 16)
                payload = bytes.fromhex(parts[4])
            except (ValueError, TypeError) as exc:
                bad_chunk_retries += 1
                _send_nack(ser, current)
                if bad_chunk_retries > 8:
                    raise RuntimeError(
                        f"Could not decode USB_CHUNK at offset {current}: {exc}"
                    )
                continue

            actual_crc = zlib.crc32(payload) & 0xFFFFFFFF

            if (
                chunk_offset != current
                or chunk_len != len(payload)
                or actual_crc != expected_crc
            ):
                bad_chunk_retries += 1
                _send_nack(ser, current)
                if bad_chunk_retries > 8:
                    raise RuntimeError(
                        f"Repeated invalid chunk at offset {current}: "
                        f"device_offset={chunk_offset}, len={chunk_len}/{len(payload)}, "
                        f"crc=device 0x{expected_crc:08X} host 0x{actual_crc:08X}"
                    )
                continue

            # Only validated bytes reach the .part file.
            out.write(payload)
            out.flush()
            current += len(payload)
            bad_chunk_retries = 0

            _send_ack(ser, current)

            if current >= next_report or current == remote_size:
                pct = 100.0 if remote_size == 0 else (current * 100.0 / remote_size)
                print(
                    f"[REMUS SD] {local_name}: "
                    f"{current:,}/{remote_size:,} bytes ({pct:.1f}%)"
                )
                next_report = current + 1024 * 1024

    os.replace(part_path, output_path)
    print(f"[REMUS SD] Complete: {output_path}")
    return output_path

def _download_one_resilient(remote, output_dir):
    # Each retry resumes from the validated .part length.  A USB reset therefore
    # costs at most the current 4 KiB frame, not the whole session file.
    last_error = None
    preferred = None

    for attempt in range(1, 9):
        ser = None
        try:
            ser, preferred = _open_serial(preferred_port=preferred, overall_timeout=10.0)
            return _download_one_connected(ser, remote, output_dir)
        except (serial.SerialException, OSError, RuntimeError) as exc:
            last_error = exc
            print(f"[REMUS SD] Transfer interrupted ({attempt}/8): {exc}")
            print("[REMUS SD] Reconnecting; validated .part data will be resumed.")
        finally:
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass
        time.sleep(0.75)

    raise RuntimeError(
        f"Could not finish {remote['path']} after reconnect attempts. "
        f"The .part file was preserved. Last error: {last_error}"
    )


def _download_dir():
    configured = env.GetProjectOption("custom_sd_download_dir", "sd_downloads")
    path = Path(configured)
    if not path.is_absolute():
        path = Path(env["PROJECT_DIR"]) / path
    path.mkdir(parents=True, exist_ok=True)
    return path


def _session_files(files):
    return [f for f in files if f["path"].lower().endswith(".bin") and f["size"] > 0]




def command_sd_probe(target, source, env):
    """Minimal USB protocol probe; does not access or modify SD data."""
    ser = None
    try:
        ser, port = _open_serial(overall_timeout=8.0)
        print(f"[REMUS SD] Port stable: {port}")
        _write_command(ser, "USB_LIST")
        first = _readline_until(ser, ("USB_LIST_BEGIN", "USB_ERR\t"), timeout=4.0)
        if first is None:
            raise RuntimeError(
                "Port stayed open but the REMUS USB recovery protocol did not answer. "
                "This usually means the recovery firmware has not actually been uploaded, "
                "or Serial is not mapped to Hardware CDC/JTAG."
            )
        print(f"[REMUS SD] Protocol response: {first}")
        if first == "USB_LIST_BEGIN":
            # Drain only the textual listing so the next command starts clean.
            while True:
                line = _readline_until(ser, ("USB_FILE\t", "USB_LIST_END", "USB_ERR\t"), timeout=2.0)
                if line is None or line == "USB_LIST_END" or line.startswith("USB_ERR\t"):
                    if line:
                        print(f"[REMUS SD] {line}")
                    break
        print("[REMUS SD] USB recovery protocol is reachable.")
    finally:
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

def command_sd_list(target, source, env):
    files = _list_files_resilient()

    print("")
    print("REMUS MicroSD files")
    print("-------------------")
    if not files:
        print("(no files)")
        return

    for item in files:
        print(f"{item['size']:>12,} bytes  {item['path']}")


def command_sd_download_all(target, source, env):
    output_dir = _download_dir()
    files = _list_files_resilient()
    sessions = _session_files(files)
    if not sessions:
        print("[REMUS SD] No .bin session files found.")
        return

    print(f"[REMUS SD] Found {len(sessions)} session file(s).")
    print(f"[REMUS SD] Destination: {output_dir}")

    for item in sessions:
        _download_one_resilient(item, output_dir)


def command_sd_download_latest(target, source, env):
    output_dir = _download_dir()
    files = _list_files_resilient()
    sessions = _session_files(files)
    if not sessions:
        print("[REMUS SD] No .bin session files found.")
        return

    # Matches the firmware's current "latest" logic: last .bin returned
    # by FAT directory iteration.
    latest = sessions[-1]
    print(f"[REMUS SD] Latest session: {latest['path']}")
    _download_one_resilient(latest, output_dir)



env.AddCustomTarget(
    name="sdprobe",
    dependencies=None,
    actions=[command_sd_probe],
    title="Probe REMUS USB",
    description="Checks the REMUS USB recovery protocol without downloading files",
)

env.AddCustomTarget(
    name="sdlist",
    dependencies=None,
    actions=[command_sd_list],
    title="List SD Files",
    description="Lists files stored on the REMUS external MicroSD over USB Serial",
)

env.AddCustomTarget(
    name="downloadsd",
    dependencies=None,
    actions=[command_sd_download_all],
    title="Download SD Sessions",
    description="Downloads all REMUS .bin sessions from the external MicroSD over USB Serial",
)

env.AddCustomTarget(
    name="downloadlatest",
    dependencies=None,
    actions=[command_sd_download_latest],
    title="Download Latest SD Session",
    description="Downloads/resumes the latest REMUS .bin session from the external MicroSD over USB Serial",
)
