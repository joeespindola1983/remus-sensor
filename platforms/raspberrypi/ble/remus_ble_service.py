#!/usr/bin/env python3
"""
REMUS Blade Sensor — BlueZ BLE GATT Server Daemon for Raspberry Pi.
Exposes the exact same BLE contract as ESP32:
- Service UUID:        4fafc201-1fb5-459e-8fcc-c5c9c331914b
- Characteristic UUID: beb5483e-36e1-4688-b7f5-ea07361b26a8
- 1 Hz CSV Snapshot notifications with live SPM and IMU
- START / STOP / GET command reception from mobile app
"""

import dbus
import dbus.exceptions
import dbus.mainloop.glib
import dbus.service
from gi.repository import GLib

import glob
import os
import signal
import socket
import struct
import sys
import threading
import time

BLUEZ_SERVICE_NAME = 'org.bluez'
GATT_MANAGER_IFACE = 'org.bluez.GattManager1'
DBUS_OM_IFACE = 'org.freedesktop.DBus.ObjectManager'
DBUS_PROP_IFACE = 'org.freedesktop.DBus.Properties'
GATT_SERVICE_IFACE = 'org.bluez.GattService1'
GATT_CHRC_IFACE = 'org.bluez.GattCharacteristic1'
GATT_DESC_IFACE = 'org.bluez.GattDescriptor1'
LE_ADVERTISING_MANAGER_IFACE = 'org.bluez.LEAdvertisingManager1'
LE_ADVERTISEMENT_IFACE = 'org.bluez.LEAdvertisement1'

REMUS_SERVICE_UUID = '4fafc201-1fb5-459e-8fcc-c5c9c331914b'
REMUS_CHAR_UUID = 'beb5483e-36e1-4688-b7f5-ea07361b26a8'

# ---------------------------------------------------------------------------
# BlueZ D-Bus Helpers
# ---------------------------------------------------------------------------

class InvalidArgsException(dbus.exceptions.DBusException):
    _dbus_error_name = 'org.freedesktop.DBus.Error.InvalidArgs'

class NotSupportedException(dbus.exceptions.DBusException):
    _dbus_error_name = 'org.bluez.Error.NotSupported'

class Application(dbus.service.Object):
    def __init__(self, bus):
        self.path = '/'
        self.services = []
        super().__init__(bus, self.path)

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def add_service(self, service):
        self.services.append(service)

    @dbus.service.method(DBUS_OM_IFACE, out_signature='a{oa{sa{sv}}}')
    def GetManagedObjects(self):
        response = {}
        for service in self.services:
            response[service.get_path()] = service.get_properties()
            for chrc in service.get_characteristics():
                response[chrc.get_path()] = chrc.get_properties()
                for desc in chrc.get_descriptors():
                    response[desc.get_path()] = desc.get_properties()
        return response

class Service(dbus.service.Object):
    PATH_BASE = '/org/bluez/remus/service'

    def __init__(self, bus, index, uuid, primary):
        self.path = f'{self.PATH_BASE}{index}'
        self.bus = bus
        self.uuid = uuid
        self.primary = primary
        self.characteristics = []
        super().__init__(bus, self.path)

    def get_properties(self):
        return {
            GATT_SERVICE_IFACE: {
                'UUID': self.uuid,
                'Primary': self.primary,
                'Characteristics': dbus.Array(
                    [chrc.get_path() for chrc in self.characteristics],
                    signature='o'
                )
            }
        }

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def add_characteristic(self, characteristic):
        self.characteristics.append(characteristic)

    def get_characteristics(self):
        return self.characteristics

class Characteristic(dbus.service.Object):
    def __init__(self, bus, index, uuid, flags, service):
        self.path = f'{service.path}/char{index}'
        self.bus = bus
        self.uuid = uuid
        self.service = service
        self.flags = flags
        self.descriptors = []
        self.value = []
        self.notifying = False
        super().__init__(bus, self.path)

    def get_properties(self):
        return {
            GATT_CHRC_IFACE: {
                'Service': self.service.get_path(),
                'UUID': self.uuid,
                'Flags': self.flags,
                'Descriptors': dbus.Array(
                    [desc.get_path() for desc in self.descriptors],
                    signature='o'
                ),
                'Value': dbus.Array(self.value, signature='y')
            }
        }

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def add_descriptor(self, descriptor):
        self.descriptors.append(descriptor)

    def get_descriptors(self):
        return self.descriptors

    @dbus.service.method(DBUS_PROP_IFACE, in_signature='s', out_signature='a{sv}')
    def GetAll(self, interface):
        if interface != GATT_CHRC_IFACE:
            raise InvalidArgsException()
        return self.get_properties()[GATT_CHRC_IFACE]

    @dbus.service.method(GATT_CHRC_IFACE, in_signature='a{sv}', out_signature='ay')
    def ReadValue(self, options):
        return self.value

    @dbus.service.method(GATT_CHRC_IFACE, in_signature='aya{sv}')
    def WriteValue(self, value, options):
        pass

    @dbus.service.method(GATT_CHRC_IFACE)
    def StartNotify(self):
        if not self.notifying:
            self.notifying = True
            print("[BLE] Client subscribed to notifications.")

    @dbus.service.method(GATT_CHRC_IFACE)
    def StopNotify(self):
        if self.notifying:
            self.notifying = False
            print("[BLE] Client unsubscribed from notifications.")

    @dbus.service.signal(DBUS_PROP_IFACE, signature='sa{sv}as')
    def PropertiesChanged(self, interface, changed, invalidated):
        pass

    def send_notification(self, byte_data):
        if not self.notifying:
            return
        self.value = dbus.Array([dbus.Byte(b) for b in byte_data], signature='y')
        self.PropertiesChanged(
            GATT_CHRC_IFACE,
            {'Value': self.value},
            []
        )

class ClientCharacteristicConfig(dbus.service.Object):
    def __init__(self, bus, index, characteristic):
        self.path = f'{characteristic.path}/desc{index}'
        self.bus = bus
        self.uuid = '2902'
        self.flags = ['read', 'write']
        self.chrc = characteristic
        self.value = [0, 0]
        super().__init__(bus, self.path)

    def get_properties(self):
        return {
            GATT_DESC_IFACE: {
                'Characteristic': self.chrc.get_path(),
                'UUID': self.uuid,
                'Flags': self.flags,
                'Value': dbus.Array(self.value, signature='y')
            }
        }

    def get_path(self):
        return dbus.ObjectPath(self.path)

    @dbus.service.method(DBUS_PROP_IFACE, in_signature='s', out_signature='a{sv}')
    def GetAll(self, interface):
        if interface != GATT_DESC_IFACE:
            raise InvalidArgsException()
        return self.get_properties()[GATT_DESC_IFACE]

    @dbus.service.method(GATT_DESC_IFACE, in_signature='a{sv}', out_signature='ay')
    def ReadValue(self, options):
        return self.value

    @dbus.service.method(GATT_DESC_IFACE, in_signature='aya{sv}')
    def WriteValue(self, value, options):
        self.value = value
        if len(value) >= 1 and value[0] == 1:
            self.chrc.StartNotify()
        else:
            self.chrc.StopNotify()

class Advertisement(dbus.service.Object):
    PATH_BASE = '/org/bluez/remus/advertisement'

    def __init__(self, bus, index, local_name):
        self.path = f'{self.PATH_BASE}{index}'
        self.bus = bus
        self.ad_type = 'peripheral'
        self.service_uuids = [REMUS_SERVICE_UUID]
        self.local_name = local_name
        self.include_tx_power = True
        super().__init__(bus, self.path)

    def get_properties(self):
        properties = {
            'Type': self.ad_type,
            'ServiceUUIDs': dbus.Array(self.service_uuids, signature='s'),
            'LocalName': dbus.String(self.local_name),
            'Includes': dbus.Array(['tx-power'], signature='s')
        }
        return {LE_ADVERTISEMENT_IFACE: properties}

    def get_path(self):
        return dbus.ObjectPath(self.path)

    @dbus.service.method(DBUS_PROP_IFACE, in_signature='s', out_signature='a{sv}')
    def GetAll(self, interface):
        if interface != LE_ADVERTISEMENT_IFACE:
            raise InvalidArgsException()
        return self.get_properties()[LE_ADVERTISEMENT_IFACE]

    @dbus.service.method(LE_ADVERTISEMENT_IFACE, in_signature='', out_signature='')
    def Release(self):
        print(f'[BLE] Advertisement {self.path} released')

# ---------------------------------------------------------------------------
# Remus Blade Characteristic & IPC Client
# ---------------------------------------------------------------------------

class RemusBladeCharacteristic(Characteristic):
    def __init__(self, bus, index, service, ipc_client):
        super().__init__(
            bus, index, REMUS_CHAR_UUID,
            ['read', 'write', 'notify'],
            service
        )
        self.ipc_client = ipc_client
        self.add_descriptor(ClientCharacteristicConfig(bus, 1, self))

    def WriteValue(self, value, options):
        try:
            cmd = bytes(value).decode('utf-8', errors='ignore').strip()
            print(f"[BLE RX] 📥 Command from app: {cmd}")
            if cmd.startswith("GET"):
                threading.Thread(target=self.handle_get_command, args=(cmd,), daemon=True).start()
            else:
                self.ipc_client.send_command(cmd)
        except Exception as e:
            print(f"[BLE] Error processing command: {e}")

    def handle_get_command(self, cmd):
        """Handle GET command from mobile app to stream latest session file over BLE."""
        session_dirs = ["/var/lib/remus/sessions", "./sessions"]
        files = []
        for d in session_dirs:
            if os.path.exists(d):
                files.extend(glob.glob(os.path.join(d, "*.bin")))
        if not files:
            self.send_notification(b"FILE_ERR:No sessions found\n")
            return

        files.sort(key=os.path.getmtime)
        filepath = files[-1]
        filename = os.path.basename(filepath)
        filesize = os.path.getsize(filepath)

        print(f"[BLE FILE] Streaming {filename} ({filesize} bytes) to phone...")
        start_msg = f"FILE_START:{filename}:{filesize}\n".encode('ascii')
        self.send_notification(start_msg)
        time.sleep(0.05)

        chunk_size = 240
        offset = 0
        with open(filepath, "rb") as f:
            while offset < filesize:
                data = f.read(chunk_size)
                if not data:
                    break
                header = bytearray([0x20]) + struct.pack("<I", offset) + struct.pack("<H", len(data))
                packet = header + data
                self.send_notification(packet)
                offset += len(data)
                time.sleep(0.015)

        time.sleep(0.05)
        end_msg = f"FILE_END:{filesize}\n".encode('ascii')
        self.send_notification(end_msg)
        print(f"[BLE FILE] Transfer complete: {filename}")

class RemusIpcClient:
    def __init__(self, char_callback=None):
        self.char_callback = char_callback
        self.sock = None
        self.running = True

    def set_callback(self, callback):
        self.char_callback = callback

    def send_command(self, cmd):
        if self.sock:
            try:
                self.sock.sendall((cmd + "\n").encode('utf-8'))
                print(f"[IPC] Forwarded command to C++ engine: {cmd}")
                return True
            except Exception as e:
                print(f"[IPC] Failed to send command: {e}")
        return False

    def loop(self):
        socket_paths = ["/run/remus/remus.sock", "/tmp/remus.sock"]
        while self.running:
            active_path = None
            for p in socket_paths:
                if os.path.exists(p):
                    active_path = p
                    break

            if not active_path:
                time.sleep(1)
                continue

            try:
                s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                s.connect(active_path)
                self.sock = s
                print(f"[IPC] Connected to remus-proto2 engine at {active_path}")
                f = s.makefile('rb')
                while self.running:
                    line = f.readline()
                    if not line:
                        break
                    if self.char_callback:
                        self.char_callback(line)
            except Exception as e:
                pass
            finally:
                if self.sock:
                    try:
                        self.sock.close()
                    except Exception:
                        pass
                    self.sock = None
            time.sleep(1)

# ---------------------------------------------------------------------------
# Main Setup
# ---------------------------------------------------------------------------

def get_adapter_path(bus):
    manager = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, '/'), DBUS_OM_IFACE)
    objects = manager.GetManagedObjects()
    for path, interfaces in objects.items():
        if GATT_MANAGER_IFACE in interfaces and LE_ADVERTISING_MANAGER_IFACE in interfaces:
            return path
    return None

def get_device_name():
    # Format: REMUS-P2-XXXX where XXXX is last 4 chars of MAC address or hostname
    try:
        with open('/sys/class/bluetooth/hci0/address', 'r') as f:
            mac = f.read().strip().replace(':', '').upper()
            if len(mac) >= 4:
                return f"REMUS-P2-{mac[-4:]}"
    except Exception:
        pass
    try:
        with open('/sys/class/net/wlan0/address', 'r') as f:
            mac = f.read().strip().replace(':', '').upper()
            if len(mac) >= 4:
                return f"REMUS-P2-{mac[-4:]}"
    except Exception:
        pass
    hostname = socket.gethostname()
    suffix = hostname[-4:].upper() if len(hostname) >= 4 else "0001"
    return f"REMUS-P2-{suffix}"

def main():
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SystemBus()

    adapter_path = get_adapter_path(bus)
    if not adapter_path:
        print("❌ Error: No BlueZ BLE adapter found. Ensure Bluetooth is powered on.")
        sys.exit(1)

    try:
        adapter_props = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, adapter_path), DBUS_PROP_IFACE)
        adapter_props.Set('org.bluez.Adapter1', 'Powered', dbus.Boolean(True))
    except Exception as e:
        print(f"[BLE] Note: Adapter Powered set error: {e}")

    gatt_manager = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, adapter_path), GATT_MANAGER_IFACE)
    ad_manager = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, adapter_path), LE_ADVERTISING_MANAGER_IFACE)

    app = Application(bus)
    service = Service(bus, 0, REMUS_SERVICE_UUID, True)

    ipc_client = RemusIpcClient()
    characteristic = RemusBladeCharacteristic(bus, 0, service, ipc_client)
    ipc_client.set_callback(characteristic.send_notification)
    service.add_characteristic(characteristic)
    app.add_service(service)

    device_name = get_device_name()
    ad = Advertisement(bus, 0, device_name)

    loop = GLib.MainLoop()

    def register_app_cb():
        print(f"✅ REMUS GATT Application registered successfully!")

    def register_app_error_cb(error):
        print(f"❌ Failed to register GATT application: {error}")
        loop.quit()

    def register_ad_cb():
        print(f"📡 REMUS BLE Advertising started as '{device_name}' (Service {REMUS_SERVICE_UUID})")

    def register_ad_error_cb(error):
        print(f"❌ Failed to register advertisement: {error}")
        loop.quit()

    gatt_manager.RegisterApplication(app.get_path(), {},
                                     reply_handler=register_app_cb,
                                     error_handler=register_app_error_cb)

    ad_manager.RegisterAdvertisement(ad.get_path(), {},
                                     reply_handler=register_ad_cb,
                                     error_handler=register_ad_error_cb)

    ipc_thread = threading.Thread(target=ipc_client.loop, daemon=True)
    ipc_thread.start()

    def on_signal(sig, frame):
        print("\nShutting down BLE service...")
        ipc_client.running = False
        loop.quit()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    print("==========================================================")
    print(" REMUS BLE GATT Server Running")
    print(f" Name: {device_name}")
    print(f" Service: {REMUS_SERVICE_UUID}")
    print(f" Char:    {REMUS_CHAR_UUID}")
    print("==========================================================")
    loop.run()

if __name__ == '__main__':
    main()
