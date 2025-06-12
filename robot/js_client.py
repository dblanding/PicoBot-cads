# js_client.py

import aioble
import asyncio
import bluetooth
import struct


# bluetooth credentials
ble_name = "3axis_joystk"
ble_svc_uuid = bluetooth.UUID(0x1812)
ble_characteristic_uuid = bluetooth.UUID(0x2A4D)
ble_scan_length = 5000
ble_interval = 30000
ble_window = 30000

def decode(data):
    return struct.unpack("3i", data)


class JS_Client():
    def __init__(self):
        self.data = None

    async def ble_scan(self):
        print("Scanning for BLE beacon named", ble_name, "...")
        async with aioble.scan(
            ble_scan_length, interval_us=ble_interval,
            window_us=ble_window, active=True) as scanner:
            async for result in scanner:
                if result.name() == ble_name and ble_svc_uuid in result.services():
                    return result.device
        return None

    def get_js_vals(self):
        return self.data

    async def start(self):
        while True:
            device = await self.ble_scan()
            if not device:
                print("BLE beacon not found.")
                continue

            try:
                print("Connecting to", device)
                connection = await device.connect()
            except asyncio.TimeoutError:
                print("Connection timed out.")
                continue

            async with connection:
                try:
                    ble_service = await connection.service(ble_svc_uuid)
                    ble_characteristic = await \
                      ble_service.characteristic(ble_characteristic_uuid)
                except (asyncio.TimeoutError, AttributeError):
                    print("Timeout discovering services/characteristics.")
                    continue

                while True:
                    try:
                        self.data = decode(await ble_characteristic.read())
                        await asyncio.sleep(0.05)
                    except Exception as e:
                        print(f"Error: {e}")
                        continue
