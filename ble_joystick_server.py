# 3axis_joystk_ble_server.py (aka main.py)

import struct
import asyncio
import aioble
import bluetooth
from machine import ADC, Pin
import time

# Set up onboard LED
led = Pin("LED", Pin.OUT, value=0)

# Set up joystick on ADC pins
adc_x = ADC(Pin(26))
adc_y = ADC(Pin(27))
adc_z = ADC(Pin(28))

# BLE values
ble_name = "3axis_joystk"
ble_svc_uuid = bluetooth.UUID(0x1812)
ble_characteristic_uuid = bluetooth.UUID(0x2A4D)
ble_appearance = 0x03C3
ble_advertising_interval = 2000
ble_service = aioble.Service(ble_svc_uuid)
ble_characteristic = aioble.Characteristic(
    ble_service,
    ble_characteristic_uuid,
    read=True,
    notify=True)
aioble.register_services(ble_service)

def encode(x, y, z):
    return struct.pack("3i", x, y, z)

async def ble_task():
    while True:
        async with await aioble.advertise(
            ble_advertising_interval,
            name=ble_name,
            services=[ble_svc_uuid],
            appearance=ble_appearance) as connection:
            print("Connection from", connection.device)
            await connection.disconnected()

async def joystk_task():
    while True:
        # get joystick axis values
        js_x = adc_x.read_u16()
        js_y = adc_y.read_u16()
        js_z = adc_z.read_u16()

        # convert to ints: -127 < value < 127
        x = round(js_x / 256) - 130
        y = round(js_y / 256) - 132
        z = round(js_z / 256) - 130

        ble_characteristic.write(encode(x, y, z))
        led.toggle()
        await asyncio.sleep_ms(100)

async def main():
    task1 = asyncio.create_task(ble_task())
    task2 = asyncio.create_task(joystk_task())
    await asyncio.gather(task1, task2)

print("Launching BLE joystick server...")
asyncio.run(main())