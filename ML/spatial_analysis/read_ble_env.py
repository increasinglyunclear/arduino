# Reads output of Arduino Nano BLE Sense Rev 2
# Requires the following libs

import asyncio
from bleak import BleakScanner, BleakClient
import struct

DEVICE_NAME = "ContextBeacon"
SERVICE_UUID = "180A"
TEMP_CHAR_UUID = "00002A6E-0000-1000-8000-00805f9b34fb"
HUM_CHAR_UUID  = "00002A6F-0000-1000-8000-00805f9b34fb"

def decode_float(data):
    # BLE sends little-endian floats
    return struct.unpack('<f', data)[0]

async def main():
    print("Scanning for BLE devices...")
    devices = await BleakScanner.discover()
    device = None
    for d in devices:
        print(f"Found: {d.name} [{d.address}]")
        if d.name == DEVICE_NAME:
            device = d
            break

    if not device:
        print(f"Device '{DEVICE_NAME}' not found.")
        return

    print(f"Connecting to {device.name} [{device.address}]...")
    async with BleakClient(device.address) as client:
        print("Connected!")

        # Read temperature
        temp_data = await client.read_gatt_char(TEMP_CHAR_UUID)
        temperature = decode_float(temp_data)
        print(f"Temperature: {temperature:.2f} °C")

        # Read humidity
        hum_data = await client.read_gatt_char(HUM_CHAR_UUID)
        humidity = decode_float(hum_data)
        print(f"Humidity: {humidity:.2f} %")

if __name__ == "__main__":
    asyncio.run(main()) 
