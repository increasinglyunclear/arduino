# Reads and displays readings from all the Arduino Nano BLE Sense Rev 2 sensors
# Kevin Walker Jun 2025

import asyncio
from bleak import BleakScanner, BleakClient
import struct

DEVICE_NAME = "ContextBeacon"
SERVICE_UUID = "180A"
TEMP_CHAR_UUID = "00002A6E-0000-1000-8000-00805f9b34fb"
HUM_CHAR_UUID  = "00002A6F-0000-1000-8000-00805f9b34fb"
PROX_CHAR_UUID = "00002A19-0000-1000-8000-00805f9b34fb"
ACCEL_X_UUID = "00002A77-0000-1000-8000-00805f9b34fb"
ACCEL_Y_UUID = "00002A78-0000-1000-8000-00805f9b34fb"
ACCEL_Z_UUID = "00002A79-0000-1000-8000-00805f9b34fb"
GYRO_X_UUID = "00002A80-0000-1000-8000-00805f9b34fb"
GYRO_Y_UUID = "00002A81-0000-1000-8000-00805f9b34fb"
GYRO_Z_UUID = "00002A82-0000-1000-8000-00805f9b34fb"
MAG_X_UUID = "19B10005-E8F2-537E-4F6C-D104768A1214"
MAG_Y_UUID = "19B10006-E8F2-537E-4F6C-D104768A1214"
MAG_Z_UUID = "19B10007-E8F2-537E-4F6C-D104768A1214"
COLOR_RED_UUID = "19B10001-E8F2-537E-4F6C-D104768A1214"
COLOR_GREEN_UUID = "19B10002-E8F2-537E-4F6C-D104768A1214"
COLOR_BLUE_UUID = "19B10003-E8F2-537E-4F6C-D104768A1214"
COLOR_CLEAR_UUID = "19B10004-E8F2-537E-4F6C-D104768A1214"
MIC_LEVEL_UUID = "00002A9C-0000-1000-8000-00805f9b34fb"
PRESSURE_UUID = "00002A6D-0000-1000-8000-00805f9b34fb"

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

        # Read proximity
        prox_data = await client.read_gatt_char(PROX_CHAR_UUID)
        proximity = decode_float(prox_data)
        print(f"Proximity: {proximity:.2f}")

        # Read IMU (accel and gyro)
        accel_x = decode_float(await client.read_gatt_char(ACCEL_X_UUID))
        accel_y = decode_float(await client.read_gatt_char(ACCEL_Y_UUID))
        accel_z = decode_float(await client.read_gatt_char(ACCEL_Z_UUID))
        gyro_x = decode_float(await client.read_gatt_char(GYRO_X_UUID))
        gyro_y = decode_float(await client.read_gatt_char(GYRO_Y_UUID))
        gyro_z = decode_float(await client.read_gatt_char(GYRO_Z_UUID))
        print(f"Accel: x={accel_x:.3f}, y={accel_y:.3f}, z={accel_z:.3f}")
        print(f"Gyro:  x={gyro_x:.3f}, y={gyro_y:.3f}, z={gyro_z:.3f}")

        # Read magnetometer
        mag_x = decode_float(await client.read_gatt_char(MAG_X_UUID))
        mag_y = decode_float(await client.read_gatt_char(MAG_Y_UUID))
        mag_z = decode_float(await client.read_gatt_char(MAG_Z_UUID))
        print(f"Mag:   x={mag_x:.3f}, y={mag_y:.3f}, z={mag_z:.3f}")

        # Read color
        color_red = decode_float(await client.read_gatt_char(COLOR_RED_UUID))
        color_green = decode_float(await client.read_gatt_char(COLOR_GREEN_UUID))
        color_blue = decode_float(await client.read_gatt_char(COLOR_BLUE_UUID))
        color_clear = decode_float(await client.read_gatt_char(COLOR_CLEAR_UUID))
        print(f"Color: R={color_red:.0f}, G={color_green:.0f}, B={color_blue:.0f}, C={color_clear:.0f}")

        # Read microphone level
        mic_level = decode_float(await client.read_gatt_char(MIC_LEVEL_UUID))
        print(f"Mic RMS: {mic_level:.2f}")

        # Read pressure
        pressure = decode_float(await client.read_gatt_char(PRESSURE_UUID))
        print(f"Pressure (hPa): {pressure:.2f}")

if __name__ == "__main__":
    asyncio.run(main()) 
