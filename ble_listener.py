import asyncio
from bleak import BleakScanner, BleakClient

# UUIDs for Cycling Power Service and Measurement Characteristic
CPS_UUID = "00001818-0000-1000-8000-00805f9b34fb"  # UUID for Cycling Power Service
CPS_MEASUREMENT_UUID = "00002a63-0000-1000-8000-00805f9b34fb"  # UUID for Cycling Power Measurement

# Function to handle received notifications
def notification_handler(sender: int, data: bytearray):
    instantaneous_power = int.from_bytes(data[1:3], byteorder="little")
    print(f"Instantaneous Power: {instantaneous_power} W")

# Asynchronous function to scan and connect to the BLE device
async def connect_ble_device():
    devices = await BleakScanner.discover()
    for device in devices:
        # Match the device by UUID instead of name
        for adv_uuid in device.metadata.get("uuids", []):
            if CPS_UUID.lower() in adv_uuid.lower():  # Match by Cycling Power Service UUID
                print(f"Found device with CPS: {device.address}")
                async with BleakClient(device.address) as client:
                    print(f"Connected to {device.address}")
                    
                    # Start notifications
                    await client.start_notify(CPS_MEASUREMENT_UUID, notification_handler)

                    # Keep listening for notifications for 60 seconds
                    await asyncio.sleep(60)

                    # Stop notifications
                    await client.stop_notify(CPS_MEASUREMENT_UUID)
                    print("Disconnected")
                return  # Exit after connecting to the first match

    print("No device with Cycling Power Service found.")

# Main function to run the BLE connection and listener
async def main():
    await connect_ble_device()

# Run the BLE listener
if __name__ == "__main__":
    asyncio.run(main())
