import asyncio
import threading
from bleak import BleakScanner, BleakClient
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# UUIDs for Cycling Power Service and Measurement Characteristic
CPS_UUID = "00001818-0000-1000-8000-00805f9b34fb"  # UUID for Cycling Power Service
CPS_MEASUREMENT_UUID = "00002a63-0000-1000-8000-00805f9b34fb"  # UUID for Cycling Power Measurement

# Global variables for power
instant_power = 0
power_readings = []
instant_cadence = 0

# Function to handle received notifications
def notification_handler(sender: int, data: bytearray):
    global instant_power, power_readings, instant_cadence

    # Parse the data based on the BLE CPS specification
    instant_power = int.from_bytes(data[1:3], byteorder="little")
    instant_cadence = int.from_bytes(data[3:5], byteorder="little")
    print(f"Instantaneous Power: {instant_power} W, Current cadence: {instant_cadence}")

    power_readings.append(instant_power)
    if len(power_readings) > 50:  # Keep last 50 readings for plotting
        power_readings.pop(0)

    # Schedule GUI update in the main thread
    root.after(0, update_gui)

# GUI update function
def update_gui():
    power_label.config(text=f"{instant_power} W")
    cadence_label.config(text=f"{instant_cadence} RPM")
    
    # Update plot
    ax.clear()
    ax.plot(power_readings, label="Power (W)")
    ax.legend(loc="upper left")
    ax.set_ylim([0, max(300, max(power_readings))])  # Adjust y-axis dynamically
    canvas.draw()

# Asynchronous function to scan and connect to BLE device
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

                    # Keep listening indefinitely
                    while True:
                        await asyncio.sleep(1)

# Function to run asyncio loop in a separate thread
def run_asyncio_loop():
    asyncio.run(connect_ble_device())

# GUI Setup using tkinter
root = tk.Tk()
root.title("Cycling Power Meter")

frame = ttk.Frame(root, padding=20)
frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

# Create a sub-frame to hold power and cadence side by side
sub_frame = ttk.Frame(frame)
sub_frame.grid(row=0, column=0)

# Power Name Label
power_name_label = ttk.Label(sub_frame, text="Power:", font=("Arial", 20), anchor="w")
power_name_label.grid(row=0, column=0, padx=20, pady=(10, 5), sticky="w")

# Cadence Name Label
cadence_name_label = ttk.Label(sub_frame, text="Cadence:", font=("Arial", 20), anchor="w")
cadence_name_label.grid(row=0, column=1, padx=20, pady=(10, 5), sticky="w")

# Instant Power Label
power_label = ttk.Label(sub_frame, text="0 W", font=("Arial", 48), anchor="w")
power_label.grid(row=1, column=0, padx=20, pady=(5, 20), sticky="w")

# Cadence Label
cadence_label = ttk.Label(sub_frame, text="0 RPM", font=("Arial", 48), anchor="w")
cadence_label.grid(row=1, column=1, padx=20, pady=(5, 20), sticky="w")

# Matplotlib Plot for Time Series of Power Readings
fig, ax = plt.subplots()
power_readings = [0] * 50  # Initial empty power readings
ax.plot(power_readings, label="Power (W)")
canvas = FigureCanvasTkAgg(fig, master=frame)
canvas.get_tk_widget().grid(row=2, column=0, padx=20, pady=20)

# Start BLE scanning and listening in background thread
asyncio_thread = threading.Thread(target=run_asyncio_loop)
asyncio_thread.daemon = True  # Ensure thread closes when main program exits
asyncio_thread.start()

# Start the GUI main loop
root.mainloop()
