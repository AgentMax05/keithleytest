import board
import digitalio
import time
import adafruit_max31856
import csv
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import sys # Import sys to exit gracefully if needed

def format_hhmmss_with_seconds(seconds):
    """Convert seconds to HH:MM:SS format and return the original seconds."""
    hrs = int(seconds // 3600)
    mins = int((seconds % 3600) // 60)
    secs = int(seconds % 60)
    return f"{hrs:02}:{mins:02}:{secs:02}", seconds

# --- User Input ---
while True:
    save_data_input = input("Do you want to save the data to a CSV file and PDF plot? (yes/no): ").lower().strip()
    if save_data_input in ['yes', 'no', 'y', 'n']:
        save_data = save_data_input in ['yes', 'y']
        break
    else:
        print("Invalid input. Please enter 'yes' or 'no'.")

while True:
    try:
        sleep_time_input = input("Enter the time constant between readings in seconds (e.g., 1 for every second): ")
        sleep_time = float(sleep_time_input)
        if sleep_time >= 0:
            break
        else:
            print("Time constant cannot be negative.")
    except ValueError:
        print("Invalid input. Please enter a number.")

# --- Hardware Setup ---
# Setup SPI and MAX31856
# You might need to adapt these lines based on your specific board and connections
try:
    spi = board.SPI()
except NotImplementedError:
    print("SPI not available on this platform. Temperature sensor will not work.")
    spi = None

# allocate a CS pin and set the direction
try:
    cs = digitalio.DigitalInOut(board.D5) # Replace D5 with your actual CS pin
    cs.direction = digitalio.Direction.OUTPUT
except NotImplementedError:
    print("Digital pins not available on this platform. Temperature sensor will not work.")
    cs = None

# create a thermocouple object with the above
thermocouple = None
if spi and cs:
    try:
        thermocouple = adafruit_max31856.MAX31856(spi, cs)
    except RuntimeError as e:
        print(f"Error initializing MAX31856 sensor: {e}")
        thermocouple = None

# --- Data Logging Setup (only if saving data) ---
csv_filename = "temperature_log.csv"
if save_data:
    try:
        with open(csv_filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Time (s)", "Temperature (C)"])  # CSV header
        print(f"\nSaving data to {csv_filename}")
    except Exception as e:
        print(f"Error creating CSV file: {e}")
        save_data = False # Disable saving if file creation fails

start_time = time.time()
times = []
temps = []

# --- Measurement Loop ---
try:
    if thermocouple: # Only run the loop if the sensor was initialized successfully
        print("\nStarting temperature measurement. Press Ctrl+C to stop.")
        while True:
            elapsed = time.time() - start_time
            temp = thermocouple.temperature

            # Store data
            times.append(elapsed)
            temps.append(temp)

            # Save to CSV (only if save_data is True)
            if save_data:
                try:
                    with open(csv_filename, mode='a', newline='') as file:
                        writer = csv.writer(file)
                        formatted_time, total_seconds = format_hhmmss_with_seconds(elapsed)
                        writer.writerow([f"{total_seconds:.2f}", f"{temp:.2f}"]) # Save seconds to CSV
                except Exception as e:
                    print(f"Error writing to CSV: {e}")
                    # Optionally disable saving if errors occur repeatedly
                    # save_data = False

            # Print with HH:MM:SS and seconds display
            formatted_time, total_seconds = format_hhmmss_with_seconds(elapsed)
            print(f"Time: {formatted_time} ({total_seconds:.2f}s) | Temp: {temp:.2f} °C")

            time.sleep(sleep_time)
    else:
        print("Temperature sensor not available, skipping measurement loop.")

except KeyboardInterrupt:
    print("\nMeasurement stopped by user.")

finally:
    # --- Save Data (only if saving data and data exists) ---
    if save_data and times and temps: # Only save plot if saving is enabled and there is data
        print("Saving PDF plot...")
        try:
            pdf_filename = "temperature_plot.pdf"
            with PdfPages(pdf_filename) as pdf:
                plt.figure(figsize=(10, 6))
                plt.plot(times, temps, marker='o', linestyle='-')
                plt.title("Temperature vs Time")
                plt.xlabel("Time (s)")
                plt.ylabel("Temperature (°C)")
                plt.grid(True)
                pdf.savefig()
                plt.close()

            print(f"CSV saved as {csv_filename}")
            print(f"PDF saved as {pdf_filename}")
        except Exception as e:
            print(f"Error saving plot: {e}")
    elif save_data and (not times or not temps):
         print("No data collected to save.")
    else:
         print("Data saving was not enabled.")

    print("\nProgram terminated.")
