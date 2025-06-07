import serial
import time
import csv

COM_PORT = 'COM3'       # Replace with the correct port (check Arduino IDE Tools > Port)
BAUD_RATE = 9600        # Must match Serial.begin()
OUTPUT_FILE = 'ultrasonic_data.csv'

def main():
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Wait for Arduino to reset

        with open(OUTPUT_FILE, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Timestamp', 'Left Distance (cm)', 'Right Distance (cm)'])

            print("Logging started. Press Ctrl+C to stop.")
            while True:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    try:
                        left_val, right_val = line.split(',')
                        timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
                        writer.writerow([timestamp, left_val, right_val])
                        print(f"{timestamp} | Left: {left_val} cm, Right: {right_val} cm")
                    except Exception as e:
                        print(f"Error: {e} -- Line: {line}")
    except serial.SerialException:
        print(f"Could not open serial port {COM_PORT}. Check connection and port name.")
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == '__main__':
    main()
