import serial
import time
import sys
import errno

# Configuration
SERIAL_PORT = '/dev/ttyAMA0'
BAUD_RATE = 115200

def check_serial():
    print(f"--- Listening on {SERIAL_PORT} at {BAUD_RATE} baud ---")
    print("If you see scrolling data (gibberish/symbols), the connection is GOOD.")
    print("If you see nothing, check RX/TX wiring.")
    print("If you get 'Device or resource busy', kill other python/mavproxy tasks.")
    print("Press Ctrl+C to stop.")
    print("-------------------------------------------------------")

    ser = None
    try:
        # Open serial port with a timeout so we don't block forever on read
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        
        while True:
            # Check if there is data waiting in the buffer
            if ser.in_waiting > 0:
                # Read whatever is in the buffer
                data = ser.read(ser.in_waiting)
                # Print hex representation for clarity, then raw bytes
                hex_data = data.hex()
                print(f"RAW ({len(data)} bytes): {hex_data[:60]}...") 
            else:
                # Wait a tiny bit so we don't hog CPU
                time.sleep(0.1)

    except serial.SerialException as e:
        print(f"ERROR: Serial Exception: {e}")
        if e.errno == errno.EBUSY:
             print("HINT: The port is busy. Run 'sudo fuser -k /dev/serial0' to clear it.")
    except OSError as e:
        print(f"ERROR: OS Error: {e}")
    except KeyboardInterrupt:
        print("\n--- Test Stopped ---")
    finally:
        if ser and ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    check_serial()
