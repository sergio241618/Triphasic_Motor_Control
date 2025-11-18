import serial
import time
import sys

# --- Configuration ---
# Change this to the serial port of your ESP32
# Linux: /dev/ttyUSB0
# macOS: /dev/cu.usbserial-XXXX
# Windows: COM3
DEFAULT_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200
# ---------------------

def get_serial_port():
    """ Tries to guess the default serial port based on the OS. """
    if sys.platform.startswith('win'):
        return 'COM3'
    if sys.platform.startswith('darwin'):
        # macOS
        return '/dev/cu.usbserial-0001' # Change this
    # Assume Linux/other
    return '/dev/ttyUSB0'

def send_command(ser, channel, values):
    """
    Formats and sends a command to the ESP32.
    - channel: "CH1", "CH2", or "CH3"
    - values: "025" or "025,040,075"
    """
    # Format the command according to the protocol
    command = f"SET {channel} {values}\n"
    
    # Send the data as bytes
    ser.write(command.encode('utf-8'))
    
    # Print what we sent (for debugging)
    print(f"PC -> ESP32: {command.strip()}") # .strip() removes the \n for clean printing

def connect_to_esp32(port=None, baud=BAUD_RATE):
    """
    Tries to connect to the specified serial port.
    Returns the 'ser' object on success, or None on failure.
    """
    if port is None:
        port = get_serial_port()
        
    print(f"Attempting to connect to {port} at {baud} bps...")
    
    try:
        # We change the timeout from 1 second to 50ms (0.05)
        # This prevents ser.readline() from blocking the script indefinitely
        # if the ESP32 is not sending data.
        ser = serial.Serial(port, baud, timeout=0.05)
        
        # !! IMPORTANT !!
        # Wait 2 seconds. When pyserial opens the port,
        # the DTR pin of the USB-Serial chip is activated,
        # resetting the ESP32. We need to give it time to boot up.
        time.sleep(2)
        
        ser.flushInput() # Clear any junk data from the input buffer
        print("Connection successful.")
        return ser
        
    except serial.SerialException as e:
        print(f"Error: Could not open port {port}.")
        print(f"Details: {e}")
        print("Ensure the ESP32 is connected and you are not using")
        print("another serial monitor (e.g., 'idf.py monitor') at the same time.")
        return None