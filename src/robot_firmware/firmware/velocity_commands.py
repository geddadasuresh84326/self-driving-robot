import serial
import time
import sys

# ================================================================
# CONFIGURATION
# ================================================================
SERIAL_PORT = '/dev/ttyACM0'  # Change this to your Arduino's port (e.g., /dev/ttyUSB0 on Linux/Raspberry Pi)
BAUD_RATE = 115200

def initialize_serial(port, baud):
    try:
        arduino = serial.Serial(port, baud, timeout=0.1)
        print(f"Connected to {port} at {baud} baud.")
        
        # VERY IMPORTANT: Wait for Arduino to reboot after opening the serial port
        print("Waiting for Arduino to initialize...")
        time.sleep(3) 
        arduino.reset_input_buffer() 
        return arduino
        
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        sys.exit(1)

def send_velocities(arduino, right_rads, left_rads):
    """Formats and sends the velocity command in the custom protocol."""
    command = f"rp{right_rads:.2f},ln{left_rads:.2f}\n"
    arduino.write(command.encode('utf-8'))

def read_feedback(arduino):
    """Reads incoming data without blocking the main control loop."""
    if arduino.in_waiting > 0:
        try:
            # Read the line, decode it, and strip the trailing newline character
            feedback = arduino.readline().decode('utf-8').strip()
            return feedback
        except UnicodeDecodeError:
            # Catch garbled data that occasionally happens on startup
            pass
    return None

# ================================================================
# MAIN CONTROL LOOP
# ================================================================
def main():
    arduino = initialize_serial(SERIAL_PORT, BAUD_RATE)
    
    # Example control variables (You can update these dynamically from your algorithms)
    target_right_rads = 2.56
    target_left_rads = 2.56
    
    # Timing variables to control how fast Python sends commands
    last_send_time = time.time()
    send_interval = 0.05  # Send commands every 50ms (20Hz)

    print("Starting control loop. Press Ctrl+C to stop.")
    
    try:
        while True:
            current_time = time.time()
            
            # 1. Send commands at a fixed interval
            if current_time - last_send_time >= send_interval:
                send_velocities(arduino, target_right_rads, target_left_rads)
                last_send_time = current_time
                
            # 2. Continuously check for incoming feedback
            feedback = read_feedback(arduino)
            if feedback:
                print(f"Feedback Received -> {feedback}")
                
                # Optional: Parse the feedback if you need the values in Python
                # Example string: "rp2.56,ln2.56"
                """
                if "rp" in feedback and "ln" in feedback:
                    parts = feedback.split(',')
                    right_fb = float(parts[0].replace('rp', ''))
                    left_fb = float(parts[1].replace('ln', ''))
                    # You can now pass right_fb and left_fb into a Kalman filter or Odometry node
                """

    except KeyboardInterrupt:
        print("\nProcess interrupted by user. Stopping motors safely...")
    finally:
        # 3. Safe Shutdown: Send 0 velocity to motors before closing
        send_velocities(arduino, 0.0, 0.0)
        time.sleep(0.1) # Give it a moment to send
        arduino.close()
        print("Serial connection closed.")

if __name__ == '__main__':
    main()
