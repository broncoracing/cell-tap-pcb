import can
import struct
import time


# Initialize CAN bus
bus = None
def send_calibration_command(cmd, value):
    # Pack the command and value into data using struct
    data = struct.pack('>Bf', cmd, value)

    # Construct the calibration command frame
    cmd_frame = can.Message(
        arbitration_id=0x2EF,
        data=data,
        is_extended_id=False
    )

    # Send the calibration command frame
    bus.send(cmd_frame)

def receive_calibration_response():
    # Receive the response frame
    resp_start_time = time.time()
    response_frame = None
    while time.time() - resp_start_time < 1:
        latest_frame = bus.recv(timeout=1)
        if latest_frame is None:
            break
        if latest_frame.arbitration_id >= 0x2F0 and latest_frame.arbitration_id < 0x300:
            response_frame = latest_frame
            break

    if response_frame is None:
        print("Response timeout :(")
        return
    # Extract error number from the response frame
    errno = response_frame.data[0]

    # TODO support multiple boards
    # Output error message based on error number
    if errno == 0:
        print("Calibration command successful.")
    elif errno == 1:
        print("Flash write failed.")
    elif errno == 2:
        print("Invalid command.")
    elif errno == 3:
        print("Sensor out of range error.")
    elif errno == 4:
        print("Cannot compute ABC calibration: Missing at least one temperature calibration.")
    elif errno == 5:
        print("Failed to compute ABC calibration")
    else:
        print(f"Unknown error number: {errno}")

def main():
    # Menu for selecting the calibration command
    print("Select calibration action:")
    print("1. Clear calibration data")
    print("2. Calibrate temperature")
    print("3. Compute calibrations")
    choice = int(input("Enter your choice (1-3): "))

    if choice == 1:
        cmd = 0  # Clear calibration data
        value = 0
    elif choice == 2:
        cmd = int(input("Enter temperature to calibrate (1-3): "))
        if cmd < 1 or cmd > 3:
            print("Invalid temperature.")

        value = float(input("Enter calibration value (floating point): "))
    elif choice == 3:
        cmd = 4  # Compute calibrations
        value = 0
    else:
        print("Invalid choice.")
        return

    print("Connecting...")
    global bus
    bus = can.interface.Bus(channel='/dev/ttyACM2', bustype='slcan', bitrate=500000)
    print("Sending command...")
    # Send calibration command
    send_calibration_command(cmd, value)
    print("Awaiting response...")
    # Receive and process response
    receive_calibration_response()

    bus.shutdown()


if __name__ == "__main__":
    main()
