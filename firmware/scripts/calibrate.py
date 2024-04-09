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
    response_frames = []
    while time.time() - resp_start_time < 1:
        latest_frame = bus.recv(timeout=1)
        # print(hex(latest_frame.arbitration_id))
        if latest_frame is None:
            break
        if latest_frame.arbitration_id >= 0x2F0 and latest_frame.arbitration_id < 0x300:
            response_frames.append(latest_frame)
            # break

    if len(response_frames) == 0:
        print("response timeout")
        return
    if len(response_frames) < 7:
        print(f"Partial response timeout :( only recieved {len(response_frames)} responses")

    # Extract error number from the response frame
    for response_frame in response_frames:
        errno = response_frame.data[0]
        # Previous debugging code
        # if errno >= 200:
        #     print(f"{errno} {extract_datum(response_frame.data)}")
        #     continue
        print(f"Board ID {hex(response_frame.arbitration_id)}:")
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


def extract_datum(data):
    return struct.unpack('>xxxxf', data)[0]


def receive_datums():
    # Receive the response frame
    resp_start_time = time.time()
    response_frames = []
    # Give the boards 10 seconds to spew out data
    while time.time() - resp_start_time < 10:
        latest_frame = bus.recv(timeout=1)
        # print(hex(latest_frame.arbitration_id))
        if latest_frame is None:
            break
        if latest_frame.arbitration_id >= 0x2F0 and latest_frame.arbitration_id < 0x300:
            response_frames.append(latest_frame)
            # break

    board_abcs = {}
    cal_temps = {}
    cal_adc_readings = {}
    for response_frame in response_frames:
        board_num = response_frame.arbitration_id - 0x2F0 + 10

        errno = response_frame.data[0]

        if board_num not in board_abcs.keys():
            board_abcs[board_num] = (
                [0] * 15,
                [0] * 15,
                [0] * 15
            )
        if board_num not in cal_temps.keys():
            cal_temps[board_num] = [0,0,0]

        if board_num not in cal_adc_readings.keys():
            cal_adc_readings[board_num] = (
                [0] * 15,
                [0] * 15,
                [0] * 15
            )

        if errno < 20:
            pass
        elif errno < 40:
            channel = errno - 20
            board_abcs[board_num][0][channel] = extract_datum(response_frame.data)
        elif errno < 60:
            channel = errno - 40
            board_abcs[board_num][1][channel] = extract_datum(response_frame.data)
        elif errno < 80:
            channel = errno - 60
            board_abcs[board_num][2][channel] = extract_datum(response_frame.data)

        elif errno == 80:
            cal_temps[board_num][0] = extract_datum(response_frame.data)
        elif errno == 81:
            cal_temps[board_num][1] = extract_datum(response_frame.data)
        elif errno == 82:
            cal_temps[board_num][2] = extract_datum(response_frame.data)

        elif errno < 120:
            channel = errno - 100
            cal_adc_readings[board_num][0][channel] = extract_datum(response_frame.data)
        elif errno < 140:
            channel = errno - 120
            cal_adc_readings[board_num][1][channel] = extract_datum(response_frame.data)
        elif errno < 160:
            channel = errno - 140
            cal_adc_readings[board_num][2][channel] = extract_datum(response_frame.data)



    print('Board calibrations:')
    for board, cal in board_abcs.items():
        temps = cal_temps[board]
        print(f"Board {board}: calibrated at {temps[0]}C, {temps[1]}C, {temps[2]}C")
        for i in range(15):
            print(f"  Thermistor {i}: A={cal[0][i]} B={cal[1][i]} C={cal[2][i]}")
            adc_readings = cal_adc_readings[board]
            print(f"    Cal data: {temps[0]}C->{adc_readings[0][i]} {temps[1]}C->{adc_readings[1][i]} {temps[2]}C->{adc_readings[2][i]} ")

    print('-------------------')
    # Extract error number from the response frame
    for response_frame in response_frames:
        errno = response_frame.data[0]
        if errno >= 20:
            continue
        print(f"Board ID {hex(response_frame.arbitration_id)}:")
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
        elif errno < 20:
            print(f"Unknown error number: {errno}")


def main():
    # Menu for selecting the calibration command
    print("Select calibration action:")
    print("1. Clear calibration data")
    print("2. Calibrate temperature")
    print("3. Compute calibrations")
    print("4. Dump calibration values")
    choice = int(input("Enter your choice (1-4): "))

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
    elif choice == 4:
        cmd = 5  # Dump cal values
        value = 0
    else:
        print("Invalid choice.")
        return

    print("Connecting...")
    global bus
    bus = can.interface.Bus(channel='/dev/ttyACM24', bustype='slcan', bitrate=500000)
    print("Sending command...")
    # Send calibration command
    send_calibration_command(cmd, value)
    print("Awaiting response...")

    if cmd == 5:
        receive_datums()
    else:
        # Receive and process response
        receive_calibration_response()

    bus.shutdown()


if __name__ == "__main__":
    main()
