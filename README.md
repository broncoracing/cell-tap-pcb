# Bronco Racing Segment Monitor PCB

This repository contains the hardware design, firmware, and software calibration scripts for the segment monitor PCB.

The segment monitor contains an STM32f103R8T6 microcontroller which interfaces 15 ADCs (12 external for monitoring cell temperatures and 3 onboard on the PCB.)
A separate break-away board for digital and power isolation is also included.

## Hardware details

The thermistor measurement circuit consists of a 10k thermistor (or connector for a thermistor), a precision 10k reference resistor, and a filter capacitor (The filter capacitor is mostly important to prevent measurement errors due to the changing input impedance of the microcontroller's analog pins when they sample the voltage, this is likely a very small effect that wouldn't cause large errors anyway).

The reference resistor and thermistor form a voltage divider circuit, which allows the resistance of the thermistor to be determined by reading the center voltage.

The circuit is designed specifically to work with 10k NTC thermistors, but it would likely work with any thermistor with a resistance in the 1k-100k range.

Each external thermistor connector also has an LED located next to the connector. This LED indicates whether the thermistor's resistance is valid. If the thermistor is within a reasonable range, the LED will be off. If the microcontroller detects too high or low of a voltage (indicating a short circuit or disconnected thermistor) the LED is turned on. Upon powering up the LEDs will all turn on and off in sequence to verify that the LEDs themselves function correctly.

On the back side of the board (opposite the microcontroller) each cell tap connection connects to an SMD fuse and a trace which leads to the cell tap connector. These fuses are located close to the cell tap connection to meet FSAE rules for fusing of tractive system wires, and eliminate the need for inline fuses on each cell tap wire. 
Due to a minor design oversight, the fuses on the endmost cell taps can be damaged or knocked off the board if the busbars for the Radlok maintenance plugs are rotated too far inwards when the segment is being assembled. Take care when assembling or disassembling the segments to avoid this issue.

## Firmware documentation
The firmware reads data from all the connected thermistors and sends them over the CAN bus at 100ms intervals.

To avoid overloading CAN bus with too many concurrent messages, each segment sends out only 4 thermistor values in each message, and rotates which thermistor temperatures are sent out in each message.

The segment PCB firmware also includes a thermistor calibration routine, which stores measured ADC values for three different temperatures  the Steinhart-Hart equation as described below in the Performance Testing & Analysis section. To perform this calibration, a simple CAN protocol similar to the one utilized by the CAN bootloader was developed. This protocol consists of four commands to perform the following actions:
- Clear calibration data
- Record resistances at a given temperature
- Calculate Steinhart-Hart parameters based on three temperature values
- Read out calibration data over CAN bus and print it to the console

In order to simplify the user experience, a Python command line application was developed to execute these commands and interpret the response sent back over the CAN bus.
This script is located at `firmware/scripts/calibrate.py`. Since I was too lazy to make it automatically detect the CAN bus interface, you'll have to manually set it in the script (or upgrade the script to use a command line parameter), it will be something like `/dev/ttyACM*` on linux.
