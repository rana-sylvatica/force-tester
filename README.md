# Single-Axis Force and Cycle Tester

A desktop force and cycle testing system built using 3D printer components, a Raspberry Pi Pico, and a load cell.

## Overview

This project implements a force testing apparatus that can measure tension and compression forces while performing controlled movement cycles. It combines:

- **Motion control**: Uses a 3D printer control board and stepper motor system for precise linear movement
- **Force measurement**: Uses an HX711 load cell amplifier connected to a Raspberry Pi Pico
- **Position tracking**: Uses an AS5600 magnetic rotary encoder to track precise position
- **Data visualization**: Real-time plotting of force vs time or position

The system can perform single movements or cyclic tests with configurable parameters.

## Hardware Requirements

- 3D printer control board (compatible with Marlin firmware)
- Stepper motor with lead screw (4-start, 10mm lead recommended)
- Raspberry Pi Pico
- HX711 load cell amplifier
- Load cell (tension/compression)
- AS5600 magnetic rotary encoder
- Linear motion system (rails, bearings)
- Power supply for control board
- Two USB connections to computer

## Software Components

### PyQt Application (`force_tester.py`)

The main desktop application provides:
- Serial port connection management
- Test parameter configuration
- Real-time data visualization
- CSV data logging
- Manual jogging controls

### Pico Firmware (`encoder_and_load_cell_firmware.py`)

The firmware running on the Raspberry Pi Pico:
- Reads load cell values via HX711
- Reads position values via AS5600 encoder
- Performs calibration calculations
- Outputs CSV-formatted data over serial

## Python Dependencies

### Desktop Application
- Python 3.6+
- PyQt6
- pyqtgraph
- numpy
- pandas
- pyserial

Install dependencies with:
```
pip install PyQt6 pyqtgraph numpy pandas pyserial
```

### Pico Firmware
The firmware requires MicroPython to be installed on the Raspberry Pi Pico.

1. Download the latest MicroPython UF2 file for Raspberry Pi Pico from the [official MicroPython website](https://micropython.org/download/rp2-pico/)
2. Connect the Pico while holding the BOOTSEL button to enter programming mode
3. Copy the UF2 file to the mounted drive
4. After the Pico reboots, upload the `encoder_and_load_cell_firmware.py` file using your preferred method (Thonny IDE recommended)

## Getting Started

1. Connect the hardware:
   - Connect the Raspberry Pi Pico to your computer via USB
   - Connect the 3D printer control board to your computer via USB
   - Ensure the load cell is wired to the HX711, which connects to GPIO 16 (DOUT) and 17 (SCK) on the Pico
   - Wire the AS5600 encoder to the Pico's I2C0 bus (GPIO 4/5)

2. Flash the firmware:
   - Upload `encoder_and_load_cell_firmware.py` to the Raspberry Pi Pico

3. Run the application:
   ```
   python force_tester.py
   ```

4. In the application:
   - Select the appropriate COM ports
   - Connect to the devices
   - Configure test parameters
   - Zero the axis before beginning tests
   - Start testing and view real-time data
   - Data will be saved to the specified CSV file

## Test Parameters

- **Number of cycles**: Set to 0 for a single movement or specify the number of cycles
- **Speed**: Movement speed in mm/s (max 50mm/s)
- **Distance**: Movement distance in mm 
- **X-axis variable**: Plot time or distance on the X-axis
- **Y-axis variable**: Plot real-time force or peak forces on the Y-axis

## Data Output

The system logs:
- Timestamp (ms)
- Position (mm)
- Force (lbf)
- Cycle count (for cyclic tests)
- Peak tension and compression forces (for cyclic tests)

## Calibration

The load cell auto-tares on startup. For accurate force measurements, adjust the following constants in the firmware:
- `NO_LOAD_VALUE`: Raw value with no load
- `KNOWN_LOAD_VALUE`: Raw value with a known weight
- `KNOWN_LOAD_WEIGHT`: The known weight in pounds

## License

MIT License

Copyright (c) [year] [fullname]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

## Contributors

- Rana Labs

- Claude 3.7 Sonnet
