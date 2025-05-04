# XPA125B-Control
Program to control XPA125B with IC-705 using bluetooth serial connection to ESP32 module.


# ESP32 Controller for XPA125B Amplifier with IC-705 via Bluetooth CI-V

## Overview

This project uses an ESP32 microcontroller to interface between an Icom IC-705 radio and a Xiegu XPA125B linear amplifier. It provides automatic band switching for the amplifier based on the radio's frequency and keys the amplifier based on the radio's actual transmit status, all communicated via Bluetooth CI-V. It also allows an external PTT input (like a foot switch) connected to the ESP32 to trigger the IC-705's PTT via CI-V command.

## Features

*   **Automatic Amplifier Band Switching:** Reads the IC-705's frequency via Bluetooth CI-V and sets the appropriate band voltage for the XPA125B via a DAC output.
*   **Amplifier PTT Control:** Polls the IC-705's actual TX/RX status via CI-V and controls the amplifier's PTT line accordingly.
*   **Radio PTT Control:** Uses a GPIO input pin (with pull-up) to allow an external switch (e.g., foot pedal) to trigger the IC-705's PTT ON/OFF state via CI-V commands.
*   **Bluetooth Connectivity:** Connects directly to the IC-705's built-in Bluetooth Serial Port Profile (SPP) for CI-V communication.
*   **Safety Timers:** Includes configurable transmit time limiter and lockout period for amplifier protection.
*   **Serial Monitor Status:** Provides status updates and allows debugging via the USB Serial Monitor.

## Hardware Requirements

*   ESP32 module (tested on ESP-WROOM-32, but likely compatible with many others)
*   Icom IC-705 Radio
*   Xiegu XPA125B Amplifier (connection using ACC)
*   Connecting Wires, including power (in this case I am using HAM PSU that outputs 13.8V, connectors for power wires should be good idea too)
*   Optional: Foot switch or other momentary switch for external PTT input.
*   Stable power supply for the ESP32, as it can affect DAC voltages. I have used DC-DC converter QS-1205CME-3A (https://vi.aliexpress.com/item/1005005393995240.html)
*   2N2222 or other NPN transistor, 10kOhm, 2kOhm and 1kOhm resistors, 0.1 µF ceramic capacitor
*   Breadboard to solder components on
*   Mini-DIN 6 pin female connector
*   Other 2 pin connector of your choice for foot pedal
*   Some box to put everything in

## Pinout (ESP32 Side)

*   **GPIO 26:** Amplifier PTT Output -> Connect to XPA125B ACC PTT input (Active HIGH for TX)
*   **GPIO 25:** Band DAC Output -> Connect to XPA125B ACC Band Data input
*   **GPIO 34:** PTT Input -> Connect one side of external PTT switch (other side to GND). Internal pull-up used (LOW = Request TX). **Note:** Pin 34 is INPUT ONLY on many ESP32s.
*   **GND:** Common Ground -> Connect to XPA125B ACC GND, PTT switch GND, and ESP32 Power Supply GND.

## Schematics
    ![image](https://github.com/user-attachments/assets/de07165c-9e72-42bc-b973-b98037844a57)


## Software & Setup

1.  **IDE:** Arduino IDE or PlatformIO.
2.  **Libraries:**
    *   `BluetoothSerial.h` (usually included with ESP32 Arduino Core)
    *   `Regexp.h` (Install via Arduino Library Manager or PlatformIO Library Manager)
3.  **Configuration:** Key parameters (Polling intervals, Timers, Bluetooth Name, CI-V Debug) are hardcoded near the top of the `.ino` file within the `START CONFIG` / `END CONFIG` block. Modify these directly if needed.
4.  **Flashing:** Compile and upload the sketch (`.ino` file) to your ESP32 board.

## How it Works

1.  **Bluetooth Connection:** The ESP32 advertises itself with the name defined in `bluetooth_device_name`. Pair and connect the IC-705's Bluetooth [SERIAL PORT] function to the ESP32.
2.  **Frequency Polling:** The ESP32 periodically sends a CI-V command (0x03) to the IC-705 asking for its frequency.
3.  **Band DAC Setting:** When a frequency response is received, the ESP32 determines the corresponding amateur band and outputs a specific voltage on the DAC pin (GPIO 25) using `dacWrite()`. The XPA125B reads this voltage to switch bands.
4.  **PTT Status Polling:** The ESP32 periodically sends a CI-V command (0x1C 0x00) asking for the radio's TX/RX status.
5.  **Amplifier PTT Keying:** Based on the radio's response to the PTT status poll, the ESP32 sets the Amplifier PTT pin (GPIO 26) HIGH (for TX) or LOW (for RX), respecting safety timers and band validity.
6.  **External PTT Input:** When the external PTT input pin (GPIO 34) is pulled LOW (grounded by the switch), the ESP32 debounces the input and sends a CI-V command (0x1C 0x00 0x01) to the IC-705 to initiate transmission. When the pin goes HIGH again, it sends the command to stop transmission (0x1C 0x00 0x00).

## Usage

1.  Wire the ESP32 to the XPA125B ACC port and optional PTT switch according to the Pinout.
2.  Flash the firmware to the ESP32.
3.  Power on the ESP32, IC-705, and XPA125B.
4.  On the IC-705, go to bluetooth data device pairing. Find the ESP32 (`XPA125B Control` by default) and pair with it.
5.  On the IC-705, set `Serial Port Function` to `CI-V (Echo Back OFF)`.
6.  Operate the radio. The XPA125B should automatically switch bands as you change frequency on the IC-705, and the amplifier should key when the IC-705 transmits (either via its own PTT or the external switch connected to the ESP32). Monitor the ESP32 Serial Output for status messages.

## Serial Monitor Commands

Connect to the ESP32 via USB and open the Serial Monitor at 115200 baud.

*   `status`: Prints the current connection status, frequency, band, PTT states, and timers.
*   `setcivdebug true` / `setcivdebug false`: Enables/Disables detailed CI-V message logging.
*   `restart` or `reboot`: Restarts the ESP32.

## Disclaimer

Use this code and hardware setup at your own risk. Incorrect wiring or configuration could potentially damage your radio or amplifier. Ensure proper grounding and verify PTT timing and sequencing are appropriate for your equipment, especially on faster modes.
