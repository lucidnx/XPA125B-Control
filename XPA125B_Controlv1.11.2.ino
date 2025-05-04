// *****************************************************************************
// ** XPA125B Controller for IC-705 via Bluetooth CI-V                      **
// **                                                                         **
// ** Functionality:                                                          **
// ** - Uses Pin 34 Input (e.g., foot pedal) to control Radio PTT via         **
// **   CI-V command (1C 00 01/00).                                           **
// ** - Uses Radio PTT Status (polled via CI-V 1C 00) to control Amplifier    **
// **   PTT Output on Pin 26.                                                 **
// ** - Uses Radio Frequency (polled via CI-V 03) to control Band DAC Output  **
// **   on Pin 25 for automatic band switching on the XPA125B amplifier.      **
// **                                                                         **
// ** Version: 1.11.2                                                         **
// *****************************************************************************

#include <ArduinoJson.h>      // Although included, JSON parsing isn't actively used. Kept for potential future features.
#include "BluetoothSerial.h"  // For ESP32 Bluetooth Classic Serial Profile
#include <Regexp.h>           // For basic pattern matching (used for frequency validation)

// --- LED Definition ---
#ifndef LED_BUILTIN
#define LED_BUILTIN 2 // Default ESP32 onboard LED if not defined elsewhere
#endif
const int ICOM_CHANGE_BLINK_MS = 50; // Blink duration for frequency change indication

// *********** START CONFIG ***********
// These values are hardcoded.
int serial_baud = 115200; // Baud rate for USB Serial Monitor/Commands
bool CIV_DEBUG = false;   // Enable detailed CI-V parsing debug output via Serial

// Amplifier PTT Control
int tx_to_rx_delay = 0;   // TX->RX delay for Amplifier PTT OFF (ms). Add delay if needed for amp sequencing.
int tx_limit = 300;       // Amplifier TX safety timer limit (seconds). 0 to disable.
int tx_block_time = 60;   // Amplifier TX block duration after limit is reached (seconds).

// Radio PTT Control (from Pin 34)
int debounce_delay = 10;  // Debounce time for PTT Input Pin 34 (ms). Adjust if needed for noisy switches.

// Bluetooth
const char *bluetooth_device_name = "XPA125B Control"; // Name the ESP32 advertises via Bluetooth

// CI-V Polling Intervals
const int civ_ptt_poll_interval = 100;    // How often to ask Radio for PTT status (ms). Affects Amp PTT reaction speed.
const int civ_freq_poll_interval = 300;   // How often to ask Radio for Frequency (ms). Affects Band DAC update speed.
// *********** END CONFIG ***********

// --- Pin Definitions ---
const int ptt_amp_pin = 26;    // GPIO Output to Amplifier PTT Relay (Active HIGH = TX)
const int dac_band_pin = 25;   // GPIO DAC Output (DAC Channel 1 on most ESP32s) for Band Voltage
const int ptt_input_pin = 34; // GPIO Input (Input Only) for external PTT Request (LOW = Request TX)

// --- CI-V Constants ---
const byte CIV_PREAMBLE = 0xFE;             // CI-V message start
const byte CIV_EOM = 0xFD;                  // CI-V message end
const byte CIV_RADIO_ADDR = 0xA4;           // Default IC-705 CI-V address
const byte CIV_CONTROLLER_ADDR = 0xE0;      // Default controller CI-V address (can often be E0)
const byte CIV_CMD_READ_FREQ = 0x03;        // CI-V command to read frequency
const byte CIV_CMD_SET_FREQ = 0x00;         // CI-V command used for frequency transceive/response
const byte CIV_CMD_TRANSCEIVER_STATUS = 0x1C; // CI-V command ID for Read/Set TX/RX status
const byte CIV_SUBCMD_READ_STATUS = 0x00;     // CI-V subcommand for Reading TX/RX status (used for both read and set with data byte)

// --- Global Objects ---
BluetoothSerial BTserial; // Bluetooth Serial object

// --- Global Variables ---
String serialValue;        // Buffer for incoming USB Serial data
char serialEOL = '\n';     // End-of-line character for USB Serial commands

int current_band = 0;      // Currently selected band (0 = None/Unknown)
int previous_band = 0;     // Previous band (used for change detection)

// Amplifier TX Timing & State
unsigned long current_tx_millis = 0; // Timestamp for TX duration calculation
unsigned long tx_start_millis = 0;   // Timestamp when Amp TX started
int tx_seconds_true = 0;   // Total seconds Amp has been continuously transmitting
bool is_transmitting = false; // Tracks Amplifier PTT output state (Pin 26 HIGH = true)
unsigned long current_block_millis = 0;    // Timestamp for block duration calculation
unsigned long tx_block_start_millis = 0; // Timestamp when TX block started
int tx_block_seconds_remaining = 0; // Countdown for TX block
bool tx_blocked = false;    // Flag indicating Amplifier TX is currently blocked by safety timer

// PTT Input Pin State (Pin 34)
unsigned long last_debounce_time = 0;     // Timestamp for debounce logic
bool last_pin34_state = HIGH;             // Debounced state of Pin 34 (HIGH = RX requested)
volatile bool current_pin34_state = HIGH; // Raw state from ISR (volatile because it's modified by ISR)

// Radio State Info (from CI-V polling)
String frequency = "0";         // Current frequency string (e.g., "14074000")
String previous_frequency = "0"; // Previous frequency string

// CI-V Communication Buffering
byte bt_buffer[32];       // Buffer for incoming CI-V bytes from Bluetooth
byte bt_com = 0;          // CI-V message framing state (0=idle, 1=receiving, 2=complete)
byte bt_ptr = 0;          // Pointer/index for bt_buffer

// CI-V Polling Timers
unsigned long last_civ_ptt_poll = 0;  // Timestamp of last PTT status poll
unsigned long last_civ_freq_poll = 0; // Timestamp of last Frequency poll

// --- Function Prototypes ---
void setState(bool amp_tx_active);
void setBand(String band);
void setFreq(String freq);
void send_civ_ptt_request();
void send_civ_freq_request();
void send_civ_set_ptt(bool tx_on); // Sends CI-V command to set Radio TX/RX state
void printTimestamp();
void IRAM_ATTR handlePTTInterrupt(); // Interrupt Service Routine for Pin 34
void processSerial(String commandLine);
bool regexMatch(const char *value, const char *regex); // Helper for regex matching

// --- Helper Functions ---

// Simple string splitting function
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;
  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

// Prints a timestamp to the Serial monitor
void printTimestamp()
{
  unsigned long currentMillis = millis();
  unsigned long seconds = currentMillis / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  seconds %= 60; minutes %= 60; hours %= 24; // Keep within 24 hours for display
  Serial.print("[");
  if (hours < 10) Serial.print("0"); Serial.print(hours); Serial.print(":");
  if (minutes < 10) Serial.print("0"); Serial.print(minutes); Serial.print(":");
  if (seconds < 10) Serial.print("0"); Serial.print(seconds); Serial.print("] ");
}

// Interrupt Service Routine for PTT Input Pin 34
// Reads the pin state and stores it in a volatile variable. Keep this FAST.
void IRAM_ATTR handlePTTInterrupt() {
    current_pin34_state = digitalRead(ptt_input_pin);
}

// Simple regex matching helper
bool regexMatch(const char *value, const char *regex)
{
  char tempValue[32]; // Temporary buffer for the value
  strncpy(tempValue, value, sizeof(tempValue) - 1);
  tempValue[sizeof(tempValue) - 1] = '\0'; // Ensure null termination
  MatchState ms;
  ms.Target(tempValue);
  char result = ms.Match(regex); // Perform the match
  return (result > 0);           // Return true if match found, false otherwise
}


// --- Core Logic Functions ---

// Controls the AMPLIFIER PTT Output Pin (26) based on the desired state
// Takes safety conditions (TX block, Band 0) into account.
void setState(bool amp_tx_active)
{
    // --- Safety Checks BEFORE changing state ---
    if (amp_tx_active) { // Requesting Amplifier TX ON
        // Check 1: Is TX currently blocked by the safety timer?
        if (tx_blocked) {
            printTimestamp(); Serial.print("AMP PTT: TX Request BLOCKED (Timer: ");
            Serial.print(tx_block_seconds_remaining); Serial.println("s).");
            // Ensure Amp is OFF if it happened to be ON
            if (is_transmitting) {
                digitalWrite(ptt_amp_pin, LOW); // Force Amp PTT OFF
                is_transmitting = false;        // Update internal state
                tx_seconds_true = 0;            // Reset TX duration counter
                digitalWrite(LED_BUILTIN, LOW); // Turn LED OFF
            }
            return; // Exit function, do not activate Amp PTT
        }
        // Check 2: Is the current band valid (not 0)?
        if (current_band == 0) {
            printTimestamp(); Serial.println("AMP PTT: TX Request BLOCKED (Band is 0 / Out of band).");
            // Ensure Amp is OFF if it happened to be ON
            if (is_transmitting) {
                digitalWrite(ptt_amp_pin, LOW); // Force Amp PTT OFF
                is_transmitting = false;        // Update internal state
                tx_seconds_true = 0;            // Reset TX duration counter
                digitalWrite(LED_BUILTIN, LOW); // Turn LED OFF
            }
            return; // Exit function, do not activate Amp PTT
        }
    }

    // --- Proceed with State Change ---
    bool state_changed = false;
    bool current_amp_ptt_output_state = digitalRead(ptt_amp_pin) == HIGH; // Read current physical pin state

    if (amp_tx_active) { // Requesting Amp TX ON
        if (!current_amp_ptt_output_state) { // Amp is currently OFF, turn it ON
            digitalWrite(ptt_amp_pin, HIGH);
            if (!is_transmitting) { // Internal state was also RX, normal transition
                is_transmitting = true;         // Update internal state
                tx_start_millis = millis();     // Record TX start time
                state_changed = true;
                // Log state change unless verbose debugging is on (reduces noise)
                if (!CIV_DEBUG) { printTimestamp(); Serial.println("Amp State: RX -> TX"); }
            }
             // else: Internal state mismatch (state was TX, pin was LOW). Might indicate external control conflict or timing issue.
        }
        // else: Amp is already ON.
            // if (!is_transmitting): Internal state mismatch (state was RX, pin was HIGH). Could force sync if needed.
    } else { // Requesting Amp TX OFF (RX)
        if (current_amp_ptt_output_state) { // Amp is currently ON, turn it OFF
             if (tx_to_rx_delay > 0) { delay(tx_to_rx_delay); } // Apply TX->RX delay if configured
             digitalWrite(ptt_amp_pin, LOW);
             if (is_transmitting) { // Internal state was TX, normal transition
                 is_transmitting = false;        // Update internal state
                 tx_seconds_true = 0;            // Reset TX duration counter
                 state_changed = true;
                 // Log state change unless verbose debugging is on
                 if (!CIV_DEBUG) { printTimestamp(); Serial.println("Amp State: TX -> RX"); }
             }
             // else: Internal state mismatch (state was RX, pin was HIGH).
        }
        // else: Amp is already OFF.
            // if (is_transmitting): Internal state mismatch (state was TX, pin was LOW). Could force sync if needed.
    }

    // Update LED to match final amplifier PTT state only if state changed or LED is out of sync
    if (state_changed || (digitalRead(LED_BUILTIN) != is_transmitting)) {
        digitalWrite(LED_BUILTIN, is_transmitting);
    }
}

// Set Band DAC Voltage based on the band string (e.g., "160", "80", "40") using legacy Arduino dacWrite
void setBand(String band)
{
  int bandInt = band.toInt();

  // Avoid redundant DAC writes if band number hasn't changed,
  // but allow explicitly setting band 0 even if it was already 0.
  if (bandInt == current_band && band != "0") return;

  if (CIV_DEBUG) { printTimestamp(); Serial.print("DEBUG BAND: Setting Band '"); Serial.print(band); Serial.println("' ..."); }

  previous_band = current_band; // Store the old band
  current_band = bandInt;       // Update the current band
  int dac_value = 0;            // DAC value (0-255) corresponding to the band voltage

  // Map band number to DAC value for Pin 25
  // Voltages listed are approximate based on ESP32 DAC output and XPA125B requirements.
  switch (bandInt) {
    // Band: DAC Value -> Approx. Output Voltage (Pin 25), XPA125B Expected Voltage
    case 160: dac_value = 10; break;  // ~0.21V, Expected 0.23V
    case 80:  dac_value = 30; break;  // ~0.45V, Expected 0.46V
    case 60:  dac_value = 49; break;  // ~0.68V, Expected 0.69V
    case 40:  dac_value = 68; break;  // ~0.92V, Expected 0.92V
    case 30:  dac_value = 86; break;  // ~1.14V, Expected 1.15V
    case 20:  dac_value = 105; break; // ~1.37V, Expected 1.38V
    case 17:  dac_value = 124; break; // ~1.60V, Expected 1.61V
    case 15:  dac_value = 142; break; // ~1.84V, Expected 1.84V
    case 12:  dac_value = 161; break; // ~2.07V, Expected 2.07V
    case 10:  dac_value = 180; break; // ~2.30V, Expected 2.30V
    case 6:   dac_value = 199; break; // ~2.54V, Expected 2.53V
    default:  // Includes band "0" or any unrecognized band
        dac_value = 0;
        current_band = 0; // Ensure current_band is 0 for unknown/off states
        break;
  }

  dacWrite(dac_band_pin, dac_value); // Use standard Arduino DAC function
  if (CIV_DEBUG) { printTimestamp(); Serial.print("DEBUG BAND: DAC Value "); Serial.print(dac_value); Serial.print(" written for Band "); Serial.println(current_band); }

  // Safety Check: If the band is set to 0 (invalid/out of band) while the amplifier
  // is transmitting, immediately force the amplifier to RX state.
  if (current_band == 0 && is_transmitting) {
    printTimestamp(); Serial.println("WARNING: Band set to 0 while Amp TX. Forcing Amp RX state.");
    setState(false); // Request Amp RX
  }
}

// Process Frequency String, Determine Band, and Call setBand()
void setFreq(String freq)
{
    // Basic validation: must be numeric, or "0"
    if (!regexMatch(freq.c_str(), "^[0-9]+$") && freq != "0") {
        if (CIV_DEBUG) { printTimestamp(); Serial.print("Invalid frequency format received: "); Serial.println(freq); }
        return;
    }
    // No change needed if frequency is the same (and not "0")
    if (freq == frequency && freq != "0") return;

    previous_frequency = frequency; // Store old frequency
    frequency = freq;             // Update current frequency
    if (CIV_DEBUG) { printTimestamp(); Serial.print("Processing Freq: "); Serial.println(frequency); }

    // If frequency is "0", turn off band voltage and exit
    if (frequency == "0") {
        setBand("0");
        return;
    }

    // Blink LED briefly on frequency change (only if not transmitting)
    if (!is_transmitting) {
         digitalWrite(LED_BUILTIN, HIGH); delay(ICOM_CHANGE_BLINK_MS); digitalWrite(LED_BUILTIN, LOW);
    }

    String determinedBand = "0"; // Default to band 0 (unknown/out of band)
    // Convert frequency string to number for range checking
    unsigned long freq_long = strtoul(frequency.c_str(), NULL, 10);

    // Determine band based on frequency ranges (adjust ranges if needed for your band plan)
    if      (freq_long >= 1800000 && freq_long < 2000000)   { determinedBand = "160"; }
    else if (freq_long >= 3500000 && freq_long < 4000000)   { determinedBand = "80"; }
    else if (freq_long >= 5000000 && freq_long < 5500000)   { determinedBand = "60"; }  // Wider range common for 60m
    else if (freq_long >= 7000000 && freq_long < 7300000)   { determinedBand = "40"; }
    else if (freq_long >= 10100000 && freq_long < 10150000) { determinedBand = "30"; }
    else if (freq_long >= 14000000 && freq_long < 14350000) { determinedBand = "20"; }
    else if (freq_long >= 18068000 && freq_long < 18168000) { determinedBand = "17"; }
    else if (freq_long >= 21000000 && freq_long < 21450000) { determinedBand = "15"; }
    else if (freq_long >= 24890000 && freq_long < 24990000) { determinedBand = "12"; }
    // Note: 11m band check commented out, uncomment and adjust DAC value in setBand if needed.
    // else if (freq_long >= 26900000 && freq_long < 27500000) { determinedBand = "11"; }
    else if (freq_long >= 28000000 && freq_long < 29700000) { determinedBand = "10"; }
    else if (freq_long >= 50000000 && freq_long < 54000000) { determinedBand = "6"; }

    setBand(determinedBand); // Set the DAC voltage based on the determined band
}

// --- CI-V Command Sending Functions ---

// Send CI-V command to Poll Radio PTT Status (1C 00)
void send_civ_ptt_request() {
    if (!BTserial.hasClient()) return; // Don't send if no BT client connected
    // Command: FE FE A4 E0 1C 00 FD
    byte command[] = {
        CIV_PREAMBLE, CIV_PREAMBLE,
        CIV_RADIO_ADDR, CIV_CONTROLLER_ADDR,
        CIV_CMD_TRANSCEIVER_STATUS, CIV_SUBCMD_READ_STATUS,
        CIV_EOM
    };
    if (CIV_DEBUG) { printTimestamp(); Serial.println("CI-V TX> Poll Radio PTT Status (1C 00)..."); }
    BTserial.write(command, sizeof(command));
}

// Send CI-V command to Poll Radio Frequency (03)
void send_civ_freq_request() {
    if (!BTserial.hasClient()) return; // Don't send if no BT client connected
    // Command: FE FE A4 E0 03 FD
    byte command[] = {
        CIV_PREAMBLE, CIV_PREAMBLE,
        CIV_RADIO_ADDR, CIV_CONTROLLER_ADDR,
        CIV_CMD_READ_FREQ,
        CIV_EOM
    };
    if (CIV_DEBUG) { printTimestamp(); Serial.println("CI-V TX> Poll Radio Frequency (03)..."); }
    BTserial.write(command, sizeof(command));
}

// Send CI-V Command to Set Radio PTT Status (TX ON or TX OFF) using 1C 00 + data
// This command directly tells the radio to go into TX or RX.
void send_civ_set_ptt(bool tx_on) {
    if (!BTserial.hasClient()) return; // Don't send if no BT client connected
    byte ptt_data = tx_on ? 0x01 : 0x00; // Data byte: 01 = TX, 00 = RX
    // Command: FE FE A4 E0 1C 00 <data> FD
    byte command[] = {
        CIV_PREAMBLE, CIV_PREAMBLE,
        CIV_RADIO_ADDR, CIV_CONTROLLER_ADDR,
        CIV_CMD_TRANSCEIVER_STATUS, CIV_SUBCMD_READ_STATUS,
        ptt_data, // The data byte that sets the state
        CIV_EOM
    };
    // Only log this potentially frequent command when debugging is enabled
    if (CIV_DEBUG) { printTimestamp(); Serial.print("CI-V TX> Set Radio PTT (1C 00 "); Serial.print(ptt_data, HEX); Serial.println(")..."); }
    BTserial.write(command, sizeof(command));
}

// --- Serial Command Processor (for USB Serial Monitor) ---
void processSerial(String commandLine) {
  commandLine.trim(); // Remove leading/trailing whitespace
  if (commandLine.length() == 0) return; // Ignore empty commands

  String command = getValue(commandLine, ' ', 0); // Get the first word as the command
  String value = getValue(commandLine, ' ', 1);   // Get the second word as the value
  command.toLowerCase(); // Make command case-insensitive

  printTimestamp(); Serial.print("Serial Cmd Received: '"); Serial.print(command);
  if (value.length() > 0) { Serial.print("' Value: '"); Serial.print(value); }
  Serial.println("'");

  if (command == "restart" || command == "reboot") {
      printTimestamp(); Serial.println("Rebooting ESP32...");
      delay(100); // Short delay to allow serial message to send
      ESP.restart();
  }
  else if (command == "status") {
      printTimestamp(); Serial.println("--- STATUS ---");
      Serial.print(" BT Client: "); Serial.println(BTserial.hasClient() ? "Connected" : "Disconnected");
      Serial.print(" PTT Input (Pin "); Serial.print(ptt_input_pin); Serial.print(") Request: "); Serial.println(last_pin34_state == LOW ? "TX (LOW)" : "RX (HIGH)");
      Serial.print(" Radio Freq: "); Serial.print(frequency); Serial.print(" Hz | Band: ");
          if (current_band != 0) { Serial.print(current_band); Serial.print("m"); } else { Serial.print("N/A"); }
          Serial.println();
      Serial.print(" Amplifier PTT (Pin "); Serial.print(ptt_amp_pin); Serial.print("): "); Serial.print(is_transmitting ? "TX (HIGH)" : "RX (LOW)");
      if (is_transmitting) { Serial.print(" | Amp TX Time: "); Serial.print(tx_seconds_true); Serial.print("s"); }
      if (tx_blocked) { Serial.print(" | TX BLOCKED (Rem: "); Serial.print(tx_block_seconds_remaining); Serial.print("s)"); }
      Serial.println();
      Serial.print(" CI-V Debug: "); Serial.println(CIV_DEBUG ? "ON" : "OFF");
      Serial.println("--------------");
  }
  else if (command == "setcivdebug") {
      // Set CIV_DEBUG based on value ("true", "1", "on" or "false", "0", "off")
      if (value == "true" || value == "1" || value == "on") {
          CIV_DEBUG = true;
      } else if (value == "false" || value == "0" || value == "off") {
          CIV_DEBUG = false;
      }
      printTimestamp(); Serial.print("CI-V Debug changed to: "); Serial.println(CIV_DEBUG ? "ON" : "OFF");
  }
  // Add other commands here if needed
  else {
    printTimestamp(); Serial.print("Unknown command: '"); Serial.print(command); Serial.println("'. Type 'status' for info.");
  }
}

// --- Setup ---
void setup(void)
{
    pinMode(LED_BUILTIN, OUTPUT); digitalWrite(LED_BUILTIN, HIGH); // Turn LED on during setup

    Serial.begin(serial_baud);
    delay(500); // Wait for Serial monitor to connect
    Serial.println("\n\n=======================================");
    Serial.print(" Booting XPA125B Controller (v"); Serial.print("1.11.2"); Serial.println(")");
    Serial.println("=======================================");
    printTimestamp(); Serial.println("Firmware init: IC-705 Mode, Direct Amp PTT, DAC Band Control");
    printTimestamp(); Serial.println("NOTE: Using hardcoded configuration values.");

    // Setup Pins
    printTimestamp(); Serial.println("Configuring Pins...");
    pinMode(ptt_amp_pin, OUTPUT);       // Configure Amplifier PTT pin as Output
    digitalWrite(ptt_amp_pin, LOW);     // Ensure Amplifier PTT is initially OFF (RX)
    is_transmitting = false;            // Sync internal state

    pinMode(dac_band_pin, ANALOG);      // Configure Band DAC pin for ANALOG output (required by dacWrite)
    setBand("0");                       // Set initial band voltage to 0 (off) using dacWrite

    pinMode(ptt_input_pin, INPUT_PULLUP); // Configure PTT Input pin with internal pull-up resistor (HIGH = idle/RX)

    // Attach Interrupt for PTT Input Pin
    noInterrupts(); // Disable interrupts temporarily
    current_pin34_state = digitalRead(ptt_input_pin); // Read initial state safely
    interrupts(); // Re-enable interrupts
    last_pin34_state = current_pin34_state; // Initialize debounced state
    attachInterrupt(digitalPinToInterrupt(ptt_input_pin), handlePTTInterrupt, CHANGE); // Trigger ISR on any change
    printTimestamp(); Serial.print("PTT Input (Pin "); Serial.print(ptt_input_pin); Serial.print(") initial state: ");
    Serial.println(current_pin34_state == LOW ? "LOW (Request TX)" : "HIGH (Request RX)");

    // Start Bluetooth Serial
    printTimestamp(); Serial.print("Starting Bluetooth SPP: '"); Serial.print(bluetooth_device_name); Serial.println("'");
    if (!BTserial.begin(bluetooth_device_name)) {
        printTimestamp(); Serial.println("!!! Bluetooth Initialization FAILED !!! Halting.");
        while(1) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(100); } // Fast blink indicates fatal error
    } else {
        printTimestamp(); Serial.println("Bluetooth Serial started. Waiting for client connection...");
    }

    printTimestamp(); Serial.println("--- Setup Complete ---");

    // Print initial status summary
    Serial.println("--- Initial State ---");
    Serial.print(" PTT Input Request (Pin "); Serial.print(ptt_input_pin); Serial.print("): "); Serial.println(last_pin34_state == LOW ? "TX" : "RX");
    Serial.print(" Band DAC (Pin "); Serial.print(dac_band_pin); Serial.print("): "); Serial.print(current_band); if (current_band !=0) Serial.print("m"); else Serial.print("N/A"); Serial.println();
    Serial.print(" Amplifier PTT (Pin "); Serial.print(ptt_amp_pin); Serial.print("): "); Serial.println(is_transmitting ? "TX" : "RX");
    Serial.println("---------------------");

    delay(1000); // Pause before final ready indication
    // Blink LED 3 times to indicate ready state
    for (int i=0; i<3; i++) { digitalWrite(LED_BUILTIN, HIGH); delay(150); digitalWrite(LED_BUILTIN, LOW); delay(150); }
    printTimestamp(); Serial.println("Ready.");
    digitalWrite(LED_BUILTIN, is_transmitting); // Set LED to reflect initial (expected RX) amp state
}


// --- Main Loop ---
void loop(void)
{
    // --- Process Serial Input (USB commands) ---
    while (Serial.available()) {
        char c = Serial.read();
        serialValue += c; // Append character to buffer
        // Process buffer when newline received or buffer is full
        if (c == serialEOL || serialValue.length() > 100) {
            serialValue.trim(); // Remove whitespace
            if (serialValue.length() > 0) {
                processSerial(serialValue); // Process the received command
            }
            serialValue = ""; // Clear the buffer
        }
    }

    // --- Process PTT Input Pin (Pin 34) Debounce & Send Radio TX Command ---
    // This section reads the raw PTT input state (set by ISR), debounces it,
    // and if the debounced state changes, sends a CI-V command to the radio
    // to set its TX/RX state accordingly.
    bool pin34_state_now;
    noInterrupts(); // Atomically read volatile variable modified by ISR
    pin34_state_now = current_pin34_state;
    interrupts();

    // Simple Debounce Logic:
    // Check if the raw state read now is different from the last *debounced* state
    if (pin34_state_now != last_pin34_state) {
        // If it's different, check if enough time has passed since the last *raw* state change
        if ((millis() - last_debounce_time) >= debounce_delay) {
            // Debounce time has passed, the state is considered stable.
            bool previous_debounced_state = last_pin34_state; // Store state before update
            last_pin34_state = pin34_state_now; // Update the official debounced state

            // Determine desired Radio TX state based on the new debounced state (LOW = TX Request)
            bool request_radio_tx = (last_pin34_state == LOW);

            // Send the CI-V command to the Radio *only if* the debounced state actually flipped
            if (last_pin34_state != previous_debounced_state) {
                 if (CIV_DEBUG) { printTimestamp(); Serial.print("PTT Input (Pin 34): Debounced change -> Request Radio "); Serial.println(request_radio_tx ? "TX" : "RX"); }
                 send_civ_set_ptt(request_radio_tx); // Send CI-V 1C 00 01 (TX) or 1C 00 00 (RX)
            }
            // else: Debounce confirmed the state hasn't *actually* flipped (noise spike?), do nothing.
        }
        // else: Not enough time passed since last raw change, wait longer for debounce.
    }

    // Update the debounce timer reference point *whenever* the raw state changes.
    // This ensures the debounce timer starts from the last detected transition.
    static bool last_raw_pin34_state_tracker = HIGH; // Track the raw state to reset timer only on change
    if (pin34_state_now != last_raw_pin34_state_tracker) {
        last_debounce_time = millis(); // Reset debounce timer
        last_raw_pin34_state_tracker = pin34_state_now; // Update tracker
    }


    // --- Process Icom CI-V Communication (Polling and Parsing) ---
    if (BTserial.hasClient()) { // Only proceed if a Bluetooth client (the IC-705) is connected

        // --- STEP 1: Read and Buffer ALL available incoming Bluetooth CI-V data FIRST ---
        while (BTserial.available()) {
            int bt_c_int = BTserial.read(); // Read one byte
            if (bt_c_int < 0) continue;     // Skip if read error
            byte bt_c = (byte)bt_c_int;

            // Print raw incoming bytes if debugging
            if (CIV_DEBUG) { Serial.print(bt_c, HEX); Serial.print(" "); }

            // Simple CI-V Framing State Machine: Looks for FE FE ... FD sequence
            if (bt_c == CIV_PREAMBLE) {
                if (bt_com == 1) { bt_ptr = 0; } // Second FE seen, reset buffer ptr
                else { // First FE seen
                    bt_ptr = 0;
                    memset(bt_buffer, 0, sizeof(bt_buffer));
                    bt_com = 1; // State: receiving
                }
            } else if (bt_com == 1 && bt_c == CIV_EOM) {
                bt_com = 2; // State: complete message received
                if (CIV_DEBUG) { Serial.println(); } // Newline after raw hex dump
            } else if (bt_com == 1) { // Receiving data byte
                if (bt_ptr < sizeof(bt_buffer) - 1) {
                    bt_buffer[bt_ptr++] = bt_c; // Store byte
                } else { // Buffer overflow
                    printTimestamp(); Serial.println("ERROR: CI-V RX Buffer Overflow! Resetting parser.");
                    bt_com = 0; bt_ptr = 0; // Reset parser state
                }
            }
            // Ignore bytes if not in state 1
        } // end while BTserial.available()


        // --- STEP 2: Process any complete CI-V message received in the buffer ---
        if (bt_com == 2) { // A complete message (FE FE ... FD) has been buffered
            if (CIV_DEBUG && bt_ptr > 0) { // Log the received buffer content if debugging
                printTimestamp(); Serial.print("CI-V Rx: Processing Buffer (len="); Serial.print(bt_ptr); Serial.print("):");
                for (int k = 0; k < bt_ptr; k++) { Serial.print(" "); if (bt_buffer[k] < 16) Serial.print("0"); Serial.print(bt_buffer[k], HEX); }
                Serial.println();
            }

            bool processed = false; // Flag to track if we understood this message
            String local_bt_freq; // Local variable for frequency parsing

            // --- Check for PTT Status Response (From Radio -> To Controller, response to 1C 00 poll) ---
            // Format: E0 A4 1C 00 <data> (Length 5)
            if (bt_ptr == 5 && bt_buffer[0] == CIV_CONTROLLER_ADDR && bt_buffer[1] == CIV_RADIO_ADDR &&
                bt_buffer[2] == CIV_CMD_TRANSCEIVER_STATUS && bt_buffer[3] == CIV_SUBCMD_READ_STATUS)
            {
                processed = true;
                byte ptt_data = bt_buffer[4];       // Data byte (00=RX, 01=TX)
                bool radio_is_tx = (ptt_data == 0x01); // Radio's reported TX state

                if (CIV_DEBUG) { printTimestamp(); Serial.print("CI-V Rx: PTT Status Response. Radio is "); Serial.println(radio_is_tx ? "TX" : "RX"); }

                // Control the AMPLIFIER based on the Radio's reported state
                // Only call setState if reported state differs from amp's current state, or if debugging
                if (radio_is_tx != is_transmitting || CIV_DEBUG) {
                    // Only log state change if not debugging (setState logs when CIV_DEBUG is off)
                    if (!CIV_DEBUG && radio_is_tx != is_transmitting) {
                         printTimestamp(); Serial.print("Amp State Update (from Radio Poll): Requesting Amp "); Serial.println(radio_is_tx ? "TX" : "RX");
                    }
                    setState(radio_is_tx); // Control amp_ptt_pin (Pin 26)
                }
            }

            // --- Check for Frequency Response (03 poll resp) OR Async Frequency Update (00 transceive) ---
            // Format: E0 A4 03 <freq_bcd...> (Length >= 8) OR 00 A4 00 <freq_bcd...> (Length >= 8)
            else if (bt_ptr >= 8 &&
                     (bt_buffer[0] == CIV_CONTROLLER_ADDR || bt_buffer[0] == 0x00) && // Dest is us or broadcast
                     bt_buffer[1] == CIV_RADIO_ADDR &&                 // Source is Radio
                     (bt_buffer[2] == CIV_CMD_READ_FREQ || bt_buffer[2] == CIV_CMD_SET_FREQ)) // Freq command
            {
                processed = true;
                if (CIV_DEBUG) { printTimestamp(); Serial.print("CI-V Rx: Freq Msg Received (Cmd "); Serial.print(bt_buffer[2],HEX); Serial.print(")."); }

                // --- Frequency BCD Parsing Logic (Bytes 3-7) ---
                unsigned long freq_hz = 0;
                unsigned long multiplier = 1;
                bool parse_error = false;
                for (int i = 3; i <= 7; i++) { // Iterate through the 5 BCD bytes
                    if (i < bt_ptr) {
                        byte b = bt_buffer[i];
                        byte l = b & 0x0F;        // Lower nibble
                        byte h = (b >> 4) & 0x0F; // Upper nibble
                        if (l > 9 || h > 9) { parse_error = true; break; } // Validate BCD
                        freq_hz += l * multiplier; multiplier *= 10;
                        freq_hz += h * multiplier; multiplier *= 10;
                    } else { parse_error = true; break; } // Buffer too short (shouldn't happen here)
                }

                if (!parse_error && freq_hz > 0) {
                     local_bt_freq = String(freq_hz); // Convert parsed frequency to String
                     // Update frequency and band DAC if frequency changed
                     if (local_bt_freq != frequency) {
                         printTimestamp(); Serial.print("Radio Freq Update Received: "); Serial.print(local_bt_freq); Serial.println(" Hz");
                         setFreq(local_bt_freq); // Process frequency and set band
                     } else if (CIV_DEBUG) { Serial.println(" Freq unchanged."); }
                } else if (parse_error && CIV_DEBUG) { Serial.println(" Freq parse error occurred."); }
                 else if (freq_hz == 0 && CIV_DEBUG) { Serial.println(" Parsed frequency is 0 Hz."); }
                // --- End Frequency Parsing Logic ---
            } // End Frequency processing check

            // --- Add checks for other CI-V messages here if needed ---

            if (!processed && CIV_DEBUG && bt_ptr > 0) {
                printTimestamp(); Serial.println("CI-V Rx: Unhandled Buffer Content.");
            }

            // Reset CI-V parser state machine for the next message
            bt_com = 0; bt_ptr = 0;
        } // End if (bt_com == 2)


        // --- STEP 3: Check timers and perform CI-V polling AFTER processing incoming data ---
        unsigned long currentMillisLoop = millis();

        // Check and Send PTT Status Poll (Relatively Fast)
        if (currentMillisLoop - last_civ_ptt_poll >= civ_ptt_poll_interval) {
            send_civ_ptt_request();
            last_civ_ptt_poll = currentMillisLoop;
        }

        // Check and Send Frequency Poll (Slower)
        if (currentMillisLoop - last_civ_freq_poll >= civ_freq_poll_interval) {
             send_civ_freq_request();
             last_civ_freq_poll = currentMillisLoop;
        }

    } else { // --- Bluetooth Client Disconnected ---
         // Actions to take when the radio disconnects
         if (frequency != "0") { // Reset band/freq only if previously tuned
             printTimestamp(); Serial.println("Bluetooth client disconnected. Resetting frequency and band DAC.");
             setFreq("0"); // Set frequency to "0", which calls setBand("0")
         }
         if (is_transmitting) { // Force Amp RX if it was transmitting
            printTimestamp(); Serial.println("Forcing Amp RX state due to BT disconnect.");
            setState(false); // Request Amp RX state
         }
         // Reset polling timers and CI-V parser state
         last_civ_ptt_poll = 0; last_civ_freq_poll = 0;
         bt_com = 0; bt_ptr = 0;
         // Reset PTT input state tracking to default (RX request)
         last_pin34_state = HIGH;
         last_raw_pin34_state_tracker = HIGH;
    } // End if/else BTserial.hasClient()


    // --- Process Amplifier TX Timer & Blocking Logic ---
    // Tracks Amplifier PTT active time and enforces safety timer (tx_limit).
    if (is_transmitting) {
        current_tx_millis = millis();
        int elapsed_seconds = (current_tx_millis - tx_start_millis) / 1000;

        if (elapsed_seconds != tx_seconds_true) { // Update TX time counter only once per second
            tx_seconds_true = elapsed_seconds;
            // Log every 10 seconds of continuous TX
            if (tx_seconds_true > 0 && tx_seconds_true % 10 == 0) {
                printTimestamp(); Serial.print("Amp TX Time: "); Serial.print(tx_seconds_true); Serial.println("s");
            }

            // Check Amplifier TX Time Limit
            if (tx_limit > 0 && tx_seconds_true >= tx_limit) {
                printTimestamp(); Serial.print("!!! Amp TX Limit Reached ("); Serial.print(tx_limit); Serial.println("s) !!! Forcing Amp RX & Blocking TX.");
                setState(false); // Force Amp PTT OFF

                // Activate TX blocking
                tx_blocked = true;
                tx_block_start_millis = millis();
                tx_block_seconds_remaining = tx_block_time;
                printTimestamp(); Serial.print("Amp TX Blocked for "); Serial.print(tx_block_time); Serial.println(" seconds.");

                // Optional: Try to command Radio RX if PTT input is still active
                 if (last_pin34_state == LOW) {
                    printTimestamp(); Serial.println("INFO: Sending Radio RX command (1C 00 00) due to Amp TX Limit override.");
                    send_civ_set_ptt(false); // Send TX OFF command to Radio
                 }
            } // End tx_limit check
        } // End elapsed_seconds update
    } // End if (is_transmitting)


    // --- Process TX Block Timer Countdown ---
    if (tx_blocked) {
        current_block_millis = millis();
        int elapsed_block_seconds = (current_block_millis - tx_block_start_millis) / 1000;
        int remaining = tx_block_time - elapsed_block_seconds;

        if (remaining <= 0) { // Block time expired
            tx_blocked = false;
            tx_block_seconds_remaining = 0;
            printTimestamp(); Serial.println("Amp TX Block Timer Expired. TX re-enabled.");
        } else { // Block still active
            if (remaining != tx_block_seconds_remaining) { // Update remaining time display once per second
                tx_block_seconds_remaining = remaining;
                // Log remaining time every 5 seconds
                if (tx_block_seconds_remaining > 0 && tx_block_seconds_remaining % 5 == 0) {
                    printTimestamp(); Serial.print("Amp TX Block Remaining: "); Serial.print(tx_block_seconds_remaining); Serial.println("s");
                }
            }
        }
    } // End if (tx_blocked)

    yield(); // Allow ESP32 background tasks
} // End loop()