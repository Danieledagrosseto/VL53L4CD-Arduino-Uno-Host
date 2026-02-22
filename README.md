# VL53L4CD Arduino Uno Host

Arduino Uno I2C host firmware for interfacing with VL53L4CD Time-of-Flight (ToF) distance sensor breakout boards. Supports both interactive configuration mode and autonomous continuous ranging mode.

## Table of Contents

- [Features](#features)
- [Hardware Overview](#hardware-overview)
- [Tindie Listing](#tindie-listing)
- [Prerequisites](#prerequisites)
- [Getting Started](#getting-started)
- [Firmware Operation Modes](#firmware-operation-modes)
- [Interactive Setup Mode](#interactive-setup-mode)
- [Autonomous Ranging Mode](#autonomous-ranging-mode)
- [VL53L4CD Breakout Board Firmware](#vl53l4cd-breakout-board-firmware)
- [I2C Protocol](#i2c-protocol)

## Features

### Arduino Host Firmware

- **Dual Operation Modes**: Interactive setup/configuration or autonomous continuous ranging
- **Multi-Device Support**: Automatically detect and communicate with multiple VL53L4CD sensors on the I2C bus
- **Flexible Ranging**: Single-shot or continuous ranging with configurable periods
- **Multiple Units**: Distance measurements in millimeters, centimeters, or inches
- **Device Configuration**: Read and display sensor configurations (time budget, inter-measurement period, calibration values, thresholds)
- **Calibration Support**: Offset and crosstalk calibration
- **Threshold Configuration**: Adjust sigma and signal thresholds
- **Address Management**: Change I2C addresses dynamically
- **Non-blocking Timing**: Uses `millis()` for timing in autonomous mode
- **User Extensible**: Empty `process_data()` function for custom data processing

### VL53L4CD Breakout Board Features

- **PSoC4 (CY8C4200 family) I2C Slave**: Firmware running on breakout board acts as I2C slave (default address 0x70)
- **VL53L4CD Sensor**: STMicroelectronics Time-of-Flight ranging sensor
- **EEPROM Storage**: Persistent storage of calibration values, timing parameters, and I2C address
- **Multiple Ranging Units**: mm, cm, or inch
- **Configurable Timing**: Time budget (10-200 ms) and inter-measurement period (0-5000 ms)
- **Calibration**: Offset and crosstalk calibration with EEPROM storage
- **Threshold Control**: Configurable sigma and signal thresholds

## Hardware Overview

### Board Images

![Board Front](images/board_front.jpg)

![Board Back](images/board_back.jpg)

### J1 Connector Pins

- **Pin 1** (square pad): Vdd (3.3V or 5V)
- **Pin 2**: SDA (I2C Data)
- **Pin 3**: SCL (I2C Clock)
- **Pin 4**: NU (Not Used)
- **Pin 5**: GND (Ground)

### I2C Pull-ups

- **R1 and R2**: Weak 10kΩ pull-ups installed on SDA and SCL
- **J2 and J3**: Cut these solder jumpers if using external pull-ups
- Recommended for multi-device setups or long cable runs: Use external 4.7kΩ or 2.2kΩ pull-ups and cut J2/J3

### I2C Bus Configuration

- Default I2C slave address: **0x70**
- Configurable address range: 0x08 to 0x7F
- I2C clock frequency: 400 kHz (Fast Mode)
- Multiple devices supported on same bus with unique addresses

## Tindie Listing

For a concise product-style description of the breakout board features, see:

- [TINDIE.md](TINDIE.md)

## Prerequisites

- **Arduino CLI** installed and available on your PATH
- **Arduino Uno** (or compatible board) connected over USB
- **VL53L4CD breakout board(s)** connected via I2C
- **VS Code** with Arduino extension (optional, for task integration)

## Getting Started

### 1. Hardware Setup

1. Connect VL53L4CD breakout board to Arduino Uno:
   - J1 Pin 1 (Vdd) → Arduino 5V
   - J1 Pin 2 (SDA) → Arduino A4
   - J1 Pin 3 (SCL) → Arduino A5
   - J1 Pin 5 (GND) → Arduino GND

2. Verify connection: Observe the LED blink pattern at power-on:
   - **One long blink**: Normal address range (0x08-0xF0)
   - **One long + short blinks**: High address range (0xF0-0xFF)
   - Default address 0x70 shows one long blink

3. For multiple sensors, use unique I2C addresses (see [address change procedure](#change-i2c-address))

### 2. Configure Build Settings

1. Update the board and port in `.vscode/settings.json` if needed:
   ```json
   {
     "arduino.defaultBoard": "arduino:avr:uno",
     "arduino.defaultPort": "COM3",
     "arduino.defaultSketch": "src/VL53L4CD_Host"
   }
   ```

### 3. Choose Operation Mode

Edit `src/VL53L4CD_Host/VL53L4CD_Host.ino`:

- **For interactive setup mode**: Uncomment `#define SETUP_RANGING`
- **For autonomous ranging mode**: Leave `#define SETUP_RANGING` commented (default)

### 4. Build and Upload

Using VS Code tasks:
- Run task: **"Arduino: Compile"** to build the sketch
- Run task: **"Arduino: Upload"** to upload to the board

Or using Arduino CLI:
```bash
arduino-cli compile --fqbn arduino:avr:uno src/VL53L4CD_Host
arduino-cli upload -p COM3 --fqbn arduino:avr:uno src/VL53L4CD_Host
```

## Firmware Operation Modes

The host firmware supports two compilation modes controlled by the `SETUP_RANGING` macro:

### Mode Selection

```cpp
// Uncomment to enable interactive setup/configuration mode
// Comment out (or leave undefined) to enable autonomous ranging mode
// #define SETUP_RANGING
```

## Interactive Setup Mode

**When to use**: Device configuration, calibration, testing, and troubleshooting

### Requirements
- Serial connection required (115200 baud)
- Serial monitor open in Arduino IDE or terminal

### Features

**Interactive Commands:**
1. **Scan I2C and print configs** - Detect all I2C devices and display their configurations
2. **Read config for one address** - Read configuration from a specific sensor
3. **Ranging (single measurement)** - Perform single distance measurement
4. **Continuous ranging (all devices)** - Continuously measure all detected devices
5. **Set timing** - Configure time budget and inter-measurement period
6. **Offset calibration** - Perform offset calibration
7. **XTalk calibration** - Perform crosstalk calibration
8. **Restore defaults** - Reset sensor to factory defaults
9. **Sigma/signal thresholds** - Configure detection thresholds
10. **Restart** - Reload configuration from EEPROM
11. **Change I2C address** - Modify sensor I2C address

### Usage

1. Open serial monitor at 115200 baud
2. Wait for 10-second startup delay
3. Follow on-screen menu prompts
4. Enter command letters (1-9, A, B, H)
5. Respond to configuration prompts as needed

### Configuration Display

For each detected device, the following is displayed:
- I2C address (current and stored)
- Firmware revision
- Time budget (ms)
- Inter-measurement period (ms)
- Offset calibration (mm)
- Crosstalk value (kcps)
- Sigma threshold (mm)
- Signal threshold (kcps)

### Ranging Results

Each ranging operation returns:
- Distance (in selected unit)
- Range status code
- Signal rate (kcps)
- Ambient rate (kcps)
- Sigma (mm)
- Ambient per SPAD (kcps)
- Signal per SPAD (kcps)
- Number of SPADs

## Autonomous Ranging Mode

**When to use**: Production operation, embedded applications, continuous monitoring

### Features

- **No serial communication** - Operates independently without host interaction
- **Automatic device detection** - Scans and configures all connected sensors at startup
- **Continuous ranging** - Performs periodic distance measurements
- **Configurable period** - Default 100ms between ranging cycles (adjustable in code)
- **Non-blocking operation** - Uses `millis()` for timing, doesn't block other operations
- **User callback** - `process_data()` function called for each measurement

### Operation

1. Power-up delay: 10 seconds
2. I2C initialization at 400 kHz
3. Automatic device detection and configuration readout
4. Enter continuous ranging loop
5. For each ranging cycle:
   - Measure distance from all detected devices
   - Call `process_data()` for each reading
   - Wait for next cycle period

### Customization

Edit the following constants in the source code:

```cpp
static const uint8_t RANGING_UNIT = 0x52;        // 0x52=mm, 0x51=cm, 0x50=inch
static const uint16_t RANGING_PERIOD_MS = 100;   // Time between ranging cycles
```

### User Data Processing

Implement your custom logic in the `process_data()` function:

```cpp
void process_data(uint8_t deviceIndex, uint8_t address, 
                  uint16_t distance, uint8_t rangeStatus, 
                  const uint8_t* rangingData) {
    // Your code here
    // Examples:
    // - Log data to SD card
    // - Send data over wireless link
    // - Control actuators based on distance
    // - Apply filtering algorithms
}
```

**Parameters:**
- `deviceIndex`: Index in devices array (0-based)
- `address`: I2C address of the sensor
- `distance`: Distance reading (in configured unit)
- `rangeStatus`: Status code (0 = valid measurement)
- `rangingData`: Pointer to 15-byte ranging buffer with full sensor data

**Ranging Data Buffer Layout (15 bytes):**
- [0-1]: Distance (uint16_t, big-endian)
- [2]: Range status
- [3-4]: Signal rate (uint16_t, big-endian, kcps)
- [5-6]: Ambient rate (uint16_t, big-endian, kcps)
- [7-8]: Sigma (uint16_t, big-endian, mm)
- [9-10]: Ambient per SPAD (uint16_t, big-endian, kcps)
- [11-12]: Signal per SPAD (uint16_t, big-endian, kcps)
- [13-14]: Number of SPADs (uint16_t, big-endian)

## VL53L4CD Breakout Board Firmware

Each VL53L4CD breakout board contains a PSoC4 microcontroller from the CY8C4200 family running custom firmware that:

1. Acts as an I2C slave (default address 0x70)
2. Manages the VL53L4CD ToF sensor
3. Stores configuration in EEPROM
4. Handles ranging, calibration, and configuration commands
5. Provides a simple command-based interface

### Firmware Features

- **Power-On LED Indicator**: Each device blinks its onboard LED at power-on to indicate I2C address range:
  - **One long blink**: Address between 0x08 and 0xF0 (most common range)
  - **One long blink + N short blinks**: Address between 0xF0 and 0xFF, where N = (address - 0xF0)
- **Persistent Configuration**: All settings stored in EEPROM survive power cycles
- **Calibration Storage**: Offset and crosstalk values saved automatically
- **Watchdog Protection**: WDT ensures recovery from errors
- **Big-Endian Protocol**: Multi-byte values transmitted MSB first
- **Error Handling**: Invalid parameters constrained to valid ranges

### Default Configuration

- I2C Address: 0x70
- Time Budget: 50 ms
- Inter-measurement Period: 0 ms (continuous)
- Offset: 0 mm
- Crosstalk: 0 kcps
- Sigma Threshold: 15 mm
- Signal Threshold: 100 kcps

## I2C Protocol

The host communicates with VL53L4CD breakout boards using a command-based I2C protocol. Each breakout board acts as an I2C slave.

### Command Summary

| Command | Code | Description |
|---------|------|-------------|
| Ranging | 0x00 | Trigger single distance measurement |
| Set Timing | 0x01 | Configure time budget and inter-measurement period |
| Offset Cal | 0x02 | Run offset calibration |
| XTalk Cal | 0x03 | Run crosstalk calibration |
| Read Config | 0x04 | Read current configuration |
| Restore Defaults | 0x05 | Reset to factory defaults |
| Set Thresholds | 0x07 | Configure sigma/signal thresholds |
| Restart | 0x08 | Reload configuration from EEPROM |

### Change I2C Address

Special 4-step sequence to change sensor address:
1. Write: `00 A0`
2. Write: `00 AA`
3. Write: `00 A5`
4. Write: `00 <new_address>`

New address saved to EEPROM and takes effect immediately.

### Detailed Protocol Documentation

See [I2C_COMMANDS.md](I2C_COMMANDS.md) for complete protocol specification including:
- Command formats and parameters
- Response buffer layouts
- Timing requirements
- Usage examples
- Valid parameter ranges

## Troubleshooting

### No devices detected
- Check I2C wiring (SDA, SCL, GND, VDD)
- Verify power supply voltage (3.3V or 5V)
- Ensure pull-up resistors installed (R1/R2 or external)
- Use I2C scanner to verify sensor presence

### Ranging timeout
- Increase time budget
- Check for I2C bus interference
- Verify sensor has clear field of view
- Ensure proper target reflectivity

### Calibration issues
- Place target at precise known distance
- Use appropriate target (white matte surface recommended)
- Ensure stable mounting during calibration
- Increase sample count for better averaging

### Multiple sensors not responding
- Verify each has unique I2C address
- Check pull-up resistor strength (may need stronger for multiple devices)
- Ensure adequate power supply for all sensors
- Verify no address conflicts

## License

This project provides host firmware for interfacing with VL53L4CD breakout boards. See individual component datasheets and libraries for their respective licenses.

## References

- [VL53L4CD Datasheet](https://www.st.com/en/imaging-and-photonics-solutions/vl53l4cd.html)
- [I2C_COMMANDS.md](I2C_COMMANDS.md) - Complete I2C protocol documentation
