# VL53L4CD Breakout Board (PSoC4 CY8C4200 Family I2C Host Interface)

Compact VL53L4CD Time-of-Flight breakout board with PSoC4 (CY8C4200 family) firmware for simple I2C integration and persistent on-board configuration.

Designed as a plug-and-play replacement for ultrasonic rangers such as **SRF02** and **SRF10** with no host software changes required for standard ranging workflows.

## Key Features

- **VL53L4CD ToF Sensor** for accurate short-range distance measurements
- **PSoC4 (CY8C4200 family) I2C Slave Interface** (default address `0x70`)
- **Multi-device I2C support** with configurable address (`0x08` to `0x7F`)
- **Two-wire host interface**: only the two I2C bus pins (`SDA` and `SCL`) are needed to interface one or several sensors with the host
- **EEPROM-backed settings** (address, calibration, timing, thresholds)
- **Selectable output units**: millimeters, centimeters, or inches
- **Configurable ranging timing**:
  - Time budget: **10–200 ms**
  - Inter-measurement period: **0–5000 ms**
- **Calibration support**:
  - Offset calibration
  - Crosstalk calibration
- **Threshold controls**:
  - Sigma threshold
  - Signal threshold
- **I2C pull-up flexibility**:
  - On-board weak 10kΩ pull-ups (R1/R2)
  - Optional external pull-ups via jumper cuts (J2/J3)

## SRF02/SRF10 Compatibility

- **Drop-in replacement behavior** for existing SRF02/SRF10 I2C integrations
- **No software changes needed** for typical read-distance command/response usage
- **Plug-and-play upgrade path** from ultrasonic to ToF sensing
- **Additional extended features** beyond SRF02/SRF10:
  - Time budget configuration commands
  - Calibration commands (offset and crosstalk)

## Typical Use Cases

- Multi-sensor distance monitoring over I2C
- Embedded proximity / occupancy detection
- Robotics and automation ranging nodes
- Fast prototyping with Arduino-compatible hosts

## Repository

Full firmware, host examples, and protocol docs are available here:

- https://github.com/Danieledagrosseto/VL53L4CD-Arduino-Uno-Host
