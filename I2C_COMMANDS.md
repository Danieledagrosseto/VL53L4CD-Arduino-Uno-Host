# I2C Command Reference (VL53L4CD breakout board)

This document describes the I2C slave commands implemented by the firmware and the command sequences a host should use to access all features.

The firmware runs as an I2C slave with a default address of 0x70. The address can be changed and stored in EEPROM.

## Conventions

- All multi-byte values are big-endian (MSB first).
- The I2C write buffer is parsed as bytes `wrBuf[0..]`.
- The I2C read buffer is returned as bytes `rdBuf[0..]`.
- If a command needs time to complete (ranging or calibration), the host should delay before reading the result. If read too early, bytes may still be 0xFF.
- EEPROM writes are retried internally. If EEPROM fails, the device enters an error loop and the WDT resets it.

## I2C Address

- Default I2C slave address: 0x70
- Valid address range: 0x08 to 0x7F

### Change I2C Address (sequence)
Send the following 4 write transactions, in order, to the current slave address:

1. `00 A0`
2. `00 AA`
3. `00 A5`
4. `00 <new_address>`

Notes:
- `<new_address>` must be in the range 0x08 to 0x7F.
- The new address is saved to EEPROM and takes effect immediately.

## Commands Summary

Each command is issued as an I2C write to the slave address. Some commands then require a read from the slave to get results.

| Command | Code | Write Payload | Description |
| --- | --- | --- | --- |
| Ranging | 0x00 | `00 <unit>` | Trigger single ranging measurement; read results later |
| Set timing | 0x01 | `01 <tb_ms> <im_ms_msb> <im_ms_lsb>` | Set time budget and inter-measurement period |
| Offset calibration | 0x02 | `02 <dist_msb> <dist_lsb> <samples_msb> <samples_lsb>` | Run offset calibration and save result |
| XTalk calibration | 0x03 | `03 <dist_msb> <dist_lsb> <samples_msb> <samples_lsb>` | Run crosstalk calibration and save result |
| Read config | 0x04 | `04` | Load config into read buffer; read config bytes |
| Restore defaults | 0x05 | `05` | Reset EEPROM parameters to defaults |
| Sigma/signal thresholds | 0x07 | `07 <sigma_msb> <sigma_lsb> <signal_msb> <signal_lsb>` | Update sigma and signal thresholds |
| Restart | 0x08 | `08` | Reload EEPROM values and slave address |

Note: The comment header mentions command 0x06 ("get EEPROM data"), but there is no handler in the firmware. The implemented readback is command 0x04.

## Command Details and Sequences

### 1) Ranging command (0x00)

**Write:**
- `00 <unit>`

**Unit codes:**
- `0x52` = millimeters (MM)
- `0x51` = centimeters (CM)
- `0x50` = inches (INCH)

**Sequence:**
1. Host writes `00 <unit>`.
2. Wait for the measurement to complete.
   - Recommended delay: at least the configured time budget plus a small margin (for example, time budget + 10 ms).
3. Host reads 15 bytes from the slave.

**Read buffer (15 bytes):**

| Byte | Name | Notes |
| --- | --- | --- |
| 0 | distance_msb | distance in selected unit |
| 1 | distance_lsb | distance in selected unit |
| 2 | range_status | VL53L4CD range status |
| 3 | signal_rate_msb | kcps (raw) |
| 4 | signal_rate_lsb | kcps (raw) |
| 5 | ambient_rate_msb | kcps (raw) |
| 6 | ambient_rate_lsb | kcps (raw) |
| 7 | sigma_msb | mm (raw) |
| 8 | sigma_lsb | mm (raw) |
| 9 | ambient_per_spad_msb | kcps (raw) |
| 10 | ambient_per_spad_lsb | kcps (raw) |
| 11 | signal_per_spad_msb | kcps (raw) |
| 12 | signal_per_spad_lsb | kcps (raw) |
| 13 | number_of_spad_msb | count (raw) |
| 14 | number_of_spad_lsb | count (raw) |

Notes:
- The firmware fills these bytes once the measurement completes.
- Before data is ready, the read buffer is typically 0xFF.

### 2) Set ranging timing (0x01)

**Write:**
- `01 <time_budget_ms> <inter_ms_msb> <inter_ms_lsb>`

**Behavior and limits:**
- Time budget is constrained to 10 to 200 ms.
- Inter-measurement period is constrained to 0 to 5000 ms.
- If inter-measurement <= time_budget + 7, it is forced to 0 (continuous).
- Values are saved to EEPROM and applied immediately.

### 3) Offset calibration (0x02)

**Write:**
- `02 <dist_msb> <dist_lsb> <samples_msb> <samples_lsb>`

**Parameters:**
- Target distance in mm: 10 to 1000
- Sample count: 5 to 255

**Sequence:**
1. Place target at known distance.
2. Write command with target distance and sample count.
3. Wait for calibration to finish.
4. The calibrated offset is saved in EEPROM.

### 4) XTalk calibration (0x03)

**Write:**
- `03 <dist_msb> <dist_lsb> <samples_msb> <samples_lsb>`

**Parameters:**
- Target distance in mm: 10 to 5000
- Sample count: 5 to 255

**Sequence:**
1. Place target at known distance.
2. Write command with target distance and sample count.
3. Wait for calibration to finish.
4. The calibrated crosstalk value is saved in EEPROM.

### 5) Read configuration (0x04)

**Write:**
- `04`

**Sequence:**
1. Host writes `04`.
2. Host reads 13 bytes from the slave.

**Read buffer (13 bytes):**

| Byte | Name | Notes |
| --- | --- | --- |
| 0 | i2c_address | current stored address |
| 1 | time_budget_ms | current time budget |
| 2 | inter_ms_msb | inter-measurement period |
| 3 | inter_ms_lsb | inter-measurement period |
| 4 | offset_msb | signed offset (mm) |
| 5 | offset_lsb | signed offset (mm) |
| 6 | xtalk_msb | crosstalk kcps |
| 7 | xtalk_lsb | crosstalk kcps |
| 8 | sigma_msb | sigma threshold (mm) |
| 9 | sigma_lsb | sigma threshold (mm) |
| 10 | signal_msb | signal threshold (kcps) |
| 11 | signal_lsb | signal threshold (kcps) |
| 12 | firmware_rev | firmware revision |

### 6) Restore defaults (0x05)

**Write:**
- `05`

**Behavior:**
- Resets EEPROM parameters (timing, offset, xtalk, sigma, signal) to factory defaults.
- Address byte is preserved.
- Parameters are applied immediately.

### 7) Sigma and signal thresholds (0x07)

**Write:**
- `07 <sigma_msb> <sigma_lsb> <signal_msb> <signal_lsb>`

**Behavior:**
- Updates sigma threshold and signal threshold in EEPROM.
- Applied immediately to the sensor.

### 8) Restart (0x08)

**Write:**
- `08`

**Behavior:**
- Reloads I2C address and all ranging parameters from EEPROM.
- Re-applies parameters to the sensor.

## Typical Host Sequences

### Basic ranging (mm)
1. Write: `00 52`
2. Delay >= time_budget + 10 ms
3. Read 15 bytes (ranging result)

### Change I2C address to 0x2A
1. Write: `00 A0`
2. Write: `00 AA`
3. Write: `00 A5`
4. Write: `00 2A`
5. Use 0x2A for all future commands

### Configure timing
1. Write: `01 <tb_ms> <im_ms_msb> <im_ms_lsb>`

### Get current configuration
1. Write: `04`
2. Read 13 bytes

### Run offset calibration (200 mm, 50 samples)
1. Write: `02 00 C8 00 32`
2. Delay for calibration to complete

### Run xtalk calibration (600 mm, 50 samples)
1. Write: `03 02 58 00 32`
2. Delay for calibration to complete

## Host-Side Example Code

### Arduino UNO (Wire library)

This example triggers a single measurement in millimeters and reads back the 15-byte result.

```cpp
#include <Wire.h>

static const uint8_t kI2cAddr = 0x70; // default device address

void i2cWriteBytes(uint8_t addr, const uint8_t *data, uint8_t len) {
   Wire.beginTransmission(addr);
   for (uint8_t i = 0; i < len; i++) {
      Wire.write(data[i]);
   }
   Wire.endTransmission();
}

bool i2cReadBytes(uint8_t addr, uint8_t *data, uint8_t len) {
   Wire.requestFrom(addr, len);
   uint8_t idx = 0;
   while (Wire.available() && idx < len) {
      data[idx++] = Wire.read();
   }
   return (idx == len);
}

void setup() {
   Wire.begin();
   Serial.begin(115200);
}

void loop() {
   // Command: single ranging in mm (0x52)
   uint8_t cmd[2] = {0x00, 0x52};
   i2cWriteBytes(kI2cAddr, cmd, sizeof(cmd));

   // Wait for measurement (adjust to your time budget)
   delay(60);

   // Read 15-byte result
   uint8_t buf[15] = {0};
   if (i2cReadBytes(kI2cAddr, buf, sizeof(buf))) {
      uint16_t distance = ((uint16_t)buf[0] << 8) | buf[1];
      uint8_t range_status = buf[2];
      Serial.print("Distance (mm): ");
      Serial.print(distance);
      Serial.print("  Status: ");
      Serial.println(range_status);
   } else {
      Serial.println("I2C read failed");
   }

   delay(250);
}
```

### Arduino UNO: Change I2C Address (sequence)

```cpp
void changeAddress(uint8_t oldAddr, uint8_t newAddr) {
   uint8_t s1[2] = {0x00, 0xA0};
   uint8_t s2[2] = {0x00, 0xAA};
   uint8_t s3[2] = {0x00, 0xA5};
   uint8_t s4[2] = {0x00, newAddr};

   i2cWriteBytes(oldAddr, s1, 2);
   i2cWriteBytes(oldAddr, s2, 2);
   i2cWriteBytes(oldAddr, s3, 2);
   i2cWriteBytes(oldAddr, s4, 2);
}
```

Notes:
- On Arduino UNO, `Wire` uses 7-bit addresses. Use the literal 0x70, 0x2A, etc.
- If you changed the address, update `kI2cAddr` for future commands.

### Python (smbus2)

This example triggers a single measurement in millimeters and reads back the 15-byte result.

```python
from smbus2 import SMBus
import time

I2C_ADDR = 0x70  # default device address

def i2c_write(cmd_bytes):
   with SMBus(1) as bus:
      bus.write_i2c_block_data(I2C_ADDR, cmd_bytes[0], cmd_bytes[1:])

def i2c_read(length):
   with SMBus(1) as bus:
      return bus.read_i2c_block_data(I2C_ADDR, 0x00, length)

def main():
   # Command: single ranging in mm (0x52)
   i2c_write([0x00, 0x52])
   time.sleep(0.06)  # wait for measurement (adjust to time budget)

   data = i2c_read(15)
   distance = (data[0] << 8) | data[1]
   range_status = data[2]
   print(f"Distance (mm): {distance}  Status: {range_status}")

if __name__ == "__main__":
   main()
```

Notes:
- On Linux, the I2C bus is typically `1` (e.g., `/dev/i2c-1`).
- On Windows, use a platform-specific I2C adapter and library (e.g., USB-I2C bridge) and adapt the calls accordingly.

### Generic C (embedded host)

This example uses placeholder functions for I2C write/read that you should replace with your platform's driver calls.

```c
#include <stdint.h>
#include <stdbool.h>

#define I2C_ADDR 0x70

// Replace these with your platform I2C functions
bool i2c_write(uint8_t addr, const uint8_t *data, uint8_t len);
bool i2c_read(uint8_t addr, uint8_t *data, uint8_t len);
void delay_ms(uint32_t ms);

bool tof_read_once_mm(uint16_t *distance_mm, uint8_t *range_status) {
   uint8_t cmd[2] = {0x00, 0x52};
   uint8_t buf[15] = {0};

   if (!i2c_write(I2C_ADDR, cmd, sizeof(cmd))) {
      return false;
   }

   delay_ms(60); // adjust to time budget

   if (!i2c_read(I2C_ADDR, buf, sizeof(buf))) {
      return false;
   }

   *distance_mm = ((uint16_t)buf[0] << 8) | buf[1];
   *range_status = buf[2];
   return true;
}
```

### STM32 HAL (I2C)

This example uses STM32 HAL I2C calls to trigger a single measurement and read the 15-byte result.

```c
#include "stm32f1xx_hal.h"

#define TOF_I2C_ADDR (0x70 << 1) // HAL expects 8-bit address

extern I2C_HandleTypeDef hi2c1;

bool tof_read_once_mm_stm32(uint16_t *distance_mm, uint8_t *range_status) {
   uint8_t cmd[2] = {0x00, 0x52};
   uint8_t buf[15] = {0};

   if (HAL_I2C_Master_Transmit(&hi2c1, TOF_I2C_ADDR, cmd, sizeof(cmd), 100) != HAL_OK) {
      return false;
   }

   HAL_Delay(60); // adjust to time budget

   if (HAL_I2C_Master_Receive(&hi2c1, TOF_I2C_ADDR, buf, sizeof(buf), 100) != HAL_OK) {
      return false;
   }

   *distance_mm = ((uint16_t)buf[0] << 8) | buf[1];
   *range_status = buf[2];
   return true;
}
```

### ESP32 (ESP-IDF)

This example uses ESP-IDF I2C master APIs.

```c
#include "driver/i2c.h"
#include <stdint.h>

#define I2C_PORT I2C_NUM_0
#define TOF_I2C_ADDR 0x70

static esp_err_t tof_write(const uint8_t *data, size_t len) {
   return i2c_master_write_to_device(I2C_PORT, TOF_I2C_ADDR, data, len, pdMS_TO_TICKS(100));
}

static esp_err_t tof_read(uint8_t *data, size_t len) {
   return i2c_master_read_from_device(I2C_PORT, TOF_I2C_ADDR, data, len, pdMS_TO_TICKS(100));
}

bool tof_read_once_mm_esp32(uint16_t *distance_mm, uint8_t *range_status) {
   uint8_t cmd[2] = {0x00, 0x52};
   uint8_t buf[15] = {0};

   if (tof_write(cmd, sizeof(cmd)) != ESP_OK) {
      return false;
   }

   vTaskDelay(pdMS_TO_TICKS(60)); // adjust to time budget

   if (tof_read(buf, sizeof(buf)) != ESP_OK) {
      return false;
   }

   *distance_mm = ((uint16_t)buf[0] << 8) | buf[1];
   *range_status = buf[2];
   return true;
}
```
