#include <Arduino.h>
#include <Wire.h>
#include <ctype.h>
#include <string.h>

// ============================================================================
// PSoC5 Host Emulation - Command-based I2C Interface
// ============================================================================

// Command definitions (matching PSoC5 protocol)
#define CMD_GET_RANGING_RESULT      0x00
#define CMD_SET_RANGING_TIMING      0x01
#define CMD_START_OFFSET_CAL        0x02
#define CMD_START_XTALK_CAL         0x03
#define CMD_GET_CONFIG              0x04
#define CMD_RESTORE_FACTORY_CONFIG  0x05
#define CMD_SAVE_CONFIG             0x06
#define CMD_SET_THRESHOLDS          0x07
#define CMD_RESTART                 0x08

// Unit definitions
#define MM    0x52
#define CM    0x51
#define INCH  0x50

// State machine states
enum SystemState {
	STATE_IDLE,
	STATE_CHANGE_TIMEBUDGET,
	STATE_CHANGE_INTERMEASUREMENT,
	STATE_CHANGEADDR,
	STATE_SELECT_UNIT_SINGLE,
	STATE_SELECT_UNIT_CONTINUOUS,
	STATE_RANGE_ONCE,
	STATE_RANGE_MULTI,
	STATE_OFFSET_CAL_DISTANCE,
	STATE_OFFSET_CAL_SAMPLES,
	STATE_XTALK_CAL_DISTANCE,
	STATE_XTALK_CAL_SAMPLES,
	STATE_SET_THRESHOLD_SIGMA,
	STATE_SET_THRESHOLD_SIGNAL
};

// Command structure (matches PSoC5 Command typedef)
typedef struct {
	uint8_t dev_address;
	uint8_t command_id;
	union {
		struct {
			uint8_t unitsbyte;
		} get_ranging;
		struct {
			uint8_t timebudget;
			uint16_t intermeasurementTime;
		} set_timing;
		struct {
			uint16_t cal_distance_mm;
			uint16_t samples_nbr;
		} offset_cal;
		struct {
			uint16_t cal_distance_mm;
			uint16_t samples_nbr;
		} xtalk_cal;
		struct {
			uint16_t sigma;
			uint16_t signal_threshold;
		} set_thresholds;
	} data;
} Command;

// Global variables
SystemState g_state = STATE_IDLE;
uint8_t g_dev_address = 0x29;  // Default VL53L4CD address
uint8_t g_selected_unit = MM;   // Default unit
uint16_t g_repeat_delay_ms = 0;
bool g_new_command = false;
char g_rx_buffer[256] = {0};
uint8_t g_rx_index = 0;

// UART RX interrupt handler
void serialEventRun(void) {
	while (Serial.available()) {
		char c = Serial.read();
		if (c == '\n') {
			g_rx_buffer[g_rx_index] = '\0';
			g_new_command = true;
			g_rx_index = 0;
		} else if (c != '\r' && g_rx_index < sizeof(g_rx_buffer) - 1) {
			g_rx_buffer[g_rx_index++] = c;
		}
	}
}

// ============================================================================
// I2C Communication Functions
// ============================================================================

// Write a raw command/data payload to the given I2C address.
static bool i2cWriteBytes(uint8_t addr, const uint8_t *data, uint8_t len) {
	Wire.beginTransmission(addr);
	for (uint8_t i = 0; i < len; ++i) {
		Wire.write(data[i]);
	}
	return (Wire.endTransmission() == 0);
}

// Read a fixed-length response from the given I2C address.
static bool i2cReadBytes(uint8_t addr, uint8_t *data, uint8_t len) {
	Wire.requestFrom(addr, len);
	uint8_t idx = 0;
	while (Wire.available() && idx < len) {
		data[idx++] = Wire.read();
	}
	return (idx == len);
}

// Auto-detect I2C slave address
static uint8_t detectI2cSlave(void) {
	for (uint8_t addr = 0x08; addr <= 0x7F; addr++) {
		Wire.beginTransmission(addr);
		if (Wire.endTransmission() == 0) {
			return addr;
		}
		delay(10);
	}
	return 0;
}

// Scan I2C addresses
static void scanI2cAddresses(void) {
	Serial.println(F("Scanning I2C bus (0x08 to 0x7F)..."));
	Serial.println(F("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F"));
	
	uint8_t found_count = 0;
	for (uint8_t address = 0; address <= 0x7F; address++) {
		if ((address & 0x0F) == 0) {
			Serial.print(address, HEX);
			Serial.print(F(": "));
		}
		
		if (address < 0x08) {
			Serial.print(F("   "));
		} else {
			Wire.beginTransmission(address);
			if (Wire.endTransmission() == 0) {
				Serial.print(address, HEX);
				Serial.print(F(" "));
				found_count++;
			} else {
				Serial.print(F("-- "));
			}
			delay(10);
		}
		
		if ((address & 0x0F) == 0x0F) {
			Serial.println();
		}
	}
	Serial.print(F("Scan complete. Found "));
	Serial.print(found_count);
	Serial.println(F(" device(s)."));
}

// ============================================================================
// Helper Functions
// ============================================================================

static uint16_t readU16Be(const uint8_t *buf, uint8_t msbIndex) {
	return static_cast<uint16_t>(buf[msbIndex] << 8) | buf[msbIndex + 1];
}

// Convert command structure to I2C bytes
static uint8_t commandToBytes(const Command* cmd, uint8_t* buffer, uint8_t buffer_size) {
	if (!cmd || !buffer || buffer_size < 1) {
		return 0;
	}
	
	uint8_t pos = 0;
	buffer[pos++] = cmd->command_id;
	
	switch (cmd->command_id) {
		case CMD_GET_RANGING_RESULT:
			if (buffer_size < pos + 1) return 0;
			buffer[pos++] = cmd->data.get_ranging.unitsbyte;
			break;
			
		case CMD_SET_RANGING_TIMING:
			if (buffer_size < pos + 3) return 0;
			buffer[pos++] = cmd->data.set_timing.timebudget;
			buffer[pos++] = (cmd->data.set_timing.intermeasurementTime >> 8) & 0xFF;
			buffer[pos++] = cmd->data.set_timing.intermeasurementTime & 0xFF;
			break;
			
		case CMD_START_OFFSET_CAL:
			if (buffer_size < pos + 4) return 0;
			buffer[pos++] = (cmd->data.offset_cal.cal_distance_mm >> 8) & 0xFF;
			buffer[pos++] = cmd->data.offset_cal.cal_distance_mm & 0xFF;
			buffer[pos++] = (cmd->data.offset_cal.samples_nbr >> 8) & 0xFF;
			buffer[pos++] = cmd->data.offset_cal.samples_nbr & 0xFF;
			break;
			
		case CMD_START_XTALK_CAL:
			if (buffer_size < pos + 4) return 0;
			buffer[pos++] = (cmd->data.xtalk_cal.cal_distance_mm >> 8) & 0xFF;
			buffer[pos++] = cmd->data.xtalk_cal.cal_distance_mm & 0xFF;
			buffer[pos++] = (cmd->data.xtalk_cal.samples_nbr >> 8) & 0xFF;
			buffer[pos++] = cmd->data.xtalk_cal.samples_nbr & 0xFF;
			break;
			
		case CMD_SET_THRESHOLDS:
			if (buffer_size < pos + 4) return 0;
			buffer[pos++] = (cmd->data.set_thresholds.sigma >> 8) & 0xFF;
			buffer[pos++] = cmd->data.set_thresholds.sigma & 0xFF;
			buffer[pos++] = (cmd->data.set_thresholds.signal_threshold >> 8) & 0xFF;
			buffer[pos++] = cmd->data.set_thresholds.signal_threshold & 0xFF;
			break;
			
		case CMD_SAVE_CONFIG:
		case CMD_RESTORE_FACTORY_CONFIG:
		case CMD_GET_CONFIG:
		case CMD_RESTART:
			break;
			
		default:
			return 0;
	}
	
	return pos;
}

// Send command via I2C
static bool sendCommandI2c(const Command* cmd) {
	uint8_t buffer[16];
	uint8_t length = commandToBytes(cmd, buffer, sizeof(buffer));
	if (length == 0) {
		return false;
	}
	return i2cWriteBytes(cmd->dev_address, buffer, length);
}

// Unit label helper
static const char* unitLabel(uint8_t unit) {
	switch (unit) {
		case MM:
			return "mm";
		case CM:
			return "cm";
		case INCH:
			return "inch";
		default:
			return "?";
	}
}

// Request config
static void requestConfig(void) {
	uint8_t cfg[13] = {0};
	uint8_t cmd = CMD_GET_CONFIG;
	
	if (!i2cWriteBytes(g_dev_address, &cmd, 1)) {
		Serial.println(F("Failed to request config"));
		return;
	}
	
	delay(10);
	if (!i2cReadBytes(g_dev_address, cfg, sizeof(cfg))) {
		Serial.println(F("Failed to read config"));
		return;
	}
	
	Serial.print(F("Device at 0x"));
	Serial.print(g_dev_address, HEX);
	Serial.println(F(" configuration:"));
	Serial.print(F("  Stored address: 0x"));
	Serial.println(cfg[0], HEX);
	Serial.print(F("  Time budget: "));
	Serial.println(cfg[1]);
	Serial.print(F("  Inter-measurement: "));
	Serial.println(readU16Be(cfg, 2));
	Serial.print(F("  Sigma threshold: "));
	Serial.println(readU16Be(cfg, 8));
	Serial.print(F("  Signal threshold: "));
	Serial.println(readU16Be(cfg, 10));
}

// Print command menu
static void printCommandMenu(void) {
	Serial.println(F("=== Available Commands ==="));
	Serial.println(F("0  - Single Ranging"));
	Serial.println(F("1  - Continuous Ranging"));
	Serial.println(F("2  - Set New Address"));
	Serial.println(F("3  - Set Ranging Timing"));
	Serial.println(F("4  - Start Offset Calibration"));
	Serial.println(F("5  - Start XTALK Calibration"));
	Serial.println(F("6  - Save Configuration"));
	Serial.println(F("7  - Restore Factory Configuration"));
	Serial.println(F("8  - Get EEPROM Data"));
	Serial.println(F("9  - Set Thresholds"));
	Serial.println(F("10 - Restart"));
	Serial.println(F("11 - Scan for Available I2C Devices"));
	Serial.print(F("Enter command number: "));
}

// Execute ranging command
static void executeRangingCommand(uint8_t dev_address, uint8_t units) {
	Command range_cmd;
	range_cmd.dev_address = dev_address;
	range_cmd.command_id = CMD_GET_RANGING_RESULT;
	range_cmd.data.get_ranging.unitsbyte = units;
	
	Serial.println(F("Executing range command..."));
	if (!sendCommandI2c(&range_cmd)) {
		Serial.println(F("Failed to send range command"));
		return;
	}
	
	uint8_t range_data[15] = {0};
	uint32_t attempts = 0;
	const uint32_t max_attempts = 100;
	
	while (attempts < max_attempts) {
		delay(10);
		if (i2cReadBytes(dev_address, range_data, sizeof(range_data))) {
			uint8_t status = range_data[2];
			if (status == 0x00) {
				uint16_t distance = readU16Be(range_data, 0);
				uint16_t signal = readU16Be(range_data, 3);
				uint16_t ambient = readU16Be(range_data, 5);
				uint16_t sigma = readU16Be(range_data, 7);
				
				Serial.print(F("Distance: "));
				Serial.print(distance);
				Serial.print(F(" | Signal: "));
				Serial.print(signal);
				Serial.print(F(" | Ambient: "));
				Serial.print(ambient);
				Serial.print(F(" | Sigma: "));
				Serial.println(sigma);
				return;
			}
		}
		attempts++;
	}
	
	Serial.println(F("Timeout: No valid data"));
}

// ============================================================================
// Setup and Loop (State Machine Implementation)
// ============================================================================

void setup() {
	Serial.begin(115200);
	delay(5000);  // Allow power-up
	Wire.begin();
	Wire.setClock(400000);
	
	// Auto-detect I2C slave
	Serial.println(F("Scanning for I2C slave device..."));
	uint8_t detected = detectI2cSlave();
	if (detected != 0) {
		g_dev_address = detected;
		Serial.print(F("Found I2C slave at address: 0x"));
		Serial.println(detected, HEX);
	} else {
		Serial.print(F("No I2C slave detected. Using default address: 0x"));
		Serial.println(g_dev_address, HEX);
	}
	
	g_state = STATE_IDLE;
	delay(500);
	printCommandMenu();
}

void loop() {
	// Process any incoming commands
	serialEventRun();
	
	switch (g_state) {
		case STATE_IDLE:
			if (g_new_command) {
				g_new_command = false;
				int cmd_choice = atoi(g_rx_buffer);
				
				switch (cmd_choice) {
					case 0:
						g_state = STATE_SELECT_UNIT_SINGLE;
						Serial.println(F("\nSelect unit:"));
						Serial.println(F("1) Millimeters (mm)"));
						Serial.println(F("2) Centimeters (cm)"));
						Serial.println(F("3) Inches (inch)"));
						Serial.print(F("Enter choice: "));
						break;
					case 1:
						g_state = STATE_SELECT_UNIT_CONTINUOUS;
						Serial.println(F("\nSelect unit:"));
						Serial.println(F("1) Millimeters (mm)"));
						Serial.println(F("2) Centimeters (cm)"));
						Serial.println(F("3) Inches (inch)"));
						Serial.print(F("Enter choice: "));
						break;
					case 2:
						g_state = STATE_CHANGEADDR;
						Serial.print(F("\nEnter new I2C address (hex, e.g. 0x29): "));
						break;
					case 3:
						g_state = STATE_CHANGE_TIMEBUDGET;
						Serial.print(F("\nEnter time budget (10-200 ms): "));
						break;
					case 4:
						g_state = STATE_OFFSET_CAL_DISTANCE;
						Serial.print(F("\nEnter calibration distance (10-1000 mm): "));
						break;
					case 5:
						g_state = STATE_XTALK_CAL_DISTANCE;
						Serial.print(F("\nEnter calibration distance (10-5000 mm): "));
						break;
					case 6: {
						Command save_cmd;
						save_cmd.dev_address = g_dev_address;
						save_cmd.command_id = CMD_SAVE_CONFIG;
						sendCommandI2c(&save_cmd);
						Serial.println(F("Configuration saved."));
						printCommandMenu();
						break;
					}
					case 7: {
						Command restore_cmd;
						restore_cmd.dev_address = g_dev_address;
						restore_cmd.command_id = CMD_RESTORE_FACTORY_CONFIG;
						sendCommandI2c(&restore_cmd);
						Serial.println(F("Factory configuration restored."));
						printCommandMenu();
						break;
					}
					case 8:
						requestConfig();
						printCommandMenu();
						break;
					case 9:
						g_state = STATE_SET_THRESHOLD_SIGMA;
						Serial.print(F("\nEnter sigma threshold (mm): "));
						break;
					case 10: {
						Command restart_cmd;
						restart_cmd.dev_address = g_dev_address;
						restart_cmd.command_id = CMD_RESTART;
						sendCommandI2c(&restart_cmd);
						Serial.println(F("Restart command sent."));
						printCommandMenu();
						break;
					}
					case 11:
						scanI2cAddresses();
						printCommandMenu();
						break;
					default:
						Serial.println(F("Unknown command."));
						printCommandMenu();
						break;
				}
			}
			break;
			
		case STATE_SELECT_UNIT_SINGLE:
			if (g_new_command) {
				g_new_command = false;
				int unit_choice = atoi(g_rx_buffer);
				switch (unit_choice) {
					case 1:
						g_selected_unit = MM;
						break;
					case 2:
						g_selected_unit = CM;
						break;
					case 3:
						g_selected_unit = INCH;
						break;
					default:
						Serial.println(F("Invalid choice."));
						g_state = STATE_IDLE;
						printCommandMenu();
						break;
				}
				if (unit_choice >= 1 && unit_choice <= 3) {
					g_state = STATE_RANGE_ONCE;
				}
			}
			break;
			
		case STATE_RANGE_ONCE:
			Serial.print(F("Ranging with unit: "));
			Serial.println(unitLabel(g_selected_unit));
			executeRangingCommand(g_dev_address, g_selected_unit);
			g_state = STATE_IDLE;
			printCommandMenu();
			break;
			
		case STATE_SELECT_UNIT_CONTINUOUS:
			if (g_new_command) {
				g_new_command = false;
				int unit_choice = atoi(g_rx_buffer);
				switch (unit_choice) {
					case 1:
						g_selected_unit = MM;
						break;
					case 2:
						g_selected_unit = CM;
						break;
					case 3:
						g_selected_unit = INCH;
						break;
					default:
						Serial.println(F("Invalid choice."));
						g_state = STATE_IDLE;
						printCommandMenu();
						break;
				}
				if (unit_choice >= 1 && unit_choice <= 3) {
					g_state = STATE_RANGE_MULTI;
					Serial.print(F("Enter repetition rate in ms: "));
				}
			}
			break;
			
		case STATE_RANGE_MULTI:
			if (g_new_command) {
				g_new_command = false;
				g_repeat_delay_ms = atoi(g_rx_buffer);
				Serial.print(F("Continuous ranging ("));
				Serial.print(g_repeat_delay_ms);
				Serial.println(F(" ms rate). Press 's' to stop."));
				
				unsigned long last_range_ms = 0;
				while (true) {
					serialEventRun();
					if (Serial.available()) {
						char c = Serial.read();
						if (c == 's' || c == 'S') {
							Serial.println(F("Continuous ranging stopped."));
							break;
						}
					}
					
					unsigned long now = millis();
					if (now - last_range_ms >= g_repeat_delay_ms) {
						executeRangingCommand(g_dev_address, g_selected_unit);
						last_range_ms = now;
					}
				}
				
				g_state = STATE_IDLE;
				printCommandMenu();
			}
			break;
			
		case STATE_CHANGEADDR:
			if (g_new_command) {
				g_new_command = false;
				uint8_t new_addr = strtol(g_rx_buffer, NULL, 16);
				Serial.print(F("Attempting to change address to: 0x"));
				Serial.println(new_addr, HEX);
				
				// Send unlock sequence
				uint8_t seq1[2] = {0x00, 0xA0};
				uint8_t seq2[2] = {0x00, 0xAA};
				uint8_t seq3[2] = {0x00, 0xA5};
				uint8_t seq4[2] = {0x00, new_addr};
				
				if (i2cWriteBytes(g_dev_address, seq1, 2) &&
					i2cWriteBytes(g_dev_address, seq2, 2) &&
					i2cWriteBytes(g_dev_address, seq3, 2) &&
					i2cWriteBytes(g_dev_address, seq4, 2)) {
					Serial.print(F("Address changed successfully to 0x"));
					Serial.println(new_addr, HEX);
					g_dev_address = new_addr;
				} else {
					Serial.println(F("Failed to change address."));
				}
				
				g_state = STATE_IDLE;
				printCommandMenu();
			}
			break;
			
		case STATE_CHANGE_TIMEBUDGET:
			if (g_new_command) {
				g_new_command = false;
				static uint8_t stored_timebudget = 0;
				stored_timebudget = atoi(g_rx_buffer);
				
				Serial.print(F("Enter inter-measurement time (0-5000 ms): "));
				g_state = STATE_CHANGE_INTERMEASUREMENT;
			}
			break;
			
		case STATE_CHANGE_INTERMEASUREMENT:
			if (g_new_command) {
				g_new_command = false;
				static uint8_t stored_timebudget = 0;
				uint16_t inter_time = atoi(g_rx_buffer);
				
				Command timing_cmd;
				timing_cmd.dev_address = g_dev_address;
				timing_cmd.command_id = CMD_SET_RANGING_TIMING;
				timing_cmd.data.set_timing.timebudget = stored_timebudget;
				timing_cmd.data.set_timing.intermeasurementTime = inter_time;
				
				if (sendCommandI2c(&timing_cmd)) {
					Serial.println(F("Timing updated."));
				} else {
					Serial.println(F("Failed to update timing."));
				}
				
				g_state = STATE_IDLE;
				printCommandMenu();
			}
			break;
			
		case STATE_OFFSET_CAL_DISTANCE:
			if (g_new_command) {
				g_new_command = false;
				static uint16_t stored_distance = 0;
				stored_distance = atoi(g_rx_buffer);
				Serial.print(F("Enter sample count (5-255): "));
				g_state = STATE_OFFSET_CAL_SAMPLES;
			}
			break;
			
		case STATE_OFFSET_CAL_SAMPLES:
			if (g_new_command) {
				g_new_command = false;
				static uint16_t stored_distance = 0;
				uint16_t samples = atoi(g_rx_buffer);
				
				Command offset_cmd;
				offset_cmd.dev_address = g_dev_address;
				offset_cmd.command_id = CMD_START_OFFSET_CAL;
				offset_cmd.data.offset_cal.cal_distance_mm = stored_distance;
				offset_cmd.data.offset_cal.samples_nbr = samples;
				
				if (sendCommandI2c(&offset_cmd)) {
					Serial.println(F("Offset calibration started."));
				} else {
					Serial.println(F("Failed to start offset calibration."));
				}
				
				g_state = STATE_IDLE;
				printCommandMenu();
			}
			break;
			
		case STATE_XTALK_CAL_DISTANCE:
			if (g_new_command) {
				g_new_command = false;
				static uint16_t stored_distance = 0;
				stored_distance = atoi(g_rx_buffer);
				Serial.print(F("Enter sample count (5-255): "));
				g_state = STATE_XTALK_CAL_SAMPLES;
			}
			break;
			
		case STATE_XTALK_CAL_SAMPLES:
			if (g_new_command) {
				g_new_command = false;
				static uint16_t stored_distance = 0;
				uint16_t samples = atoi(g_rx_buffer);
				
				Command xtalk_cmd;
				xtalk_cmd.dev_address = g_dev_address;
				xtalk_cmd.command_id = CMD_START_XTALK_CAL;
				xtalk_cmd.data.xtalk_cal.cal_distance_mm = stored_distance;
				xtalk_cmd.data.xtalk_cal.samples_nbr = samples;
				
				if (sendCommandI2c(&xtalk_cmd)) {
					Serial.println(F("XTALK calibration started."));
				} else {
					Serial.println(F("Failed to start XTALK calibration."));
				}
				
				g_state = STATE_IDLE;
				printCommandMenu();
			}
			break;
			
		case STATE_SET_THRESHOLD_SIGMA:
			if (g_new_command) {
				g_new_command = false;
				static uint16_t stored_sigma = 0;
				stored_sigma = atoi(g_rx_buffer);
				Serial.print(F("Enter signal threshold (kcps): "));
				g_state = STATE_SET_THRESHOLD_SIGNAL;
			}
			break;
			
		case STATE_SET_THRESHOLD_SIGNAL:
			if (g_new_command) {
				g_new_command = false;
				static uint16_t stored_sigma = 0;
				uint16_t signal = atoi(g_rx_buffer);
				
				Command threshold_cmd;
				threshold_cmd.dev_address = g_dev_address;
				threshold_cmd.command_id = CMD_SET_THRESHOLDS;
				threshold_cmd.data.set_thresholds.sigma = stored_sigma;
				threshold_cmd.data.set_thresholds.signal_threshold = signal;
				
				if (sendCommandI2c(&threshold_cmd)) {
					Serial.println(F("Thresholds updated."));
				} else {
					Serial.println(F("Failed to update thresholds."));
				}
				
				g_state = STATE_IDLE;
				printCommandMenu();
			}
			break;
	}
}
