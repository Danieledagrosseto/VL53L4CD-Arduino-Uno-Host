// Uncomment to enable interactive setup/configuration mode
// Comment out (or leave undefined) to enable autonomous ranging mode
// #define SETUP_RANGING

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
	STATE_SET_THRESHOLD_SIGNAL,
	STATE_RANGE_ALL_UNITS,
	STATE_RANGE_ALL_EXEC,
	STATE_RANGE_ALL_CONTINUOUS_UNITS,
	STATE_RANGE_ALL_CONTINUOUS_RATE
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
uint8_t g_dev_address = 0x29;  // Current device address
uint8_t g_selected_unit = MM;   // Default unit
uint16_t g_repeat_delay_ms = 0;
bool g_new_command = false;
char g_rx_buffer[256] = {0};
uint8_t g_rx_index = 0;

// Intermediate state variables (used across multiple state transitions)
uint8_t g_temp_timebudget = 0;
uint16_t g_temp_distance = 0;
uint16_t g_temp_sigma = 0;

// Timing tracking for non-blocking delays
unsigned long g_delay_start_ms = 0;
uint16_t g_delay_duration_ms = 0;
bool g_delay_active = false;

// I2C device detection
#define MAX_I2C_DEVICES 10
uint8_t g_detected_devices[MAX_I2C_DEVICES] = {0};
uint8_t g_num_devices = 0;

// Cached ranging parameters for each detected device (setup mode)
typedef struct {
	bool valid;
	uint8_t address;
	uint8_t time_budget_ms;
	uint16_t intermeasurement_ms;
	int16_t offset_mm;
	uint16_t xtalk_kcps;
	uint16_t sigma_threshold_mm;
	uint16_t signal_threshold_kcps;
	uint8_t firmware_rev;
} SetupRangingParams;

SetupRangingParams g_saved_ranging_params[MAX_I2C_DEVICES] = {0};
uint8_t g_saved_ranging_params_count = 0;

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
// Non-blocking Delay Functions (using millis())
// ============================================================================

// Start a non-blocking delay
static void startDelay(uint16_t duration_ms) {
	g_delay_start_ms = millis();
	g_delay_duration_ms = duration_ms;
	g_delay_active = true;
}

// Check if delay has completed (call repeatedly in loop)
static bool isDelayComplete(void) {
	if (!g_delay_active) {
		return true;
	}
	if (millis() - g_delay_start_ms >= g_delay_duration_ms) {
		g_delay_active = false;
		return true;
	}
	return false;
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
		unsigned long start = millis();
		while (millis() - start < 10) {
			serialEventRun();
		}
	}
	return 0;
}

// Scan I2C addresses and store detected devices
static void scanI2cAddresses(void) {
	Serial.println(F("Scanning I2C bus (0x08 to 0x7F)..."));
	Serial.println(F("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F"));
	
	g_num_devices = 0;
	memset(g_detected_devices, 0, sizeof(g_detected_devices));
	
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
				if (g_num_devices < MAX_I2C_DEVICES) {
					g_detected_devices[g_num_devices++] = address;
				}
				found_count++;
			} else {
				Serial.print(F("-- "));
			}
			unsigned long start = millis();
			while (millis() - start < 10) {
				serialEventRun();
			}
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

// Clear any buffered serial input and pending command state.
static void clearPendingSerialInput(void) {
	while (Serial.available()) {
		Serial.read();
	}
	g_new_command = false;
	g_rx_index = 0;
	g_rx_buffer[0] = '\0';
}

// List detected devices for selection
static bool selectDeviceAddress(uint8_t *addr_out) {
	if (g_num_devices == 0) {
		Serial.println(F("No I2C devices detected."));
		return false;
	}
	
	Serial.println(F("Available I2C devices:"));
	for (uint8_t i = 0; i < g_num_devices; i++) {
		Serial.print(F("["));
		Serial.print(i);
		Serial.print(F("] 0x"));
		Serial.println(g_detected_devices[i], HEX);
	}
	Serial.print(F("Select device (0-"));
	Serial.print(g_num_devices - 1);
	Serial.print(F("): "));
	
	char buf[16] = {0};
	if (readLine(buf, sizeof(buf)) == 0) {
		clearPendingSerialInput();
		return false;
	}
	
	uint16_t choice = 0;
	if (!parseU16(buf, 0, g_num_devices - 1, &choice)) {
		Serial.println(F("Invalid choice."));
		clearPendingSerialInput();
		return false;
	}
	
	*addr_out = g_detected_devices[choice];
	clearPendingSerialInput();
	return true;
}

// Request config from device
static void requestConfig(uint8_t dev_addr) {
	uint8_t cfg[13] = {0};
	uint8_t cmd = CMD_GET_CONFIG;
	
	if (!i2cWriteBytes(dev_addr, &cmd, 1)) {
		Serial.println(F("Failed to request config"));
		return;
	}
	
	// Give device time to prepare config data
	unsigned long start = millis();
	while (millis() - start < 50) {
		serialEventRun();
	}
	if (!i2cReadBytes(dev_addr, cfg, sizeof(cfg))) {
		Serial.println(F("Failed to read config"));
		return;
	}
	
	Serial.print(F("Device at 0x"));
	Serial.print(dev_addr, HEX);
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
	Serial.println(F("12 - Range All Devices Simultaneously"));
	Serial.println(F("13 - Continuous Range All Devices"));
	Serial.print(F("Enter command number: "));
}

// Get selected device time budget in ms from saved params.
static uint8_t getSelectedDeviceTimeBudgetMs(uint8_t dev_address) {
	const uint8_t fallback_time_budget_ms = 20;

	for (uint8_t i = 0; i < g_num_devices; ++i) {
		if (g_saved_ranging_params[i].valid && g_saved_ranging_params[i].address == dev_address) {
			return (g_saved_ranging_params[i].time_budget_ms == 0) ? fallback_time_budget_ms : g_saved_ranging_params[i].time_budget_ms;
		}
	}

	return fallback_time_budget_ms;
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
	
	// Wait exactly the selected device time budget before reading result.
	uint8_t time_budget_ms = getSelectedDeviceTimeBudgetMs(dev_address);
	unsigned long start = millis();
	while (millis() - start < time_budget_ms) {
		serialEventRun();
	}
	uint8_t range_data[15] = {0};
	uint32_t attempts = 0;
	const uint32_t max_attempts = 4;  // Total of 40ms max wait (4 attempts * 10ms delay)
	
	while (attempts < max_attempts) 	{
		unsigned long loop_start = millis();
		while (millis() - loop_start < 10) {
			serialEventRun();
		}
		if (i2cReadBytes(dev_address, range_data, sizeof(range_data))) {
			uint8_t status = range_data[2];
			if (status <= 12) {
				uint16_t raw_distance = readU16Be(range_data, 0);
				uint16_t distance = resolveDistance(raw_distance, status);
				uint16_t signal = readU16Be(range_data, 3);
				uint16_t ambient = readU16Be(range_data, 5);
				uint16_t sigma = readU16Be(range_data, 7);
				
				Serial.print(F("Distance: "));
				Serial.print(distance);
				Serial.print(F(" | Status: "));
				Serial.print(status);
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
	
	const uint8_t timeout_status = 255;
	uint16_t distance = resolveDistance(0, timeout_status);
	Serial.print(F("Distance: "));
	Serial.print(distance);
	Serial.print(F(" | Status: "));
	Serial.print(timeout_status);
	Serial.println(F(" | Timeout"));
}

// Execute ranging command on all detected devices simultaneously
static void executeRangingCommandAllDevices(uint8_t units) {
	if (g_num_devices == 0) {
		Serial.println(F("No devices detected"));
		return;
	}
	
	Serial.print(F("Sending range command to all "));
	Serial.print(g_num_devices);
	Serial.println(F(" device(s)..."));
	
	// Send command to all devices simultaneously
	uint8_t success_count = 0;
	for (uint8_t i = 0; i < g_num_devices; i++) {
		Command range_cmd;
		range_cmd.dev_address = g_detected_devices[i];
		range_cmd.command_id = CMD_GET_RANGING_RESULT;
		range_cmd.data.get_ranging.unitsbyte = units;
		
		if (sendCommandI2c(&range_cmd)) {
			success_count++;
		} else {
			Serial.print(F("Failed to send range command to 0x"));
			Serial.println(g_detected_devices[i], HEX);
		}
	}
	
	if (success_count == 0) {
		Serial.println(F("Failed to send command to any device"));
		return;
	}
	
	// Wait for the longest device time budget across all detected devices
	uint16_t max_budget_ms = 0;
	for (uint8_t i = 0; i < g_num_devices; i++) {
		uint16_t budget = getSelectedDeviceTimeBudgetMs(g_detected_devices[i]);
		if (budget > max_budget_ms) max_budget_ms = budget;
	}
	unsigned long start = millis();
	while (millis() - start < max_budget_ms) {
		serialEventRun();
	}
	
	// Track which devices have valid data
	bool data_valid[MAX_I2C_DEVICES] = {false};
	uint8_t device_data[MAX_I2C_DEVICES][15];
	memset(device_data, 0, sizeof(device_data));
	
	// Poll all devices until all have valid data
	uint32_t attempts = 0;
	const uint32_t max_attempts = 4;  // Total of 40ms max wait (4 attempts * 10ms delay)
	
	while (attempts < max_attempts) {
		bool all_valid = true;
		for (uint8_t i = 0; i < g_num_devices; i++) {
			if (!data_valid[i]) {
				if (i2cReadBytes(g_detected_devices[i], device_data[i], 15)) {
					uint8_t status = device_data[i][2];
					if (status <= 12) {
						data_valid[i] = true;
					}
				}
				all_valid = false;
			}
		}
		
		// Check if all devices now have valid data
		if (all_valid) {
			break;
		}
		
		unsigned long loop_start = millis();
		while (millis() - loop_start < 10) {
			serialEventRun();
		}
		attempts++;
	}
	
	// Display results for all devices
	Serial.println(F("\n=== Ranging Results (All Devices) ==="));
	for (uint8_t i = 0; i < g_num_devices; i++) {
		Serial.print(F("Device 0x"));
		Serial.print(g_detected_devices[i], HEX);
		Serial.print(F(": "));
		
		if (data_valid[i]) {
			uint8_t status = device_data[i][2];
			uint16_t raw_distance = readU16Be(device_data[i], 0);
			uint16_t distance = resolveDistance(raw_distance, status);
			uint16_t signal = readU16Be(device_data[i], 3);
			uint16_t ambient = readU16Be(device_data[i], 5);
			uint16_t sigma = readU16Be(device_data[i], 7);
			
			Serial.print(distance);
			Serial.print(F(" "));
			Serial.print(unitLabel(units));
			Serial.print(F(" | Status: "));
			Serial.print(status);
			Serial.print(F(" | Signal: "));
			Serial.print(signal);
			Serial.print(F(" | Ambient: "));
			Serial.print(ambient);
			Serial.print(F(" | Sigma: "));
			Serial.println(sigma);
		} else {
			const uint8_t timeout_status = 255;
			uint16_t distance = resolveDistance(0, timeout_status);
			Serial.print(distance);
			Serial.print(F(" "));
			Serial.print(unitLabel(units));
			Serial.print(F(" | Status: "));
			Serial.print(timeout_status);
			Serial.println(F(" | Timeout"));
		}
	}
	Serial.println();
}

// Execute continuous ranging on all detected devices simultaneously
static void executeRangingCommandAllDevicesContinuous(uint8_t units, uint16_t repeat_rate_ms) {
	if (g_num_devices == 0) {
		Serial.println(F("No devices detected"));
		return;
	}
	
	Serial.print(F("Continuous ranging on all "));
	Serial.print(g_num_devices);
	Serial.print(F(" device(s) at "));
	Serial.print(repeat_rate_ms);
	Serial.println(F(" ms rate. Press 's' to stop."));
	
	unsigned long last_range_ms = 0;
	
	while (true) {
		serialEventRun();
		
		// Check for stop command
		if (g_new_command) {
			g_new_command = false;
			if ((g_rx_buffer[0] == 's' || g_rx_buffer[0] == 'S') && g_rx_buffer[1] == '\0') {
				Serial.println(F("Continuous ranging stopped."));
				break;
			}
		}
		if (Serial.available()) {
			char c = Serial.read();
			if (c == 's' || c == 'S') {
				Serial.println(F("Continuous ranging stopped."));
				break;
			}
		}
		
		unsigned long now = millis();
		if (now - last_range_ms >= repeat_rate_ms) {
			// Send command to all devices
			for (uint8_t i = 0; i < g_num_devices; i++) {
				Command range_cmd;
				range_cmd.dev_address = g_detected_devices[i];
				range_cmd.command_id = CMD_GET_RANGING_RESULT;
				range_cmd.data.get_ranging.unitsbyte = units;
				sendCommandI2c(&range_cmd);
			}
			
			// Wait for the longest device time budget across all detected devices
			uint16_t max_budget_ms = 0;
			for (uint8_t i = 0; i < g_num_devices; i++) {
				uint16_t budget = getSelectedDeviceTimeBudgetMs(g_detected_devices[i]);
				if (budget > max_budget_ms) max_budget_ms = budget;
			}
			unsigned long cmd_start = millis();
			while (millis() - cmd_start < max_budget_ms) {
				serialEventRun();
			}
			
			// Track which devices have valid data
			bool data_valid[MAX_I2C_DEVICES] = {false};
			uint8_t device_data[MAX_I2C_DEVICES][15];
			memset(device_data, 0, sizeof(device_data));
			
			// Poll all devices until all have valid data or timeout
			uint32_t attempts = 0;
			const uint32_t max_attempts = 200;  // Increased to 2 seconds total for continuous mode
			
			while (attempts < max_attempts) {
				bool all_valid = true;
				for (uint8_t i = 0; i < g_num_devices; i++) {
					if (!data_valid[i]) {
						if (i2cReadBytes(g_detected_devices[i], device_data[i], 15)) {
							uint8_t status = device_data[i][2];
							if (status <= 12) {
								data_valid[i] = true;
							}
						}
						all_valid = false;
					}
				}
				
				if (all_valid) {
					break;
				}
				
				unsigned long poll_start = millis();
				while (millis() - poll_start < 10) {
					serialEventRun();
				}
				attempts++;
			}
			
			// Display results for all devices
			Serial.println(F("---"));
			for (uint8_t i = 0; i < g_num_devices; i++) {
				Serial.print(F("0x"));
				Serial.print(g_detected_devices[i], HEX);
				Serial.print(F(": "));
				
				if (data_valid[i]) {
					uint8_t status = device_data[i][2];
					uint16_t raw_distance = readU16Be(device_data[i], 0);
					uint16_t distance = resolveDistance(raw_distance, status);
					uint16_t signal = readU16Be(device_data[i], 3);
					uint16_t ambient = readU16Be(device_data[i], 5);
					uint16_t sigma = readU16Be(device_data[i], 7);
					
					Serial.print(distance);
					Serial.print(F(" "));
					Serial.print(unitLabel(units));
					Serial.print(F(" | St: "));
					Serial.print(status);
					Serial.print(F(" | Sig: "));
					Serial.print(signal);
					Serial.print(F(" | Amb: "));
					Serial.print(ambient);
					Serial.print(F(" | Sig: "));
					Serial.println(sigma);
				} else {
					const uint8_t timeout_status = 255;
					uint16_t distance = resolveDistance(0, timeout_status);
					Serial.print(distance);
					Serial.print(F(" "));
					Serial.print(unitLabel(units));
					Serial.print(F(" | St: "));
					Serial.print(timeout_status);
					Serial.println(F(" | Timeout"));
				}
			}
			
			last_range_ms = now;
		}
	}
}

// Helper: Read a line from serial (waits up to 30 seconds for input)
// Relies on serialEventRun() to populate g_rx_buffer
static uint8_t readLine(char *buffer, uint8_t max_len) {
	unsigned long timeout = millis() + 30000;  // 30 second timeout
	
	while (millis() < timeout) {
		serialEventRun();
		if (g_new_command) {
			g_new_command = false;
			uint8_t len = strlen(g_rx_buffer);
			strncpy(buffer, g_rx_buffer, max_len - 1);
			buffer[max_len - 1] = '\0';
			g_rx_buffer[0] = '\0';
			g_rx_index = 0;
			return len;
		}
	}
	
	buffer[0] = '\0';
	return 0;
}

// Helper: Parse unsigned 16-bit from string with bounds check
static bool parseU16(const char *str, uint16_t min_val, uint16_t max_val, uint16_t *out) {
	if (str == NULL || out == NULL) {
		return false;
	}
	
	uint32_t val = 0;
	for (int i = 0; str[i] != '\0'; i++) {
		if (str[i] < '0' || str[i] > '9') {
			return false;
		}
		val = (val * 10) + (str[i] - '0');
		if (val > 65535) {
			return false;
		}
	}
	
	if (val < min_val || val > max_val) {
		return false;
	}
	
	*out = (uint16_t)val;
	return true;
}

// Read a big-endian signed 16-bit value from a byte buffer.
static int16_t readS16Be(const uint8_t *buf, uint8_t msbIndex) {
	return static_cast<int16_t>(readU16Be(buf, msbIndex));
}

// Maximum measurable distance reported for out-of-range errors.
static const uint16_t RANGING_MAX_DISTANCE_MM = 1300;

// Map a range-status byte to an effective distance.
static uint16_t resolveDistance(uint16_t rawDistance, uint8_t status) {
	switch (status) {
		case 0: case 1: case 2: case 6:
			return rawDistance;
		case 3:
			return 0;
		case 4: case 7: case 12:
			return RANGING_MAX_DISTANCE_MM;
		default:
			return 0xFFFF;
	}
}

// Request the sensor configuration block and read it into cfg (13 bytes).
static bool readConfig(uint8_t addr, uint8_t *cfg, uint8_t len) {
	const uint8_t cmd = CMD_GET_CONFIG;
	if (!i2cWriteBytes(addr, &cmd, 1)) {
		return false;
	}
	unsigned long start = millis();
	while (millis() - start < 50) {
		serialEventRun();
	}
	return i2cReadBytes(addr, cfg, len);
}

// Read and store ranging parameters for every detected device.
static void saveDetectedDeviceRangingParameters(void) {
	g_saved_ranging_params_count = 0;
	memset(g_saved_ranging_params, 0, sizeof(g_saved_ranging_params));

	if (g_num_devices == 0) {
		Serial.println(F("No detected devices. Skipping ranging parameter save."));
		return;
	}

	Serial.println(F("Reading ranging parameters for detected devices..."));
	for (uint8_t i = 0; i < g_num_devices; ++i) {
		uint8_t cfg[13] = {0};
		const uint8_t addr = g_detected_devices[i];
		if (!readConfig(addr, cfg, sizeof(cfg))) {
			Serial.print(F("Failed to read parameters from 0x"));
			Serial.println(addr, HEX);
			continue;
		}

		SetupRangingParams &params = g_saved_ranging_params[i];
		params.valid = true;
		params.address = addr;
		params.time_budget_ms = cfg[1];
		params.intermeasurement_ms = readU16Be(cfg, 2);
		params.offset_mm = static_cast<int16_t>(readU16Be(cfg, 4));
		params.xtalk_kcps = readU16Be(cfg, 6);
		params.sigma_threshold_mm = readU16Be(cfg, 8);
		params.signal_threshold_kcps = readU16Be(cfg, 10);
		params.firmware_rev = cfg[12];
		g_saved_ranging_params_count++;

		Serial.print(F("Saved ranging parameters for 0x"));
		Serial.println(addr, HEX);
	}

	Serial.print(F("Saved parameter blocks: "));
	Serial.println(g_saved_ranging_params_count);
}

static void printCell(const char *text, uint8_t width) {
	if (text == NULL) {
		text = "";
	}
	Serial.print(text);
	uint8_t len = strlen(text);
	while (len < width) {
		Serial.print(' ');
		len++;
	}
}

// Print a table of cached ranging parameters for detected devices.
static void printSavedRangingParametersTable(void) {
	if (g_num_devices == 0) {
		Serial.println(F("No devices detected. Parameter table is empty."));
		return;
	}

	Serial.println(F("\n=== Device Ranging Parameters ==="));
	Serial.println(F("Addr  TB    IM(ms)  Offset   XTALK   Sigma   Signal  FW"));
	Serial.println(F("----  ----  ------  -------  ------  ------  ------  --"));

	for (uint8_t i = 0; i < g_num_devices; ++i) {
		const SetupRangingParams &params = g_saved_ranging_params[i];
		char cell[16] = {0};
		snprintf(cell, sizeof(cell), "0x%02X", g_detected_devices[i]);
		printCell(cell, 6);

		if (!params.valid) {
			Serial.println(F("READ_FAIL"));
			continue;
		}

		snprintf(cell, sizeof(cell), "%u", params.time_budget_ms);
		printCell(cell, 6);
		snprintf(cell, sizeof(cell), "%u", params.intermeasurement_ms);
		printCell(cell, 8);
		snprintf(cell, sizeof(cell), "%d", params.offset_mm);
		printCell(cell, 9);
		snprintf(cell, sizeof(cell), "%u", params.xtalk_kcps);
		printCell(cell, 8);
		snprintf(cell, sizeof(cell), "%u", params.sigma_threshold_mm);
		printCell(cell, 8);
		snprintf(cell, sizeof(cell), "%u", params.signal_threshold_kcps);
		printCell(cell, 8);
		snprintf(cell, sizeof(cell), "%u", params.firmware_rev);
		Serial.println(cell);
	}

	Serial.println(F("----------------------------------------------------------"));
}
#define SETUP_RANGING
#ifdef SETUP_RANGING
// ============================================================================
// Setup and Loop (State Machine Implementation)
// ============================================================================

void setup() {
	Serial.begin(115200);
	unsigned long power_up_start = millis();
	while (millis() - power_up_start < 5000) {
		serialEventRun();
	}
	Wire.begin();
	Wire.setClock(400000);
	
	// Scan for all I2C devices
	Serial.println(F("Scanning for I2C slave devices..."));
	scanI2cAddresses();
	saveDetectedDeviceRangingParameters();
	printSavedRangingParametersTable();
	if (g_num_devices > 0) {
		g_dev_address = g_detected_devices[0];
		Serial.print(F("Default device set to: 0x"));
		Serial.println(g_dev_address, HEX);
	} else {
		Serial.println(F("No I2C devices detected. Please connect VL53L4CD."));
	}
	
	g_state = STATE_IDLE;
	unsigned long menu_start = millis();
	while (millis() - menu_start < 500) {
		serialEventRun();
	}
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
						if (!selectDeviceAddress(&g_dev_address)) {
							printCommandMenu();
							break;
						}
						g_state = STATE_SELECT_UNIT_SINGLE;
						Serial.println(F("\nSelect unit:"));
						Serial.println(F("1) Millimeters (mm)"));
						Serial.println(F("2) Centimeters (cm)"));
						Serial.println(F("3) Inches (inch)"));
						Serial.print(F("Enter choice: "));
						break;
					case 1:
						if (!selectDeviceAddress(&g_dev_address)) {
							printCommandMenu();
							break;
						}
						g_state = STATE_SELECT_UNIT_CONTINUOUS;
						Serial.println(F("\nSelect unit:"));
						Serial.println(F("1) Millimeters (mm)"));
						Serial.println(F("2) Centimeters (cm)"));
						Serial.println(F("3) Inches (inch)"));
						Serial.print(F("Enter choice: "));
						break;
					case 2:
						if (!selectDeviceAddress(&g_dev_address)) {
							printCommandMenu();
							break;
						}
						g_state = STATE_CHANGEADDR;
						Serial.print(F("\nEnter new I2C address (hex, e.g. 0x29): "));
						break;
					case 3:
						if (!selectDeviceAddress(&g_dev_address)) {
							printCommandMenu();
							break;
						}
						g_state = STATE_CHANGE_TIMEBUDGET;
						Serial.print(F("\nEnter time budget (10-200 ms): "));
						break;
					case 4:
						if (!selectDeviceAddress(&g_dev_address)) {
							printCommandMenu();
							break;
						}
						g_state = STATE_OFFSET_CAL_DISTANCE;
						Serial.print(F("\nEnter calibration distance (10-1000 mm): "));
						break;
					case 5:
						if (!selectDeviceAddress(&g_dev_address)) {
							printCommandMenu();
							break;
						}
						g_state = STATE_XTALK_CAL_DISTANCE;
						Serial.print(F("\nEnter calibration distance (10-5000 mm): "));
						break;
					case 6: {
						if (!selectDeviceAddress(&g_dev_address)) {
							printCommandMenu();
							break;
						}
						Command save_cmd;
						save_cmd.dev_address = g_dev_address;
						save_cmd.command_id = CMD_SAVE_CONFIG;
						sendCommandI2c(&save_cmd);
						Serial.println(F("Configuration saved."));
						printCommandMenu();
						break;
					}
					case 7: {
						if (!selectDeviceAddress(&g_dev_address)) {
							printCommandMenu();
							break;
						}
						Command restore_cmd;
						restore_cmd.dev_address = g_dev_address;
						restore_cmd.command_id = CMD_RESTORE_FACTORY_CONFIG;
						sendCommandI2c(&restore_cmd);
						Serial.println(F("Factory configuration restored."));
						printCommandMenu();
						break;
					}
					case 8:
						if (!selectDeviceAddress(&g_dev_address)) {
							printCommandMenu();
							break;
						}
						requestConfig(g_dev_address);
						printCommandMenu();
						break;
					case 9:
						if (!selectDeviceAddress(&g_dev_address)) {
							printCommandMenu();
							break;
						}
						g_state = STATE_SET_THRESHOLD_SIGMA;
						Serial.print(F("\nEnter sigma threshold (mm): "));
						break;
					case 10: {
						if (!selectDeviceAddress(&g_dev_address)) {
							printCommandMenu();
							break;
						}
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
					case 12:
						g_state = STATE_RANGE_ALL_UNITS;
						Serial.println(F("\nSelect unit:"));
						Serial.println(F("1) Millimeters (mm)"));
						Serial.println(F("2) Centimeters (cm)"));
						Serial.println(F("3) Inches (inch)"));
						Serial.print(F("Enter choice: "));
						break;
					case 13:
						g_state = STATE_RANGE_ALL_CONTINUOUS_UNITS;
						Serial.println(F("\nSelect unit:"));
						Serial.println(F("1) Millimeters (mm)"));
						Serial.println(F("2) Centimeters (cm)"));
						Serial.println(F("3) Inches (inch)"));
						Serial.print(F("Enter choice: "));
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
					if (g_new_command) {
						g_new_command = false;
						if ((g_rx_buffer[0] == 's' || g_rx_buffer[0] == 'S') && g_rx_buffer[1] == '\0') {
							Serial.println(F("Continuous ranging stopped."));
							break;
						}
					}
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
					
					// Re-scan I2C bus to update device list
					Serial.println(F("\nRe-scanning I2C bus to update device list..."));
					scanI2cAddresses();
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
				g_temp_timebudget = atoi(g_rx_buffer);
				
				Serial.print(F("Enter inter-measurement time (0-5000 ms): "));
				g_state = STATE_CHANGE_INTERMEASUREMENT;
			}
			break;
			
		case STATE_CHANGE_INTERMEASUREMENT:
			if (g_new_command) {
				g_new_command = false;
				uint16_t inter_time = atoi(g_rx_buffer);
				
				Serial.print(F("Debug: timebudget="));
				Serial.print(g_temp_timebudget);
				Serial.print(F(", inter_time="));
				Serial.println(inter_time);
				
				Command timing_cmd;
				timing_cmd.dev_address = g_dev_address;
				timing_cmd.command_id = CMD_SET_RANGING_TIMING;
				timing_cmd.data.set_timing.timebudget = g_temp_timebudget;
				timing_cmd.data.set_timing.intermeasurementTime = inter_time;
				
				if (sendCommandI2c(&timing_cmd)) {
					Serial.println(F("Timing updated."));
					
					// Give device time to process timing change
					unsigned long timing_start = millis();
					while (millis() - timing_start < 100) {
						serialEventRun();
					}
					
					Command save_cmd;
					save_cmd.dev_address = g_dev_address;
					save_cmd.command_id = CMD_SAVE_CONFIG;
					if (sendCommandI2c(&save_cmd)) {
						Serial.println(F("Configuration saved."));
					} else {
						Serial.println(F("Failed to save configuration."));
					}
					
					// Give device time to save to EEPROM
					unsigned long save_start = millis();
					while (millis() - save_start < 100) {
						serialEventRun();
					}
					
					Command restart_cmd;
					restart_cmd.dev_address = g_dev_address;
					restart_cmd.command_id = CMD_RESTART;
					if (sendCommandI2c(&restart_cmd)) {
						Serial.println(F("Restart command sent."));
					} else {
						Serial.println(F("Failed to send restart command."));
					}
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
				g_temp_distance = atoi(g_rx_buffer);
				Serial.print(F("Enter sample count (5-255): "));
				g_state = STATE_OFFSET_CAL_SAMPLES;
			}
			break;
			
		case STATE_OFFSET_CAL_SAMPLES:
			if (g_new_command) {
				g_new_command = false;
				uint16_t samples = atoi(g_rx_buffer);
				
				Command offset_cmd;
				offset_cmd.dev_address = g_dev_address;
				offset_cmd.command_id = CMD_START_OFFSET_CAL;
				offset_cmd.data.offset_cal.cal_distance_mm = g_temp_distance;
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
				g_temp_distance = atoi(g_rx_buffer);
				Serial.print(F("Enter sample count (5-255): "));
				g_state = STATE_XTALK_CAL_SAMPLES;
			}
			break;
			
		case STATE_XTALK_CAL_SAMPLES:
			if (g_new_command) {
				g_new_command = false;
				uint16_t samples = atoi(g_rx_buffer);
				
				Command xtalk_cmd;
				xtalk_cmd.dev_address = g_dev_address;
				xtalk_cmd.command_id = CMD_START_XTALK_CAL;
				xtalk_cmd.data.xtalk_cal.cal_distance_mm = g_temp_distance;
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
				g_temp_sigma = atoi(g_rx_buffer);
				Serial.print(F("Enter signal threshold (kcps): "));
				g_state = STATE_SET_THRESHOLD_SIGNAL;
			}
			break;
			
		case STATE_SET_THRESHOLD_SIGNAL:
			if (g_new_command) {
				g_new_command = false;
				uint16_t signal = atoi(g_rx_buffer);
				
				Command threshold_cmd;
				threshold_cmd.dev_address = g_dev_address;
				threshold_cmd.command_id = CMD_SET_THRESHOLDS;
				threshold_cmd.data.set_thresholds.sigma = g_temp_sigma;
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
			
		case STATE_RANGE_ALL_UNITS:
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
					g_state = STATE_RANGE_ALL_EXEC;
				}
			}
			break;
			
		case STATE_RANGE_ALL_EXEC:
			Serial.print(F("Ranging all devices with unit: "));
			Serial.println(unitLabel(g_selected_unit));
			executeRangingCommandAllDevices(g_selected_unit);
			g_state = STATE_IDLE;
			printCommandMenu();
			break;
			
		case STATE_RANGE_ALL_CONTINUOUS_UNITS:
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
					g_state = STATE_RANGE_ALL_CONTINUOUS_RATE;
					Serial.print(F("Enter repetition rate in ms: "));
				}
			}
			break;
			
		case STATE_RANGE_ALL_CONTINUOUS_RATE:
			if (g_new_command) {
				g_new_command = false;
				uint16_t repeat_ms = atoi(g_rx_buffer);
				executeRangingCommandAllDevicesContinuous(g_selected_unit, repeat_ms);
				g_state = STATE_IDLE;
				printCommandMenu();
			}
			break;
	}
}

#else
// ============================================================================
// AUTONOMOUS RANGING MODE - Continuous ranging without serial interaction
// ============================================================================

// Structure to store device information
struct DeviceInfo {
	uint8_t address;
	uint8_t timeBudgetMs;
	uint16_t interMeasurementMs;
	int16_t offsetMm;
	uint16_t xtalkKcps;
	uint16_t sigmaThresholdMm;
	uint16_t signalThresholdKcps;
	uint8_t firmwareRev;
};

// Global device list and count
static DeviceInfo devices[120];
static uint8_t deviceCount = 0;
static uint8_t longestTimeBudgetMs = 0;

// Ranging configuration
static const uint8_t RANGING_UNIT = 0x52; // mm
static const uint16_t RANGING_PERIOD_MS = 100; // Adjust as needed
static uint16_t effectiveRangingPeriodMs = RANGING_PERIOD_MS;

// One-iteration delayed processing state
static uint8_t currentDeviceIndex = 0;
static bool hasPreviousRangingData = false;
static uint8_t previousDeviceIndex = 0;
static uint8_t previousAddress = 0;
static uint16_t previousDistance = 0;
static uint8_t previousRangeStatus = 0;
static uint8_t previousRangingData[15] = {0};

// Send a ranging command without reading the result.
static bool sendRangingCommand(uint8_t addr, uint8_t unit) {
	uint8_t cmd[2] = {0x00, unit};
	return i2cWriteBytes(addr, cmd, sizeof(cmd));
}

// Read a ranging result by polling range status until ready or timeout.
static bool readRangingResultWithPoll(uint8_t addr, uint8_t *buf, uint8_t len) {
	const uint16_t pollIntervalMs = 10;
	const uint16_t timeoutMs = 1000;
	const unsigned long startMs = millis();
	while (true) {
		if (!i2cReadBytes(addr, buf, len)) {
			return false;
		}
		if (buf[2] <= 12) {
			return true;
		}
		if (static_cast<uint16_t>(millis() - startMs) >= timeoutMs) {
			return false;
		}
		delay(pollIntervalMs);
	}
}

// Detect connected I2C devices and read their configurations
static void detectDevices() {
	deviceCount = 0;
	longestTimeBudgetMs = 0;
	
	for (uint8_t addr = 0x08; addr <= 0x7F; ++addr) {
		Wire.beginTransmission(addr);
		if (Wire.endTransmission() != 0) {
			continue;
		}
		
		// Device found, read its configuration
		uint8_t cfg[13] = {0};
		if (!readConfig(addr, cfg, sizeof(cfg))) {
			continue;
		}
		
		if (deviceCount < 120) {
			devices[deviceCount].address = addr;
			devices[deviceCount].timeBudgetMs = cfg[1];
			if (cfg[1] > longestTimeBudgetMs) {
				longestTimeBudgetMs = cfg[1];
			}
			devices[deviceCount].interMeasurementMs = readU16Be(cfg, 2);
			devices[deviceCount].offsetMm = readS16Be(cfg, 4);
			devices[deviceCount].xtalkKcps = readU16Be(cfg, 6);
			devices[deviceCount].sigmaThresholdMm = readU16Be(cfg, 8);
			devices[deviceCount].signalThresholdKcps = readU16Be(cfg, 10);
			devices[deviceCount].firmwareRev = cfg[12];
			deviceCount++;
		}
	}
}

// User-defined function to process ranging data between acquisitions
// Leave this empty for the user to implement their custom data processing
void process_data(uint8_t deviceIndex, uint8_t address, uint16_t distance, uint8_t rangeStatus, const uint8_t* rangingData) {
	// TODO: User implementation goes here
	// 
	// Parameters:
	//   deviceIndex: Index of the device in the devices array (0-based)
	//   address: I2C address of the device
	//   distance: Distance reading in the configured unit (mm by default)
	//   rangeStatus: Range status code (0 = valid)
	//   rangingData: Pointer to full 15-byte ranging result buffer
	//     [0-1]: distance (uint16_t, big-endian)
	//     [2]: range status
	//     [3-4]: signal rate (uint16_t, big-endian)
	//     [5-6]: ambient rate (uint16_t, big-endian)
	//     [7-8]: sigma (uint16_t, big-endian)
	//     [9-10]: ambient per spad (uint16_t, big-endian)
	//     [11-12]: signal per spad (uint16_t, big-endian)
	//     [13-14]: number of spads (uint16_t, big-endian)
}

// Initialize I2C and detect all connected devices
void setup() {
	// Allow 10 seconds after power-up before I2C activity
	delay(10000);
	
	Wire.begin();
	Wire.setClock(400000);
	
	// Detect all connected devices and read their configurations
	detectDevices();
	if (effectiveRangingPeriodMs < longestTimeBudgetMs) {
		effectiveRangingPeriodMs = longestTimeBudgetMs;
	}
}

// Continuously perform ranging on all detected devices
void loop() {
	if (deviceCount == 0) {
		return;
	}

	if (currentDeviceIndex >= deviceCount) {
		currentDeviceIndex = 0;
	}

	const unsigned long iterationStartMs = millis();
	const uint8_t deviceIndex = currentDeviceIndex;
	const uint8_t addr = devices[deviceIndex].address;

	// 1) Send ranging command.
	const bool commandSent = sendRangingCommand(addr, RANGING_UNIT);

	// 2) Process data acquired in the previous iteration.
	if (hasPreviousRangingData) {
		process_data(previousDeviceIndex, previousAddress, previousDistance, previousRangeStatus, previousRangingData);
		hasPreviousRangingData = false;
	}

	// 3) Possibly wait for iteration period to expire.
	const unsigned long elapsedMs = millis() - iterationStartMs;
	if (elapsedMs < effectiveRangingPeriodMs) {
		delay(static_cast<uint16_t>(effectiveRangingPeriodMs - elapsedMs));
	}

	// Acquire data for processing in the next iteration.
	if (commandSent && readRangingResultWithPoll(addr, previousRangingData, sizeof(previousRangingData))) {
		previousDeviceIndex = deviceIndex;
		previousAddress = addr;
		previousDistance = resolveDistance(readU16Be(previousRangingData, 0), previousRangingData[2]);
		previousRangeStatus = previousRangingData[2];
		hasPreviousRangingData = true;
	}

	// 4) Back to point 1.
	currentDeviceIndex = static_cast<uint8_t>((currentDeviceIndex + 1) % deviceCount);
}

#endif // SETUP_RANGING
