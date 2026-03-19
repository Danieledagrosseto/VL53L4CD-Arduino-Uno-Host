// Uncomment to enable interactive setup/configuration mode
// Comment out (or leave undefined) to enable autonomous ranging mode
// #define SETUP_RANGING

#include <Arduino.h>
#include <Wire.h>
#include <ctype.h>

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

// Request the sensor configuration block and read it into cfg.
static bool readConfig(uint8_t addr, uint8_t *cfg, uint8_t len) {
	const uint8_t cmd = 0x04;
	if (!i2cWriteBytes(addr, &cmd, 1)) {
		return false;
	}
	// Give the sensor time to prepare the response.
	delay(10);
	return i2cReadBytes(addr, cfg, len);
}

// Read a big-endian unsigned 16-bit value from a byte buffer.
static uint16_t readU16Be(const uint8_t *buf, uint8_t msbIndex) {
	return static_cast<uint16_t>(buf[msbIndex] << 8) | buf[msbIndex + 1];
}

// Read a big-endian signed 16-bit value from a byte buffer.
static int16_t readS16Be(const uint8_t *buf, uint8_t msbIndex) {
	return static_cast<int16_t>(readU16Be(buf, msbIndex));
}

// Maximum measurable distance reported for out-of-range phase/wrap/low-signal errors.
static const uint16_t RANGING_MAX_DISTANCE_MM = 1300;

// Map a range-status byte (buf[2]) to an effective distance value.
// Gravity None (0) or Warning (1, 2, 6): return raw sensor distance.
// Status 3 (below detection threshold): return 0.
// Status 4, 7, 12 (phase/wrap/low-signal errors): return RANGING_MAX_DISTANCE_MM.
// All other errors: return 0.
static uint16_t resolveDistance(uint16_t rawDistance, uint8_t status) {
	switch (status) {
		case 0:   // None: valid distance
		case 1:   // Warning: sigma above threshold
		case 2:   // Warning: signal below threshold
		case 6:   // Warning: phase valid, no wrap-around check
			return rawDistance;
		case 3:   // Error: below detection threshold
			return 0;
		case 4:   // Error: phase out of valid limit
		case 7:   // Error: wrapped target, phase mismatch
		case 12:  // Error: signal too low
			return RANGING_MAX_DISTANCE_MM;
		default:  // All other errors
			return 0;
	}
}

#define SETUP_RANGING

#ifdef SETUP_RANGING
// ============================================================================
// SETUP/CONFIGURATION MODE - Interactive serial-based configuration
// ============================================================================

// Decode and print the config payload fields returned by the sensor.
static void printConfigFields(uint8_t addr, const uint8_t *cfg, uint8_t len) {
	if (len < 13) {
		Serial.println(F("Config buffer too small."));
		return;
	}

	const uint8_t storedAddress = cfg[0];
	const uint8_t timeBudgetMs = cfg[1];
	const uint16_t interMeasurementMs = readU16Be(cfg, 2);
	const int16_t offsetMm = readS16Be(cfg, 4);
	const uint16_t xtalkKcps = readU16Be(cfg, 6);
	const uint16_t sigmaThresholdMm = readU16Be(cfg, 8);
	const uint16_t signalThresholdKcps = readU16Be(cfg, 10);
	const uint8_t firmwareRev = cfg[12];

	Serial.print(F("Device at 0x"));
	Serial.print(addr, HEX);
	Serial.print(F(" | stored: 0x"));
	Serial.print(storedAddress, HEX);
	Serial.print(F(" | firmware: "));
	Serial.println(firmwareRev);
	Serial.print(F("Time budget (ms): "));
	Serial.println(timeBudgetMs);
	Serial.print(F("Inter-measurement (ms): "));
	Serial.println(interMeasurementMs);
	Serial.print(F("Offset (mm): "));
	Serial.println(offsetMm);
	Serial.print(F("XTalk (kcps): "));
	Serial.println(xtalkKcps);
	Serial.print(F("Sigma threshold (mm): "));
	Serial.println(sigmaThresholdMm);
	Serial.print(F("Signal threshold (kcps): "));
	Serial.println(signalThresholdKcps);
}

// Read and print configuration for a single I2C address.
static bool printConfigForAddress(uint8_t addr) {
	uint8_t cfg[13] = {0};
	if (!readConfig(addr, cfg, sizeof(cfg))) {
		return false;
	}

	printConfigFields(addr, cfg, sizeof(cfg));
	return true;
}

// Read a line from Serial into buf (blocking) and NUL-terminate it.
static size_t readLine(char *buf, size_t len) {
	if (len == 0) {
		return 0;
	}
	size_t n = 0;
	while (true) {
		if (!Serial.available()) {
			continue;
		}
		char c = static_cast<char>(Serial.read());
		if (c == '\n') {
			break;
		}
		if (c == '\r') {
			continue;
		}
		if (n < len - 1) {
			buf[n++] = c;
		}
	}
	buf[n] = '\0';
	return n;
}

// Parse an integer string into a uint16_t with bounds checking.
static bool parseU16(const char *text, uint16_t minValue, uint16_t maxValue, uint16_t *valueOut) {
	if (text == nullptr || valueOut == nullptr) {
		return false;
	}

	char *end = nullptr;
	unsigned long value = strtoul(text, &end, 0);
	if (end == text || *end != '\0' || value < minValue || value > maxValue) {
		return false;
	}

	*valueOut = static_cast<uint16_t>(value);
	return true;
}

// Prompt the user and parse a bounded uint16_t response.
static bool promptForU16(const __FlashStringHelper *prompt, uint16_t minValue, uint16_t maxValue, uint16_t *valueOut) {
	Serial.println(prompt);
	char buf[24] = {0};
	if (readLine(buf, sizeof(buf)) == 0) {
		return false;
	}
	if (!parseU16(buf, minValue, maxValue, valueOut)) {
		Serial.println(F("Invalid value."));
		return false;
	}
	return true;
}

// Parse an I2C address formatted as 0xNN with valid 7-bit range.
static bool parseI2cAddress(const char *text, uint8_t *addrOut) {
	if (text == nullptr || addrOut == nullptr) {
		return false;
	}

	if (!(text[0] == '0' && (text[1] == 'x' || text[1] == 'X'))) {
		return false;
	}

	char *end = nullptr;
	unsigned long value = strtoul(text + 2, &end, 16);
	if (end == text + 2 || *end != '\0' || value < 0x08 || value > 0x7F) {
		return false;
	}

	*addrOut = static_cast<uint8_t>(value);
	return true;
}

// Prompt for a single I2C address in 0xNN format.
static bool promptForAddress(uint8_t *addrOut) {
	Serial.println(F("Enter VL53L4CD I2C address in 0x format (e.g., 0x29), then press Enter:"));

	char buf[16] = {0};
	if (readLine(buf, sizeof(buf)) == 0) {
		return false;
	}

	if (!parseI2cAddress(buf, addrOut)) {
		Serial.println(F("Invalid I2C address. Use 0x08-0x7F."));
		return false;
	}

	return true;
}

// Prompt for current and new I2C address values in 0xNN format.
static bool promptForI2cAddressPair(uint8_t *oldAddr, uint8_t *newAddr) {
	Serial.println(F("Enter current I2C address in 0x format (e.g., 0x70):"));
	char buf[16] = {0};
	if (readLine(buf, sizeof(buf)) == 0 || !parseI2cAddress(buf, oldAddr)) {
		Serial.println(F("Invalid current address."));
		return false;
	}
	Serial.println(F("Enter new I2C address in 0x format (0x08-0x7F):"));
	if (readLine(buf, sizeof(buf)) == 0 || !parseI2cAddress(buf, newAddr)) {
		Serial.println(F("Invalid new address."));
		return false;
	}
	return true;
}

// Prompt for a unit code used by the sensor (mm/cm/inch).
static bool promptForUnitCode(uint8_t *unitOut) {
	Serial.println(F("Select unit:"));
	Serial.println(F("1) Millimeters (mm)"));
	Serial.println(F("2) Centimeters (cm)"));
	Serial.println(F("3) Inches (inch)"));
	Serial.print(F("Enter choice (1-3): "));
	char buf[16] = {0};
	if (readLine(buf, sizeof(buf)) == 0) {
		return false;
	}
	uint16_t choice = 0;
	if (!parseU16(buf, 1, 3, &choice)) {
		Serial.println(F("Invalid choice."));
		return false;
	}
	switch (choice) {
		case 1:
			*unitOut = 0x52;
			break;
		case 2:
			*unitOut = 0x51;
			break;
		case 3:
			*unitOut = 0x50;
			break;
		default:
			return false;
	}
	return true;
}

// Map the unit code to a printable label.
static const __FlashStringHelper *unitLabel(uint8_t unit) {
	switch (unit) {
		case 0x52:
			return F("mm");
		case 0x51:
			return F("cm");
		case 0x50:
			return F("inch");
		default:
			return F("?");
	}
}

// Prompt for a delay before reading a ranging result.
static bool promptForDelayMs(uint16_t *delayMsOut) {
	return promptForU16(F("Enter delay in ms before reading (e.g., 60):"), 0, 60000, delayMsOut);
}

// Decode and print a single ranging result payload.
static void printRangingResult(const uint8_t *buf, uint8_t len) {
	if (len < 15) {
		Serial.println(F("Ranging buffer too small."));
		return;
	}
	const uint8_t rangeStatus = buf[2];
	const uint16_t distance = resolveDistance(readU16Be(buf, 0), rangeStatus);
	Serial.print(F("Distance: "));
	Serial.println(distance);
	Serial.print(F("Range status: "));
	Serial.println(rangeStatus);
	Serial.print(F("Signal rate: "));
	Serial.println(readU16Be(buf, 3));
	Serial.print(F("Ambient rate: "));
	Serial.println(readU16Be(buf, 5));
	Serial.print(F("Sigma: "));
	Serial.println(readU16Be(buf, 7));
	Serial.print(F("Ambient per spad: "));
	Serial.println(readU16Be(buf, 9));
	Serial.print(F("Signal per spad: "));
	Serial.println(readU16Be(buf, 11));
	Serial.print(F("Number of spads: "));
	Serial.println(readU16Be(buf, 13));
}

// Send a ranging command and poll until range status is ready or timeout.
static bool readRangingWithPoll(uint8_t addr, uint8_t unit, uint8_t *buf, uint8_t len) {
	const uint16_t pollIntervalMs = 5;
	const uint16_t timeoutMs = 20;
	const unsigned long startMs = millis();
	//uint8_t cmd[2] = {0x00, unit};
	//if (!i2cWriteBytes(addr, cmd, sizeof(cmd))) {
	//	return false;
	//}
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

// Run a single ranging command with user-selected unit and delay.
static void runRanging() {
	uint8_t addr = 0;
	if (!promptForAddress(&addr)) {
		return;
	}
	uint8_t unit = 0;
	if (!promptForUnitCode(&unit)) {
		return;
	}
	uint16_t delayMs = 0;
	if (!promptForDelayMs(&delayMs)) {
		return;
	}

	uint8_t cmd[2] = {0x00, unit};
	if (!i2cWriteBytes(addr, cmd, sizeof(cmd))) {
		Serial.println(F("Failed to send ranging command."));
		return;
	}
	if (delayMs > 0) {
		delay(delayMs);
	}
	uint8_t buf[15] = {0};
	if (!readRangingWithPoll(addr, unit, buf, sizeof(buf))) {
		Serial.println(F("Failed to read ranging result."));
		return;
	}
	printRangingResult(buf, sizeof(buf));
}

// Continuously range all detected I2C devices until 's' is received.
static void runContinuousRangingAll() {
	uint8_t unit = 0;
	if (!promptForUnitCode(&unit)) {
		return;
	}
	uint16_t repeatDelayMs = 0;
	if (!promptForU16(F("Enter repeat delay in ms (0-60000):"), 0, 60000, &repeatDelayMs)) {
		return;
	}

	// Discover devices and read their time budgets.
	uint8_t addresses[120] = {0};
	uint8_t timeBudgets[120] = {0};
	uint8_t count = 0;
	uint8_t longestTimeBudgetMs = 0;
	for (uint8_t addr = 0x08; addr <= 0x7F; ++addr) {
		Wire.beginTransmission(addr);
		if (Wire.endTransmission() != 0 || count >= sizeof(addresses)) {
			continue;
		}
		uint8_t cfg[13] = {0};
		if (!readConfig(addr, cfg, sizeof(cfg))) {
			continue;
		}
		addresses[count] = addr;
		timeBudgets[count] = cfg[1];
		if (cfg[1] > longestTimeBudgetMs) {
			longestTimeBudgetMs = cfg[1];
		}
		count++;
	}
	if (count == 0) {
		Serial.println(F("No I2C devices found."));
		return;
	}
	Serial.println(F("Continuous ranging started. Press 's' to stop."));

	uint8_t buf[15] = {0};
	uint32_t sampleIndex = 0;
	while (true) {
		// Stop if 's' received.
		if (Serial.available()) {
			char c = static_cast<char>(Serial.read());
			if (c == 's' || c == 'S') {
				Serial.println(F("Continuous ranging stopped."));
				return;
			}
		}

		// Send start ranging command to all devices.
		for (uint8_t i = 0; i < count; ++i) {
			uint8_t cmd[2] = {0x00, unit};
			i2cWriteBytes(addresses[i], cmd, sizeof(cmd));
		}

		// Wait for the longest time budget using millis().
		if (longestTimeBudgetMs > 0) {
			const unsigned long waitStart = millis();
			while (static_cast<unsigned long>(millis() - waitStart) < longestTimeBudgetMs) {
			}
		}

		// Read ranging results for all devices.
		for (uint8_t i = 0; i < count; ++i) {
			uint8_t addr = addresses[i];
			if (!readRangingWithPoll(addr, unit, buf, sizeof(buf))) {
				continue;
			}
			uint16_t distance = resolveDistance(readU16Be(buf, 0), buf[2]);
			Serial.print(F("#"));
			Serial.print(++sampleIndex);
			Serial.print(F(" 0x"));
			Serial.print(addr, HEX);
			Serial.print(F(" "));
			Serial.print(distance);
			Serial.print(F(" "));
			Serial.println(unitLabel(unit));
		}

		// Wait repeat delay using millis().
		if (repeatDelayMs > 0) {
			const unsigned long delayStart = millis();
			while (static_cast<unsigned long>(millis() - delayStart) < repeatDelayMs) {
			}
		}
	}
}

// Update timing settings (time budget and inter-measurement period).
static void runSetTiming() {
	uint8_t addr = 0;
	if (!promptForAddress(&addr)) {
		return;
	}
	uint16_t timeBudget = 0;
	if (!promptForU16(F("Enter time budget in ms (10-200):"), 10, 200, &timeBudget)) {
		return;
	}
	uint16_t interMs = 0;
	if (!promptForU16(F("Enter inter-measurement in ms (0-5000):"), 0, 5000, &interMs)) {
		return;
	}

	uint8_t cmd[4] = {0x01, static_cast<uint8_t>(timeBudget),
		static_cast<uint8_t>(interMs >> 8), static_cast<uint8_t>(interMs)};
	if (!i2cWriteBytes(addr, cmd, sizeof(cmd))) {
		Serial.println(F("Failed to set timing."));
		return;
	}
	Serial.println(F("Timing updated."));

	// Send restart command to apply the new timing settings
	uint8_t restartCmd = 0x08;
	if (!i2cWriteBytes(addr, &restartCmd, 1)) {
		Serial.println(F("Failed to restart device."));
		return;
	}
	Serial.println(F("Device restarted."));
}

// Start an offset calibration with user-provided target distance.
static void runOffsetCalibration() {
	uint8_t addr = 0;
	if (!promptForAddress(&addr)) {
		return;
	}
	uint16_t distance = 0;
	if (!promptForU16(F("Enter target distance in mm (10-1000):"), 10, 1000, &distance)) {
		return;
	}
	uint16_t samples = 0;
	if (!promptForU16(F("Enter sample count (5-255):"), 5, 255, &samples)) {
		return;
	}

	uint8_t cmd[5] = {0x02, static_cast<uint8_t>(distance >> 8), static_cast<uint8_t>(distance),
		static_cast<uint8_t>(samples >> 8), static_cast<uint8_t>(samples)};
	if (!i2cWriteBytes(addr, cmd, sizeof(cmd))) {
		Serial.println(F("Failed to start offset calibration."));
		return;
	}
	Serial.println(F("Offset calibration started."));
}

// Start an XTalk calibration with user-provided target distance.
static void runXtalkCalibration() {
	uint8_t addr = 0;
	if (!promptForAddress(&addr)) {
		return;
	}
	uint16_t distance = 0;
	if (!promptForU16(F("Enter target distance in mm (10-5000):"), 10, 5000, &distance)) {
		return;
	}
	uint16_t samples = 0;
	if (!promptForU16(F("Enter sample count (5-255):"), 5, 255, &samples)) {
		return;
	}

	uint8_t cmd[5] = {0x03, static_cast<uint8_t>(distance >> 8), static_cast<uint8_t>(distance),
		static_cast<uint8_t>(samples >> 8), static_cast<uint8_t>(samples)};
	if (!i2cWriteBytes(addr, cmd, sizeof(cmd))) {
		Serial.println(F("Failed to start xtalk calibration."));
		return;
	}
	Serial.println(F("XTalk calibration started."));
}

// Restore sensor defaults.
static void runRestoreDefaults() {
	uint8_t addr = 0;
	if (!promptForAddress(&addr)) {
		return;
	}
	uint8_t cmd = 0x05;
	if (!i2cWriteBytes(addr, &cmd, 1)) {
		Serial.println(F("Failed to restore defaults."));
		return;
	}
	Serial.println(F("Defaults restored."));
}

// Update sigma and signal thresholds used by the sensor.
static void runSigmaSignalThresholds() {
	uint8_t addr = 0;
	if (!promptForAddress(&addr)) {
		return;
	}
	uint16_t sigma = 0;
	if (!promptForU16(F("Enter sigma threshold (mm):"), 0, 65535, &sigma)) {
		return;
	}
	uint16_t signal = 0;
	if (!promptForU16(F("Enter signal threshold (kcps):"), 0, 65535, &signal)) {
		return;
	}

	uint8_t cmd[5] = {0x07, static_cast<uint8_t>(sigma >> 8), static_cast<uint8_t>(sigma),
		static_cast<uint8_t>(signal >> 8), static_cast<uint8_t>(signal)};
	if (!i2cWriteBytes(addr, cmd, sizeof(cmd))) {
		Serial.println(F("Failed to update thresholds."));
		return;
	}
	Serial.println(F("Thresholds updated."));
}

// Send a restart command to the sensor.
static void runRestart() {
	uint8_t addr = 0;
	if (!promptForAddress(&addr)) {
		return;
	}
	uint8_t cmd = 0x08;
	if (!i2cWriteBytes(addr, &cmd, 1)) {
		Serial.println(F("Failed to restart device."));
		return;
	}
	Serial.println(F("Restart command sent."));
}

// Change the sensor's I2C address using the unlock sequence.
static void runChangeAddress() {
	uint8_t oldAddr = 0;
	uint8_t newAddr = 0;
	if (!promptForI2cAddressPair(&oldAddr, &newAddr)) {
		return;
	}
	uint8_t s1[2] = {0x00, 0xA0};
	uint8_t s2[2] = {0x00, 0xAA};
	uint8_t s3[2] = {0x00, 0xA5};
	uint8_t s4[2] = {0x00, newAddr};
	if (!i2cWriteBytes(oldAddr, s1, sizeof(s1)) ||
		!i2cWriteBytes(oldAddr, s2, sizeof(s2)) ||
		!i2cWriteBytes(oldAddr, s3, sizeof(s3)) ||
		!i2cWriteBytes(oldAddr, s4, sizeof(s4))) {
		Serial.println(F("Failed to change I2C address."));
		return;
	}
	Serial.print(F("Address changed to 0x"));
	Serial.println(newAddr, HEX);
}

// Scan the I2C bus and print configuration for each responding device.
static void scanAndPrintConfigs() {
	Serial.println(F("Scanning I2C bus for devices..."));
	bool found = false;
	for (uint8_t addr = 0x08; addr <= 0x7F; ++addr) {
		Wire.beginTransmission(addr);
		if (Wire.endTransmission() != 0) {
			continue;
		}
		found = true;
		if (!printConfigForAddress(addr)) {
			Serial.print(F("Device at 0x"));
			Serial.print(addr, HEX);
			Serial.println(F(" did not return config."));
		}
	}
	if (!found) {
		Serial.println(F("No I2C devices found."));
	}
}

// Print the interactive command menu.
static void printMenu() {
	Serial.println();
	Serial.println(F("Commands:"));
	Serial.println(F("1) Scan I2C and print configs"));
	Serial.println(F("2) Read config for one address"));
	Serial.println(F("3) Ranging (single measurement)"));
	Serial.println(F("B) Continuous ranging (all devices)"));
	Serial.println(F("4) Set timing"));
	Serial.println(F("5) Offset calibration"));
	Serial.println(F("6) XTalk calibration"));
	Serial.println(F("7) Restore defaults"));
	Serial.println(F("8) Sigma/signal thresholds"));
	Serial.println(F("9) Restart"));
	Serial.println(F("A) Change I2C address"));
	Serial.println(F("H) Help (print commands)"));
	Serial.print(F("Select command (1-9, A, B, H): "));
}

// Dispatch a menu command.
static void handleCommand(char cmd) {
	switch (cmd) {
		case '1':
			scanAndPrintConfigs();
			break;
		case '2': {
			uint8_t addr = 0;
			if (!promptForAddress(&addr)) {
				return;
			}
			if (!printConfigForAddress(addr)) {
				Serial.println(F("Failed to read configuration."));
			}
			break;
		}
		case '3':
			runRanging();
			break;
		case 'B':
			runContinuousRangingAll();
			break;
		case '4':
			runSetTiming();
			break;
		case '5':
			runOffsetCalibration();
			break;
		case '6':
			runXtalkCalibration();
			break;
		case '7':
			runRestoreDefaults();
			break;
		case '8':
			runSigmaSignalThresholds();
			break;
		case '9':
			runRestart();
			break;
		case 'A':
			runChangeAddress();
			break;
		case 'H':
			printMenu();
			break;
		default:
			Serial.println(F("Unknown command. Enter H for help."));
			break;
	}
}

// Initialize serial, wait for sensor power-up, and show the menu.
void setup() {
	Serial.begin(115200);
	while (!Serial) {
		;
	}
	// Allow 10 seconds after power-up before I2C activity.
	delay(10000);
	Wire.begin();
	Wire.setClock(400000);
	printMenu();
}

// Read a command line, execute it, then reprint the menu.
void loop() {
	char buf[16] = {0};
	if (readLine(buf, sizeof(buf)) == 0) {
		return;
	}
	if (buf[0] == '\0') {
		return;
	}
	char cmd = static_cast<char>(toupper(buf[0]));
	handleCommand(cmd);
	printMenu();
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
