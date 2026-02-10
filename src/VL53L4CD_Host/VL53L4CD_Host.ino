// Arduino Uno host utility for VL53L4CD breakout I2C commands.

#include <Arduino.h>
#include <Wire.h>

// Default I2C address for the VL53L4CD breakout (7-bit).
static const uint8_t kDefaultI2cAddr = 0x70;
static uint8_t g_i2cAddr = kDefaultI2cAddr;

enum RangeUnit : uint8_t {
  kUnitMm = 0x52,
  kUnitCm = 0x51,
  kUnitIn = 0x50
};

struct RangeResult {
  uint16_t distance;
  uint8_t rangeStatus;
  uint16_t signalRate;
  uint16_t ambientRate;
  uint16_t sigma;
  uint16_t ambientPerSpad;
  uint16_t signalPerSpad;
  uint16_t spadCount;
};

struct ConfigData {
  uint8_t i2cAddress;
  uint8_t timeBudgetMs;
  uint16_t interMeasurementMs;
  int16_t offsetMm;
  uint16_t xtalkKcps;
  uint16_t sigmaMm;
  uint16_t signalKcps;
  uint8_t firmwareRev;
};

// Write a raw command payload to the device.
static void i2cWriteBytes(uint8_t addr, const uint8_t *data, uint8_t len) {
  Wire.beginTransmission(addr);
  for (uint8_t i = 0; i < len; ++i) {
    Wire.write(data[i]);
  }
  Wire.endTransmission();
}

// Read a fixed-size response buffer from the device.
static bool i2cReadBytes(uint8_t addr, uint8_t *data, uint8_t len) {
  Wire.requestFrom(addr, len);
  uint8_t idx = 0;
  while (Wire.available() && idx < len) {
    data[idx++] = Wire.read();
  }
  return (idx == len);
}

static uint16_t be16(const uint8_t *buf) {
  return (static_cast<uint16_t>(buf[0]) << 8) | buf[1];
}

static int16_t be16s(const uint8_t *buf) {
  return static_cast<int16_t>(be16(buf));
}

// Command 0x00: trigger a single measurement in the selected unit.
static bool cmdRanging(RangeUnit unit) {
  uint8_t cmd[2] = {0x00, static_cast<uint8_t>(unit)};
  i2cWriteBytes(g_i2cAddr, cmd, sizeof(cmd));
  return true;
}

// Command 0x01: set timing (time budget and inter-measurement period).
static bool cmdSetTiming(uint8_t timeBudgetMs, uint16_t interMs) {
  uint8_t cmd[4] = {0x01, timeBudgetMs, static_cast<uint8_t>(interMs >> 8),
                   static_cast<uint8_t>(interMs)};
  i2cWriteBytes(g_i2cAddr, cmd, sizeof(cmd));
  return true;
}

// Command 0x02: offset calibration at known distance.
static bool cmdOffsetCal(uint16_t distMm, uint16_t samples) {
  uint8_t cmd[5] = {0x02, static_cast<uint8_t>(distMm >> 8), static_cast<uint8_t>(distMm),
                   static_cast<uint8_t>(samples >> 8), static_cast<uint8_t>(samples)};
  i2cWriteBytes(g_i2cAddr, cmd, sizeof(cmd));
  return true;
}

// Command 0x03: crosstalk calibration at known distance.
static bool cmdXtalkCal(uint16_t distMm, uint16_t samples) {
  uint8_t cmd[5] = {0x03, static_cast<uint8_t>(distMm >> 8), static_cast<uint8_t>(distMm),
                   static_cast<uint8_t>(samples >> 8), static_cast<uint8_t>(samples)};
  i2cWriteBytes(g_i2cAddr, cmd, sizeof(cmd));
  return true;
}

// Command 0x04: read the configuration block into a struct.
static bool cmdReadConfig(ConfigData *out) {
  if (!out) {
    return false;
  }
  uint8_t cmd = 0x04;
  uint8_t buf[13] = {0};
  i2cWriteBytes(g_i2cAddr, &cmd, 1);
  if (!i2cReadBytes(g_i2cAddr, buf, sizeof(buf))) {
    return false;
  }
  out->i2cAddress = buf[0];
  out->timeBudgetMs = buf[1];
  out->interMeasurementMs = be16(&buf[2]);
  out->offsetMm = be16s(&buf[4]);
  out->xtalkKcps = be16(&buf[6]);
  out->sigmaMm = be16(&buf[8]);
  out->signalKcps = be16(&buf[10]);
  out->firmwareRev = buf[12];
  return true;
}

// Command 0x05: restore EEPROM defaults (address preserved).
static bool cmdRestoreDefaults() {
  uint8_t cmd = 0x05;
  i2cWriteBytes(g_i2cAddr, &cmd, 1);
  return true;
}

// Command 0x07: update sigma and signal thresholds.
static bool cmdSetThresholds(uint16_t sigmaMm, uint16_t signalKcps) {
  uint8_t cmd[5] = {0x07, static_cast<uint8_t>(sigmaMm >> 8), static_cast<uint8_t>(sigmaMm),
                   static_cast<uint8_t>(signalKcps >> 8), static_cast<uint8_t>(signalKcps)};
  i2cWriteBytes(g_i2cAddr, cmd, sizeof(cmd));
  return true;
}

// Command 0x08: reload EEPROM values and restart ranging.
static bool cmdRestart() {
  uint8_t cmd = 0x08;
  i2cWriteBytes(g_i2cAddr, &cmd, 1);
  return true;
}

// Address change sequence (00 A0, 00 AA, 00 A5, 00 <addr>).
static bool cmdChangeAddress(uint8_t newAddr) {
  if (newAddr < 0x08 || newAddr > 0x7F) {
    return false;
  }
  uint8_t s1[2] = {0x00, 0xA0};
  uint8_t s2[2] = {0x00, 0xAA};
  uint8_t s3[2] = {0x00, 0xA5};
  uint8_t s4[2] = {0x00, newAddr};
  i2cWriteBytes(g_i2cAddr, s1, 2);
  i2cWriteBytes(g_i2cAddr, s2, 2);
  i2cWriteBytes(g_i2cAddr, s3, 2);
  i2cWriteBytes(g_i2cAddr, s4, 2);
  g_i2cAddr = newAddr;
  return true;
}

static void printHelp() {
  Serial.println();
  Serial.println(F("Menu (enter number):"));
  Serial.println(F("  1) Range single measurement"));
  Serial.println(F("  2) Set timing"));
  Serial.println(F("  3) Offset calibration"));
  Serial.println(F("  4) XTalk calibration"));
  Serial.println(F("  5) Read config"));
  Serial.println(F("  6) Restore defaults"));
  Serial.println(F("  7) Set thresholds"));
  Serial.println(F("  8) Restart"));
  Serial.println(F("  9) Change I2C address"));
  Serial.println(F("Type 'menu' to show this list again."));
}

static bool parseUint16(const char *s, uint16_t *out) {
  if (!s || !out) {
    return false;
  }
  char *end = nullptr;
  unsigned long val = strtoul(s, &end, 0);
  if (end == s || val > 0xFFFF) {
    return false;
  }
  *out = static_cast<uint16_t>(val);
  return true;
}

static bool parseUint8(const char *s, uint8_t *out) {
  uint16_t tmp = 0;
  if (!parseUint16(s, &tmp) || tmp > 0xFF) {
    return false;
  }
  *out = static_cast<uint8_t>(tmp);
  return true;
}

static void printRangeResult(const RangeResult &r, RangeUnit unit) {
  const char *unitStr = (unit == kUnitCm) ? "cm" : (unit == kUnitIn) ? "in" : "mm";
  Serial.print(F("Distance: "));
  Serial.print(r.distance);
  Serial.print(' ');
  Serial.print(unitStr);
  Serial.print(F(" | Status: "));
  Serial.print(r.rangeStatus);
  Serial.print(F(" | Sigma(mm): "));
  Serial.print(r.sigma);
  Serial.print(F(" | Signal(kcps): "));
  Serial.println(r.signalRate);
}

static bool readRangeResult(RangeResult *out) {
  uint8_t buf[15] = {0};
  if (!out) {
    return false;
  }
  if (!i2cReadBytes(g_i2cAddr, buf, sizeof(buf))) {
    return false;
  }
  out->distance = be16(&buf[0]);
  out->rangeStatus = buf[2];
  out->signalRate = be16(&buf[3]);
  out->ambientRate = be16(&buf[5]);
  out->sigma = be16(&buf[7]);
  out->ambientPerSpad = be16(&buf[9]);
  out->signalPerSpad = be16(&buf[11]);
  out->spadCount = be16(&buf[13]);
  return true;
}

enum PendingCmd : uint8_t {
  kCmdNone = 0,
  kCmdRange = 1,
  kCmdTiming = 2,
  kCmdOffset = 3,
  kCmdXtalk = 4,
  kCmdConfig = 5,
  kCmdRestore = 6,
  kCmdThresholds = 7,
  kCmdRestart = 8,
  kCmdAddr = 9
};

struct PendingState {
  PendingCmd cmd = kCmdNone;
  uint8_t step = 0;
  RangeUnit unit = kUnitMm;
  uint16_t p1 = 0;
  uint16_t p2 = 0;
};

static PendingState g_state;

static void resetState() {
  g_state.cmd = kCmdNone;
  g_state.step = 0;
  g_state.unit = kUnitMm;
  g_state.p1 = 0;
  g_state.p2 = 0;
}

static void promptI2cAddress() {
  Serial.println(F("Enter I2C address (hex 0x70 or dec):"));
}

static void promptForNextParam() {
  switch (g_state.cmd) {
    case kCmdRange:
      if (g_state.step == 2) {
        Serial.println(F("Enter unit (mm, cm, in):"));
      } else if (g_state.step == 3) {
        Serial.println(F("Enter delay ms before read (e.g., 60):"));
      }
      break;
    case kCmdTiming:
      if (g_state.step == 2) {
        Serial.println(F("Enter time budget ms (10-200):"));
      } else if (g_state.step == 3) {
        Serial.println(F("Enter inter-measurement ms (0-5000):"));
      }
      break;
    case kCmdOffset:
    case kCmdXtalk:
      if (g_state.step == 2) {
        Serial.println(F("Enter target distance mm:"));
      } else if (g_state.step == 3) {
        Serial.println(F("Enter sample count (5-255):"));
      }
      break;
    case kCmdThresholds:
      if (g_state.step == 2) {
        Serial.println(F("Enter sigma threshold mm:"));
      } else if (g_state.step == 3) {
        Serial.println(F("Enter signal threshold kcps:"));
      }
      break;
    case kCmdAddr:
      if (g_state.step == 2) {
        Serial.println(F("Enter new I2C address (0x08-0x7F):"));
      }
      break;
    default:
      break;
  }
}

static void finishCommand() {
  resetState();
  printHelp();
}

// Parse and execute a single line from Serial input.
static void handleLine(char *line) {
  char *cmd = strtok(line, " \t\r\n");
  if (!cmd) {
    return;
  }

  if (strcmp(cmd, "menu") == 0 || strcmp(cmd, "help") == 0) {
    printHelp();
    return;
  }

  if (g_state.cmd == kCmdNone) {
    uint16_t choice = 0;
    if (!parseUint16(cmd, &choice) || choice < 1 || choice > 9) {
      Serial.println(F("Enter a menu number 1-9 or 'menu'."));
      return;
    }
    g_state.cmd = static_cast<PendingCmd>(choice);
    g_state.step = 1;
    promptI2cAddress();
    return;
  }

  if (g_state.step == 1) {
    uint8_t addr = 0;
    if (!parseUint8(cmd, &addr) || addr < 0x08 || addr > 0x7F) {
      Serial.println(F("Invalid address. Use 0x08-0x7F."));
      promptI2cAddress();
      return;
    }
    g_i2cAddr = addr;
    g_state.step = 2;
    if (g_state.cmd == kCmdConfig) {
      ConfigData cfg = {};
      if (cmdReadConfig(&cfg)) {
        Serial.print(F("Addr: 0x"));
        Serial.println(cfg.i2cAddress, HEX);
        Serial.print(F("Time budget (ms): "));
        Serial.println(cfg.timeBudgetMs);
        Serial.print(F("Inter-measurement (ms): "));
        Serial.println(cfg.interMeasurementMs);
        Serial.print(F("Offset (mm): "));
        Serial.println(cfg.offsetMm);
        Serial.print(F("XTalk (kcps): "));
        Serial.println(cfg.xtalkKcps);
        Serial.print(F("Sigma (mm): "));
        Serial.println(cfg.sigmaMm);
        Serial.print(F("Signal (kcps): "));
        Serial.println(cfg.signalKcps);
        Serial.print(F("Firmware rev: "));
        Serial.println(cfg.firmwareRev);
      } else {
        Serial.println(F("Config read failed"));
      }
      finishCommand();
      return;
    }
    if (g_state.cmd == kCmdRestore) {
      cmdRestoreDefaults();
      Serial.println(F("Defaults restored"));
      finishCommand();
      return;
    }
    if (g_state.cmd == kCmdRestart) {
      cmdRestart();
      Serial.println(F("Device restarted"));
      finishCommand();
      return;
    }
    promptForNextParam();
    return;
  }

  if (g_state.cmd == kCmdRange) {
    if (g_state.step == 2) {
      if (strcmp(cmd, "mm") == 0) {
        g_state.unit = kUnitMm;
      } else if (strcmp(cmd, "cm") == 0) {
        g_state.unit = kUnitCm;
      } else if (strcmp(cmd, "in") == 0) {
        g_state.unit = kUnitIn;
      } else {
        Serial.println(F("Invalid unit. Use mm, cm, or in."));
        promptForNextParam();
        return;
      }
      g_state.step = 3;
      promptForNextParam();
      return;
    }
    if (g_state.step == 3) {
      if (!parseUint16(cmd, &g_state.p1)) {
        Serial.println(F("Invalid delay."));
        promptForNextParam();
        return;
      }
      cmdRanging(g_state.unit);
      delay(g_state.p1);
      RangeResult r = {};
      if (readRangeResult(&r)) {
        printRangeResult(r, g_state.unit);
      } else {
        Serial.println(F("Read failed"));
      }
      finishCommand();
      return;
    }
  }

  if (g_state.cmd == kCmdTiming) {
    if (g_state.step == 2) {
      if (!parseUint16(cmd, &g_state.p1)) {
        Serial.println(F("Invalid time budget."));
        promptForNextParam();
        return;
      }
      g_state.step = 3;
      promptForNextParam();
      return;
    }
    if (g_state.step == 3) {
      if (!parseUint16(cmd, &g_state.p2)) {
        Serial.println(F("Invalid inter-measurement."));
        promptForNextParam();
        return;
      }
      cmdSetTiming(static_cast<uint8_t>(g_state.p1), g_state.p2);
      Serial.println(F("Timing updated"));
      finishCommand();
      return;
    }
  }

  if (g_state.cmd == kCmdOffset || g_state.cmd == kCmdXtalk) {
    if (g_state.step == 2) {
      if (!parseUint16(cmd, &g_state.p1)) {
        Serial.println(F("Invalid distance."));
        promptForNextParam();
        return;
      }
      g_state.step = 3;
      promptForNextParam();
      return;
    }
    if (g_state.step == 3) {
      if (!parseUint16(cmd, &g_state.p2)) {
        Serial.println(F("Invalid sample count."));
        promptForNextParam();
        return;
      }
      if (g_state.cmd == kCmdOffset) {
        cmdOffsetCal(g_state.p1, g_state.p2);
        Serial.println(F("Offset calibration started; wait before reading."));
      } else {
        cmdXtalkCal(g_state.p1, g_state.p2);
        Serial.println(F("XTalk calibration started; wait before reading."));
      }
      finishCommand();
      return;
    }
  }

  if (g_state.cmd == kCmdThresholds) {
    if (g_state.step == 2) {
      if (!parseUint16(cmd, &g_state.p1)) {
        Serial.println(F("Invalid sigma value."));
        promptForNextParam();
        return;
      }
      g_state.step = 3;
      promptForNextParam();
      return;
    }
    if (g_state.step == 3) {
      if (!parseUint16(cmd, &g_state.p2)) {
        Serial.println(F("Invalid signal value."));
        promptForNextParam();
        return;
      }
      cmdSetThresholds(g_state.p1, g_state.p2);
      Serial.println(F("Thresholds updated"));
      finishCommand();
      return;
    }
  }

  if (g_state.cmd == kCmdAddr) {
    if (g_state.step == 2) {
      uint8_t newAddr = 0;
      if (!parseUint8(cmd, &newAddr) || newAddr < 0x08 || newAddr > 0x7F) {
        Serial.println(F("Invalid address (0x08-0x7F)."));
        promptForNextParam();
        return;
      }
      if (!cmdChangeAddress(newAddr)) {
        Serial.println(F("Address change failed"));
      } else {
        Serial.print(F("Address changed to 0x"));
        Serial.println(g_i2cAddr, HEX);
      }
      finishCommand();
      return;
    }
  }

  Serial.println(F("Unexpected input. Type 'menu' for options."));
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Serial.println(F("VL53L4CD I2C host ready."));
  printHelp();
}

void loop() {
  static char lineBuf[96] = {0};
  if (Serial.available()) {
    // Read one command line at a time from the serial monitor.
    size_t len = Serial.readBytesUntil('\n', lineBuf, sizeof(lineBuf) - 1);
    lineBuf[len] = '\0';
    handleLine(lineBuf);
  }
}
