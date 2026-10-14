/*
  ESP32-S3-DevKitC (N16R8) + GY-271 magnetometer -> magnet field streamer

  Streams the 3-axis magnetic field (in microtesla) over USB serial so that
  magnet_viz.html can estimate the 3D position / orientation of a disc magnet.

  GY-271 boards ship with one of three different chips. All are auto-detected:
    - QMC5883L  (I2C 0x0D)  range +-8 G   (most common clone today)
    - HMC5883L  (I2C 0x1E)  range +-8.1 G (original Honeywell part)
    - QMC5883P  (I2C 0x2C)  range +-30 G  (newer clone, best for close magnets)
  The widest range is always selected, because a neodymium magnet close to the
  sensor easily saturates it.

  Wiring (GY-271 -> ESP32-S3-DevKitC):
    VCC -> 3V3
    GND -> GND
    SDA -> GPIO 8
    SCL -> GPIO 9
    DRDY -> not connected

  Serial output, 115200 baud, one line per sample:
    B,<millis>,<Bx_uT>,<By_uT>,<Bz_uT>,<saturated 0|1>
  Info/diagnostic lines start with '#'.
  Commands (send a line): 'i' = print sensor info, 'r' = re-initialise sensor.

  PlatformIO project (see platformio.ini). Output is sent to both the native
  "USB" port and the "UART"/"COM" port, so either connector works.
  No extra libraries are required.
*/

#include <Arduino.h>
#include <Wire.h>

// ---------------- User settings ----------------
static const int      SDA_PIN    = 8;
static const int      SCL_PIN    = 9;
static const uint32_t I2C_HZ     = 400000;
static const uint32_t BAUD       = 115200;
static const uint32_t OUTPUT_HZ  = 100;     // lines per second sent to the PC
// ------------------------------------------------

enum Chip { CHIP_NONE, CHIP_QMC5883L, CHIP_HMC5883L, CHIP_QMC5883P };

static Chip     chip       = CHIP_NONE;
static uint8_t  chipAddr   = 0;
static float    uTperLSB   = 0;            // conversion factor
static const char *chipName = "none";
static const char *chipRange = "";

// ---------- I2C helpers ----------
static bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static bool writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool readRegs(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, (int)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

static uint8_t readReg(uint8_t addr, uint8_t reg) {
  uint8_t v = 0;
  readRegs(addr, reg, &v, 1);
  return v;
}

// ---------- Chip initialisation ----------
static bool initQMC5883L() {
  const uint8_t A = 0x0D;
  writeReg(A, 0x0A, 0x80);           // soft reset
  delay(10);
  writeReg(A, 0x0B, 0x01);           // SET/RESET period (recommended)
  // CTRL1: OSR=512 (00) | RNG=8G (01) | ODR=200Hz (11) | MODE=continuous (01)
  if (!writeReg(A, 0x09, 0b00011101)) return false;
  writeReg(A, 0x0A, 0x40);           // CTRL2: pointer roll-over, no interrupt
  chip = CHIP_QMC5883L; chipAddr = A;
  uTperLSB = 100.0f / 3000.0f;       // 3000 LSB/G at 8 G, 1 G = 100 uT
  chipName = "QMC5883L"; chipRange = "+-8 G (800 uT)";
  return true;
}

static bool initHMC5883L() {
  const uint8_t A = 0x1E;
  if (!writeReg(A, 0x00, 0x78)) return false;  // 8-sample avg, 75 Hz, normal
  writeReg(A, 0x01, 0xE0);                     // gain: +-8.1 G, 230 LSB/G
  writeReg(A, 0x02, 0x00);                     // continuous mode
  chip = CHIP_HMC5883L; chipAddr = A;
  uTperLSB = 100.0f / 230.0f;
  chipName = "HMC5883L"; chipRange = "+-8.1 G (810 uT)";
  return true;
}

static bool initQMC5883P() {
  const uint8_t A = 0x2C;
  writeReg(A, 0x0B, 0x80);           // soft reset
  delay(10);
  writeReg(A, 0x29, 0x06);           // define axis sign (per datasheet)
  // CTRL2: RNG=30G (00) | SET/RESET on (00)
  if (!writeReg(A, 0x0B, 0x00)) return false;
  // CTRL1: OSR2=1 (00) | OSR1=8 (00) | ODR=200Hz (11) | MODE=continuous (11)
  writeReg(A, 0x0A, 0b00001111);
  chip = CHIP_QMC5883P; chipAddr = A;
  uTperLSB = 100.0f / 1000.0f;       // 1000 LSB/G at 30 G
  chipName = "QMC5883P"; chipRange = "+-30 G (3000 uT)";
  return true;
}

static bool detectAndInit() {
  chip = CHIP_NONE;
  if (i2cPresent(0x2C) && readReg(0x2C, 0x00) == 0x80) return initQMC5883P();
  if (i2cPresent(0x1E)) {
    uint8_t id[3] = {0};
    readRegs(0x1E, 0x0A, id, 3);
    if (id[0] == 'H' && id[1] == '4' && id[2] == '3') return initHMC5883L();
  }
  if (i2cPresent(0x0D)) return initQMC5883L();
  return false;
}

// ---------- Reading ----------
// Returns true when a fresh sample was read. Field in uT, sat = clipped/overflow.
static bool readField(float &bx, float &by, float &bz, bool &sat) {
  uint8_t d[6];
  int16_t x, y, z;
  sat = false;

  switch (chip) {
    case CHIP_QMC5883L: {
      uint8_t st = readReg(chipAddr, 0x06);
      if (!(st & 0x01)) return false;                 // DRDY
      if (st & 0x02) sat = true;                      // OVL
      if (!readRegs(chipAddr, 0x00, d, 6)) return false;
      x = (int16_t)(d[1] << 8 | d[0]);
      y = (int16_t)(d[3] << 8 | d[2]);
      z = (int16_t)(d[5] << 8 | d[4]);
      break;
    }
    case CHIP_QMC5883P: {
      uint8_t st = readReg(chipAddr, 0x09);
      if (!(st & 0x01)) return false;
      if (st & 0x02) sat = true;
      if (!readRegs(chipAddr, 0x01, d, 6)) return false;
      x = (int16_t)(d[1] << 8 | d[0]);
      y = (int16_t)(d[3] << 8 | d[2]);
      z = (int16_t)(d[5] << 8 | d[4]);
      break;
    }
    case CHIP_HMC5883L: {
      uint8_t st = readReg(chipAddr, 0x09);
      if (!(st & 0x01)) return false;                 // RDY
      if (!readRegs(chipAddr, 0x03, d, 6)) return false;
      x = (int16_t)(d[0] << 8 | d[1]);                // order is X, Z, Y
      z = (int16_t)(d[2] << 8 | d[3]);
      y = (int16_t)(d[4] << 8 | d[5]);
      if (x == -4096 || y == -4096 || z == -4096) sat = true;
      break;
    }
    default:
      return false;
  }

  if (abs(x) >= 32000 || abs(y) >= 32000 || abs(z) >= 32000) sat = true;
  bx = x * uTperLSB;
  by = y * uTperLSB;
  bz = z * uTperLSB;
  return true;
}

// Output goes to BOTH the native USB port (Serial, USB CDC) and the UART/COM
// port (Serial0), so the board works whichever USB connector is plugged in.
static void out(const char *fmt, ...) {
  char buf[160];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n <= 0) return;
  if (n >= (int)sizeof(buf)) n = sizeof(buf) - 1;
  Serial.write((const uint8_t *)buf, n);
  Serial0.write((const uint8_t *)buf, n);
}

static void printInfo() {
  out("# chip=%s addr=0x%02X range=%s\n", chipName, chipAddr, chipRange);
}

static void scanBus() {
  char line[160] = "# I2C devices:";
  for (uint8_t a = 1; a < 127; a++)
    if (i2cPresent(a)) snprintf(line + strlen(line), sizeof(line) - strlen(line), " 0x%02X", a);
  out("%s\n", line);
}

static void handleCommand(int c) {
  if (c == 'i') printInfo();
  else if (c == 'r') { detectAndInit(); printInfo(); }
}

void setup() {
  Serial.setTxTimeoutMs(0);   // never block when no PC is listening on native USB
  Serial.begin(BAUD);
  Serial0.begin(BAUD);
  delay(300);
  Wire.begin(SDA_PIN, SCL_PIN, I2C_HZ);

  out("# ESP32-S3 GY-271 magnet streamer\n");
  scanBus();
  while (!detectAndInit()) {
    out("# ERROR: no QMC5883L/HMC5883L/QMC5883P found. Check wiring (SDA=8, SCL=9, 3V3).\n");
    delay(1000);
    scanBus();
  }
  printInfo();
}

void loop() {
  static float sx = 0, sy = 0, sz = 0;
  static uint16_t n = 0;
  static bool satAny = false;
  static uint32_t lastOut = millis();

  // Handle simple commands from either port
  while (Serial.available()) handleCommand(Serial.read());
  while (Serial0.available()) handleCommand(Serial0.read());

  // Accumulate every fresh sample, average them per output period
  float bx, by, bz; bool sat;
  if (readField(bx, by, bz, sat)) {
    sx += bx; sy += by; sz += bz; n++;
    satAny |= sat;
  }

  uint32_t now = millis();
  if (now - lastOut >= 1000 / OUTPUT_HZ) {
    lastOut = now;
    if (n > 0) {
      out("B,%lu,%.2f,%.2f,%.2f,%d\n", (unsigned long)now,
          sx / n, sy / n, sz / n, satAny ? 1 : 0);
    }
    sx = sy = sz = 0; n = 0; satAny = false;
  }
}
