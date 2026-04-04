#include <Arduino.h>
#include <Wire.h>
#include <SensirionI2cScd30.h>

// Make sure NO_ERROR is defined exactly as in the Sensirion example
#ifdef NO_ERROR
#undef NO_ERROR
#endif
#define NO_ERROR 0

SensirionI2cScd30 scd30;

static char errorMessage[128];
static int16_t errorCode = 0;

static const uint8_t SDA_PIN = 21;
static const uint8_t SCL_PIN = 22;
static const uint32_t I2C_FREQ = 50000;   // conservative for SCD30
static const uint32_t SERIAL_BAUD = 115200;

bool i2cScanForSCD30() {
  Serial.println();
  Serial.println("I2C scan starting...");

  bool foundAny = false;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t rc = Wire.endTransmission();

    if (rc == 0) {
      foundAny = true;
      Serial.print("I2C device found at 0x");
      if (addr < 16) Serial.print('0');
      Serial.println(addr, HEX);
    }
  }

  if (!foundAny) {
    Serial.println("No I2C devices found.");
    return false;
  }

  Serial.println("I2C scan done.");
  return true;
}

bool scd30Init() {
  Serial.println();
  Serial.println("Initializing SCD30...");

  scd30.begin(Wire, SCD30_I2C_ADDR_61);

  // Ignore stop errors here; sensor may already be stopped
  (void)scd30.stopPeriodicMeasurement();

  errorCode = scd30.softReset();
  if (errorCode != NO_ERROR) {
    errorToString(errorCode, errorMessage, sizeof(errorMessage));
    Serial.print("softReset() failed: ");
    Serial.println(errorMessage);
    return false;
  }

  delay(2000);

  uint8_t fwMajor = 0;
  uint8_t fwMinor = 0;

  errorCode = scd30.readFirmwareVersion(fwMajor, fwMinor);
  if (errorCode != NO_ERROR) {
    errorToString(errorCode, errorMessage, sizeof(errorMessage));
    Serial.print("readFirmwareVersion() failed: ");
    Serial.println(errorMessage);
    return false;
  }

  Serial.print("SCD30 firmware: ");
  Serial.print(fwMajor);
  Serial.print(".");
  Serial.println(fwMinor);

  // Optional but useful for diagnostics
  uint16_t asc = 0;
  errorCode = scd30.getAutoCalibrationStatus(asc);
  if (errorCode == NO_ERROR) {
    Serial.print("ASC status before start: ");
    Serial.println(asc ? "ON" : "OFF");
  } else {
    errorToString(errorCode, errorMessage, sizeof(errorMessage));
    Serial.print("getAutoCalibrationStatus() failed: ");
    Serial.println(errorMessage);
  }

  errorCode = scd30.startPeriodicMeasurement(0);
  if (errorCode != NO_ERROR) {
    errorToString(errorCode, errorMessage, sizeof(errorMessage));
    Serial.print("startPeriodicMeasurement(0) failed: ");
    Serial.println(errorMessage);
    return false;
  }

  Serial.println("startPeriodicMeasurement(0): OK");
  Serial.println("Wait for the SCD30 yellow LED to begin its slow blink.");
  return true;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  Serial.println();
  Serial.println("========================================");
  Serial.println("ESP32 + SCD30 minimal diagnostic sketch");
  Serial.println("========================================");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_FREQ);
  Wire.setTimeOut(200);  // ms; more tolerant of clock stretching

  Serial.print("I2C SDA pin: ");
  Serial.println(SDA_PIN);
  Serial.print("I2C SCL pin: ");
  Serial.println(SCL_PIN);
  Serial.print("I2C clock: ");
  Serial.print(I2C_FREQ);
  Serial.println(" Hz");
  Serial.println("I2C timeout: 200 ms");

  i2cScanForSCD30();

  if (!scd30Init()) {
    Serial.println();
    Serial.println("SCD30 init FAILED.");
    Serial.println("Power-cycle the sensor after fixing the issue and try again.");
    return;
  }

  Serial.println();
  Serial.println("Setup complete.");
}

void loop() {
  static uint32_t lastReadMs = 0;
  const uint32_t now = millis();

  if (now - lastReadMs < 3000) {
    delay(10);
    return;
  }
  lastReadMs = now;

  uint16_t ready = 0;
  errorCode = scd30.getDataReady(ready);
  if (errorCode != NO_ERROR) {
    errorToString(errorCode, errorMessage, sizeof(errorMessage));
    Serial.print("getDataReady() failed: ");
    Serial.println(errorMessage);
  } else {
    Serial.print("dataReady = ");
    Serial.println(ready ? "true" : "false");
  }

  float co2 = 0.0f;
  float temp = 0.0f;
  float rh = 0.0f;

  errorCode = scd30.blockingReadMeasurementData(co2, temp, rh);
  if (errorCode != NO_ERROR) {
    errorToString(errorCode, errorMessage, sizeof(errorMessage));
    Serial.print("blockingReadMeasurementData() failed: ");
    Serial.println(errorMessage);
    return;
  }

  Serial.print("CO2: ");
  Serial.print(co2, 1);
  Serial.print(" ppm   Temp: ");
  Serial.print(temp, 2);
  Serial.print(" C   RH: ");
  Serial.print(rh, 2);
  Serial.println(" %");
}