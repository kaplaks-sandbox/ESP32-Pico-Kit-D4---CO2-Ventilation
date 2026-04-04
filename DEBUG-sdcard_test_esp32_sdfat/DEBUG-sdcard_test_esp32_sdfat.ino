#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>

// ESP32-PICO-D4 SD card SPI test using SdFat
// Pins fixed to the values you provided.

static const uint8_t SD_CS_PIN   = 4;
static const uint8_t SD_SCK_PIN  = 18;
static const uint8_t SD_MISO_PIN = 19;
static const uint8_t SD_MOSI_PIN = 23;
static const uint32_t SD_SPI_MHZ = 1;


SPIClass sdSpi(VSPI);
SdFs sd;
FsFile file;

void printDivider() {
  Serial.println(F("----------------------------------------"));
}

void listRootOnce() {
  Serial.println(F("Root directory:"));
  sd.ls("/", LS_DATE | LS_SIZE);
}

bool writeReadTest() {
  const char* path = "/sd_test.csv";

  if (!file.open(path, O_RDWR | O_CREAT | O_TRUNC)) {
    Serial.println(F("FAIL: could not open /sd_test.csv for write"));
    return false;
  }

  file.println(F("timestamp,message,value"));
  file.println(F("0,hello,123"));
  file.println(F("1,esp32-sdfat-test,456"));
  file.flush();
  file.close();

  Serial.println(F("Write test OK."));

  if (!file.open(path, O_RDONLY)) {
    Serial.println(F("FAIL: could not reopen /sd_test.csv for read"));
    return false;
  }

  Serial.println(F("Read-back:"));
  while (file.available()) {
    Serial.write(file.read());
  }
  Serial.println();
  file.close();
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1200);
pinMode(SD_CS_PIN, OUTPUT);
digitalWrite(SD_CS_PIN, HIGH);
delay(20);

  printDivider();
  Serial.println(F("ESP32 SdFat SD-card test"));
  Serial.print(F("CS   : "));
  Serial.println(SD_CS_PIN);
  Serial.print(F("SCK  : "));
  Serial.println(SD_SCK_PIN);
  Serial.print(F("MISO : "));
  Serial.println(SD_MISO_PIN);
  Serial.print(F("MOSI : "));
  Serial.println(SD_MOSI_PIN);
  Serial.print(F("SPI  : "));
  Serial.print(SD_SPI_MHZ);
  Serial.println(F(" MHz"));
  printDivider();

  sdSpi.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  delay(50);

  SdSpiConfig cfg(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(SD_SPI_MHZ), &sdSpi);

  if (!sd.begin(cfg)) {
    Serial.println(F("SD init FAILED."));
    Serial.println(F("Try these next if wiring is correct:"));
    Serial.println(F("1) lower SD_SPI_MHZ from 8 to 4"));
    Serial.println(F("2) if still failing, try 1 MHz"));
    Serial.println(F("3) verify module power and 3.3 V logic"));
    while (true) {
      delay(1000);
    }
  }

  Serial.println(F("SD init OK."));
  printDivider();

  listRootOnce();
  printDivider();

  if (writeReadTest()) {
    Serial.println(F("SD read/write test PASSED."));
  } else {
    Serial.println(F("SD read/write test FAILED."));
  }

  printDivider();
  Serial.println(F("Setup done."));
}

void loop() {
  delay(1000);
}
