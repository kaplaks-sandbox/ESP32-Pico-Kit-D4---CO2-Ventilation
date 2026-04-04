
// ESP32-PICO-D4 CO2 Ventilation Sensor
//Changes:
//added SD keep files for SD space management:
//0 = keep all
//any positive number = keep only that many newest log files in /logs
//old managed log files are pruned automatically after a successful write

//Notes:

//I set the default keep-files value to 31 if no secret or NVS value exists.
//You can also define this in arduino_secrets.h if you want a secrets-side default:
// /sensor now includes the current BME/BMP data:
//sensor model
//presence
//temperature
//humidity
//pressure
//altitude
///system now includes an SD Logging Config box showing:
//enabled
//period
//rotation
//source (Secrets or NVS)
//card ready
//current log path
//fixed SD pins and SPI clock
///config now lets you edit and save to NVS:
//SD logging ON/OFF
//SD log period (sec)
//SD rotation (DAY, MONTH, OFF)
// ESP32-PICO-D4 CO2 Ventilation Sensor
// Version 10.2.0
//What this version does:
// 
//uses SdFat for SD logging
//writes CSV files into /logs
//rotates by filename:
//daily: co2_YYYY-MM-DD.csv
//monthly: co2_YYYY-MM.csv
//off: co2_log.csv
//if time is not synced yet, it writes temporarily to co2_unsynced.csv
//logs one line every SECRET_SD_LOG_PERIOD_SEC
//keeps the current SCD30/BMP/BME/OLED behavior
//
// Update highlights (v10.1.3):
// - Reworked SCD30 startup to follow the proven minimal sequence:
//     * stopPeriodicMeasurement()
//     * softReset()
//     * wait 2 seconds
//     * readFirmwareVersion()
//     * apply config
//     * startPeriodicMeasurement(0)
// - SCD30 read path now uses blockingReadMeasurementData()
//   instead of hard-gating on getDataReady().
// - CO2 polling period increased to 3000 ms.
// - Shared I2C bus forced to 50 kHz with 200 ms timeout.
// - SH1107 display bus speed lowered to 50 kHz to coexist with SCD30.
// - SCD30 config is only marked applied after successful startup.
// - First valid SCD30 samples are treated as warmup samples and are not used
//   yet for LED/output decisions.
// - If BME280 is present, OLED temperature/humidity now use BME280 values.
// - Verbose serial output now includes BME280 temperature, humidity, pressure, and altitude.
//
// Notes:
// - This is the cleaned production-oriented build of the working 10.1.1 test version.
// - Shelly RGBW2 Gen1 control via HTTP GET: /color/0?mode=color&...
// - Supports LED host as IP, FQDN, or *.local (mDNS)
// - Real wall-clock timestamps require NTP/RTC; we store boot counter + millis-since-boot stamps.


// Version 10.1.3
//
// Update highlights (v10.1.3):
// - Reworked SCD30 startup to follow the proven minimal sequence:
//     * stopPeriodicMeasurement()
//     * softReset()
//     * wait 2 seconds
//     * readFirmwareVersion()
//     * apply config
//     * startPeriodicMeasurement(0)
// - SCD30 read path now uses blockingReadMeasurementData()
//   instead of hard-gating on getDataReady().
// - CO2 polling period increased to 3000 ms.
// - Shared I2C bus forced to 50 kHz with 200 ms timeout.
// - SH1107 display bus speed lowered to 50 kHz to coexist with SCD30.
// - SCD30 config is only marked applied after successful startup.
// - First valid SCD30 samples are treated as warmup samples and are not used
//   yet for LED/output decisions.
// - If BME280 is present, OLED temperature/humidity now use BME280 values.
// - Verbose serial output now includes BME280 temperature, humidity, pressure, and altitude.
//
// Notes:
// - This is the cleaned production-oriented build of the working 10.1.1 test version.
// - Shelly RGBW2 Gen1 control via HTTP GET: /color/0?mode=color&...
// - Supports LED host as IP, FQDN, or *.local (mDNS)
// - Real wall-clock timestamps require NTP/RTC; we store boot counter + millis-since-boot stamps.

// Version 10.0
// Change in OLED - SH1107
// Version 9.1.0
//
// Key improvements (v9.1.0):
// - Refreshed web page + layout
// Key improvements (v9.0.0):
// - Adds SCD30 calibration controls to Web Config UI:
//     * ASC (Auto Self-Calibration) ON/OFF (persisted)
//     * FRC (Forced Recalibration Reference) ppm (persisted as "pending", applied once on next boot, then auto-cleared)
//     * "Calibrate now (400 ppm)" button (applies immediately, records last-cal info)
//     * Warning banner when FRC is pending
//     * ASC uptime (enabled since boot) + last calibration stamp (boot# + uptime)
// Version 8.3.0 
//
// Key improvements (v8.3):
//	SCD30 + BME init happens before Wi-Fi / DNS / mDNS / LED resolve.
//	Wi-Fi and LED resolve happen after sensors are already running.
//	Adds a couple of useful debug timestamps when cfg.verbose is enabled.
// - AsyncWebServer (non-blocking HTTP)
// - Web Status page diagnostics:
//     * ESP32 IP / SSID / RSSI
//     * LED host configured + source (Secrets vs NVS)
//     * LED host resolution state (cached IP / fail)
//     * Latest Shelly update status / HTTP response / last URL sent
//     * CO2 raw vs EMA vs hysteresis-used value + holding flag
//     * Full NVS config dump + per-key source (password masked)
// - AsyncWebServer Config Editor UI:
//     * GET /config -> edit config in browser
//     * POST /config -> save to NVS and reboot
//     * POST /reboot -> reboot
//     * POST /factoryreset -> wipe NVS and reboot
// - Basic HTTP Auth for config/maintenance routes:
//     * user: admin
//     * pass: NVS auth_pass if set, else WiFi password fallback
// - LAN-only guard configurable:
//     * NVS key: lan_only
//     * Default from secrets: LAN_only
// - Boot window combos (first 5s):
//     "!!"            -> wipe NVS (Preferences)
//     "RESET"+Enter   -> wipe NVS
//     "++"            -> enable DEBUG/verbose this boot (temporary)
//     any key         -> enter setup menu next
// - Menu improvements:
//     * Menu item 2 replaces LED host (no append) and sanitizes http:// and paths
//     * 'H' toggles CO2 hysteresis ON/OFF (persisted in NVS)
//     * 'L' toggles LAN-only guard ON/OFF (persisted in NVS)
//
// Notes:
// - Shelly RGBW2 Gen1 control via HTTP GET: /color/0?mode=color&...
// - Supports LED host as IP, FQDN, or *.local (mDNS)
// - SCD30 explicitly startContinuousMeasurement(0) to reduce 0 ppm issues
//    * Calibration instructions shown on Status + Config pages
// //
// Notes:
// - Real wall-clock timestamps require NTP/RTC; we store boot counter + millis-since-boot stamps.
//
// TO DO:
// - OTA updates (/update) with same auth/guard
// - MQTT instead of HTTP
// - More advanced config UI (validation, live apply without reboot)
// - calibration history (ring buffer),
// - graphing CO2 via /status.json,
// - or OTA update page integrated with auth,

// ESP32-PICO-D4 CO2 Ventilation Sensor
// Version 9.0.0 (Consolidated)
//
// v9 additions:
// - SCD30 calibration UI + status:
//   * ASC toggle (persisted)
//   * FRC pending (persisted) + warning banner when pending
//   * "Calibrate now (400 ppm)" button (safe: handled in loop)
//   * ASC uptime + last calibration timestamp/reason (persisted)
// - NTP time sync:
//   * default NTP server = gateway IP (from DHCP) if non-zero
//   * fallback to pool.ntp.org
//   * NTP status shown on status page
// - requireAdmin(): LAN-only guard + Basic Auth enforced consistently
//
// Notes:
// - Shelly RGBW2 Gen1 control via HTTP GET: /color/0?mode=color&...
// - Supports LED host as IP, FQDN, or *.local (mDNS)
// - Sensors are started before WiFi, to reduce SCD30 0ppm warmup symptom.
//
// ------------------------------------------------------------
//
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <math.h>
#include <SdFat.h>
#include <time.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#define WHITE SH110X_WHITE

#include <SensirionI2cScd30.h>
#include <BME280I2C.h>
#include <EnvironmentCalculations.h>

#include "arduino_secrets.h"

// -------- SD logging config --------
// Logging behavior remains configurable from arduino_secrets.h,
// but the SD hardware pins/speed are fixed here to the values
// validated on the working hardware setup.
#ifndef SECRET_SD_LOG_ENABLE
  #define SECRET_SD_LOG_ENABLE false
#endif
#ifndef SECRET_SD_LOG_PERIOD_SEC
  #define SECRET_SD_LOG_PERIOD_SEC 60
#endif
#ifndef SECRET_SD_LOG_ROTATION
  #define SECRET_SD_LOG_ROTATION "DAY"
#endif

#ifndef SECRET_SCREENSAVE_TIME_MINUTES
  #ifdef Screensave_time_minutes
    #define SECRET_SCREENSAVE_TIME_MINUTES Screensave_time_minutes
  #else
    #define SECRET_SCREENSAVE_TIME_MINUTES 0
  #endif
#endif

// Touch wake pin is intentionally defined here in the sketch, not in secrets.
// Set to -1 to disable touch wake until a valid ESP32 touch GPIO is chosen.
static constexpr int16_t TOUCH_WAKE_PIN = 2;

static constexpr uint8_t  SD_CS_PIN   = 4;
static constexpr uint8_t  SD_SCK_PIN  = 18;
static constexpr uint8_t  SD_MISO_PIN = 19;
static constexpr uint8_t  SD_MOSI_PIN = 23;
static constexpr uint32_t SD_SPI_MHZ_ = 1;

#ifndef ENABLE_MDNS
  #define ENABLE_MDNS 1
#endif

#if ENABLE_MDNS
  #include <ESPmDNS.h>
#endif

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

static const char* MY_VERSION = "10.3.5b";

static constexpr float CO2_EMA_ALPHA = 0.20f;
static constexpr float CO2_HYST_PPM  = 25.0f;

// Conservative I2C settings for SCD30
static constexpr uint8_t  I2C_SDA_PIN = 21;
static constexpr uint8_t  I2C_SCL_PIN = 22;
static constexpr uint32_t I2C_FREQ_HZ = 50000;
static constexpr uint32_t I2C_TIMEOUT_MS = 200;

enum class ColorMode : uint8_t {
  AQI = 0,
  LEGACY_WAVELENGTH = 1
};

struct Config {
  String ssid;
  String pass;
  String led_host;
  String hostname;
  int    altitude_m;
  String report;
  String report_txt;
  bool   verbose;

  bool   co2_hyst_enabled;
  ColorMode color_mode;

  String auth_pass;
  bool   lan_only;

  bool     scd30_asc;
  int16_t  scd30_frc_ppm;

  bool     sd_log_enable;
  uint32_t sd_log_period_sec;
  String   sd_rotation_cfg;
  uint16_t sd_keep_files;

  uint16_t screensave_time_minutes;
  int16_t  touch_wake_pin;

  String   ntp_server;
};

static const char* colorModeToStr(ColorMode m) {
  return (m == ColorMode::AQI) ? "AQI" : "LEGACY";
}

float referencePressure = 1013.25f;
float outdoorTemp = 4.7f;
float barometerAltitude = SECRET_BAROMETERAltitude_m;

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET_PIN 33
#define SCREEN_ADDRESS 0x3C
// Keep OLED transfers slow on the shared bus.
Adafruit_SH1107 display(64, 128, &Wire, OLED_RESET_PIN, 50000, 50000);

BME280I2C::Settings bmeSettings(
  BME280::OSR_X1,
  BME280::OSR_X1,
  BME280::OSR_X1,
  BME280::Mode_Forced,
  BME280::StandbyTime_1000ms,
  BME280::Filter_16,
  BME280::SpiEnable_False,
  BME280I2C::I2CAddr_0x76
);
BME280I2C bme(bmeSettings);
bool  bme_sensor_found = false;
bool  bme_has_humidity = false;
float bme_T_s = NAN;
float bme_H_s = NAN;
float bme_P_s = NAN;
float alt_s = NAN;
bool  bme_humidity_warning_printed = false;
String bmeSensorName = "UNKNOWN";

// -------- SD logging --------
SdFs sd;
bool   sd_log_enabled = false;
bool   sd_card_ready = false;
bool   sd_unsynced_notice_printed = false;
bool   sd_write_error_printed = false;
uint32_t sd_log_period_ms = 60000;
String sd_rotation_mode = "OFF";
String sd_current_log_path = "";
uint32_t sd_log_success_count = 0;
uint16_t sd_keep_files = 31;
static uint32_t tSdLog = 0;


#ifndef SCD30_I2C_ADDR_61
  #define SCD30_I2C_ADDR_61 0x61
#endif

SensirionI2cScd30 scd30;
#ifdef NO_ERROR
#undef NO_ERROR
#endif
#define NO_ERROR 0

Preferences preferences;
Config cfg;
static const char* PREF_NS = "my-app";

bool key_ssid=false, key_pass=false, key_led=false, key_hostname=false, key_altitude=false;
bool key_report=false, key_report_txt=false, key_verbose=false, key_color_mode=false, key_hyst=false;
bool key_auth_pass=false, key_lan_only=false;
bool key_scd30_asc=false, key_scd30_frc=false;
bool key_sd_log_enable=false, key_sd_log_period=false, key_sd_rotation=false, key_sd_keep_files=false;
bool key_screensave_minutes=false, key_touch_wake_pin=false;
bool key_last_cal_epoch=false, key_last_cal_reason=false;
bool key_asc_enabled_epoch=false;

bool ledHostFromNVS = false;
bool enterMenuRequested = false;
bool bootDebugRequested = false;

AsyncWebServer server(80);

float scd30_T_s = NAN;
float scd30_H_s = NAN;
float scd30_CO2_s = NAN;

String led_update_status = "";
String ledResolveState   = "unknown";
bool   scd30_config_applied = false;
bool   scd30_runtime_settings_done = false;
bool   scd30_outputs_ready = false;
uint8_t scd30_valid_sample_count = 0;
static constexpr uint8_t SCD30_WARMUP_SAMPLES = 3;
static constexpr float   SCD30_CO2_MIN_VALID_PPM = 250.0f;
static uint32_t tScd30RuntimeCfg = 0;
static constexpr uint32_t SCD30_RUNTIME_CFG_RETRY_MS = 10000;

float red = 0, green = 0, blue = 0;
int lastR = -1, lastG = -1, lastB = -1;

float co2_ema = NAN;
float co2_for_color = NAN;

IPAddress led_cached_ip(0, 0, 0, 0);
bool      led_cached_ip_valid = false;

int      lastShellyHttpCode = 0;
String   lastShellyHostUsed = "";
String   lastShellyUrlSent  = "";
uint32_t lastShellyMillis   = 0;
uint32_t lastResolveMillis  = 0;

float    lastCO2raw         = NAN;
float    lastCO2RejectedRaw = NAN;
float    lastCO2ema         = NAN;
float    lastCO2forColor    = NAN;
bool     lastCO2Adjusted    = false;

static uint32_t tCO2 = 0;
static uint32_t tOLED = 0;
static uint32_t tShelly = 0;
static uint32_t tWifiRetry = 0;
static uint32_t tResolveRetry = 0;
static uint32_t tNtpPoll = 0;

static constexpr uint32_t CO2_PERIOD_MS    = 3000;
static constexpr uint32_t OLED_PERIOD_MS   = 500;
static constexpr uint32_t SHELLY_PERIOD_MS = 2500;
static constexpr uint32_t WIFI_RETRY_MS    = 10000;

static constexpr uint32_t BOOT_WINDOW_MS = 5000;

static uint32_t ledResolveNotBefore = 0;
static constexpr uint32_t RESOLVER_WARMUP_MS = 2000;
static constexpr uint32_t STARTUP_RESOLVE_MAX_MS = 15000;
static constexpr uint32_t STARTUP_RESOLVE_RETRY_MS = 500;
static constexpr uint32_t RUNTIME_RESOLVE_RETRY_MS = 10000;
static constexpr uint32_t NTP_POLL_MS = 2000;

volatile bool g_requestFrcNow400 = false;
bool frcPendingBanner = false;

time_t lastCalEpoch = 0;
String lastCalReason = "";
time_t ascEnabledEpoch = 0;

bool   ntpConfigured = false;
bool   ntpSynced = false;
time_t ntpLastSyncEpoch = 0;
String ntpServerUsed = "";

// OLED screensaver / touch-wake runtime state
static bool     oled_is_on = true;
static uint32_t oled_awake_deadline_ms = 0;
static bool     oled_touch_enabled = false;
static volatile bool g_oled_touch_irq = false;
static uint16_t oled_touch_baseline = 0;
static uint16_t oled_touch_threshold = 0;
static uint32_t tOledTouchCheck = 0;
static uint32_t oled_last_wake_ms = 0;
static constexpr uint32_t OLED_TOUCH_CHECK_MS = 100;
static constexpr uint32_t OLED_TOUCH_WAKE_DEBOUNCE_MS = 800;

// RAM history buffer for lightweight web graphs
struct HistorySample {
  uint32_t t;
  float co2;
  float temp;
  float pressure;
  bool epoch_valid;
};

static constexpr uint16_t HISTORY_CAPACITY = 180;
static HistorySample g_history[HISTORY_CAPACITY];
static uint16_t g_history_head = 0;
static uint16_t g_history_count = 0;

static inline bool strYes(const String& s) {
  return (s.length() > 0 && (s[0] == 'Y' || s[0] == 'y'));
}

static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline float lerpf(float a, float b, float t) {
  return a + (b - a) * t;
}

static bool strEqNoCase(const String& a, const char* b) {
  String x = a; x.trim(); x.toLowerCase();
  String y = String(b); y.trim(); y.toLowerCase();
  return x == y;
}

static inline bool debugFromSecretsDefault() {
#ifdef DEBUG
  String d = String(DEBUG);
  d.trim(); d.toLowerCase();
  return (d == "true" || d == "1" || d == "yes" || d == "y" || d == "on");
#else
  return false;
#endif
}

static inline bool lanOnlyFromSecretsDefault() {
#ifdef LAN_only
  return (bool)LAN_only;
#else
  return true;
#endif
}

String sanitizeHost(String h) {
  h.trim();
  h.replace("http://", "");
  h.replace("https://", "");
  int slash = h.indexOf('/');
  if (slash >= 0) h = h.substring(0, slash);
  h.trim();
  return h;
}


static void logScd30Error(const char* where, int16_t err) {
  if (!cfg.verbose) return;
  char buf[128];
  errorToString(err, buf, sizeof(buf));
  Serial.print("SCD30 ");
  Serial.print(where);
  Serial.print(" err=");
  Serial.print(err);
  Serial.print(" (");
  Serial.print(buf);
  Serial.println(")");
}

static void oledTurnOnIfNeeded(bool redraw);
static void oledTurnOffIfNeeded();

void oledMessage(const String& a, const String& b = "", const String& c = "") {
  oledTurnOnIfNeeded(false);
  if (!display.width()) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println(a);
  if (b.length()) display.println(b);
  if (c.length()) display.println(c);
  display.display();
}

void showBmeStartupInfoOLED(uint32_t ms = 1800) {
  oledTurnOnIfNeeded(false);
  if (!display.width()) return;

  String line1 = bmeSensorName + " ready";
  String line2 = isfinite(bme_P_s) ? (String("P ") + String((int)lroundf(bme_P_s)) + " hPa") : String("P n/a");
  String line3 = isfinite(alt_s) ? (String("Alt ") + String((int)lroundf(alt_s)) + " m") : String("Alt n/a");

  oledMessage(line1, line2, line3);
  delay(ms);
}


static String getSdLogIndicator() {
  if (!sd_log_enabled || !sd_card_ready) return "   ";
  if (sd_log_success_count == 0) return "...";
  switch ((sd_log_success_count - 1) % 4) {
    case 0: return "...";
    case 1: return "o..";
    case 2: return ".o.";
    default: return "..o";
  }
}

static void historyAddSample(float co2, float temp, float pressure) {
  if (!isfinite(co2)) return;

  HistorySample sample;
  sample.epoch_valid = timeIsValid();
  sample.t = sample.epoch_valid ? (uint32_t)time(nullptr) : (millis() / 1000UL);
  sample.co2 = co2;
  sample.temp = temp;
  sample.pressure = pressure;

  if (g_history_count < HISTORY_CAPACITY) {
    const uint16_t idx = (g_history_head + g_history_count) % HISTORY_CAPACITY;
    g_history[idx] = sample;
    g_history_count++;
  } else {
    g_history[g_history_head] = sample;
    g_history_head = (g_history_head + 1) % HISTORY_CAPACITY;
  }
}

static uint16_t historyCount() {
  return g_history_count;
}

static uint16_t historyPhysicalIndex(uint16_t logicalIndex) {
  return (g_history_head + logicalIndex) % HISTORY_CAPACITY;
}

static void printJsonFloat(Print& out, float v, uint8_t decimals = 1) {
  if (isfinite(v)) out.print(String((double)v, (unsigned int)decimals));
  else out.print(F("null"));
}

static void sendHistoryJson(AsyncWebServerRequest* request) {
  uint16_t limit = 120;
  if (request->hasParam("points")) {
    int v = request->getParam("points")->value().toInt();
    if (v > 0) limit = (uint16_t)v;
  }
  if (limit > HISTORY_CAPACITY) limit = HISTORY_CAPACITY;

  AsyncResponseStream* response = request->beginResponseStream("application/json; charset=utf-8");
  response->print(F("{\"capacity\":"));
  response->print(HISTORY_CAPACITY);
  response->print(F(",\"count\":"));
  response->print(g_history_count);
  response->print(F(",\"points\":["));

  const uint16_t total = g_history_count;
  const uint16_t start = (total > limit) ? (total - limit) : 0;
  bool first = true;
  for (uint16_t i = start; i < total; ++i) {
    const HistorySample& s = g_history[historyPhysicalIndex(i)];
    if (!first) response->print(',');
    first = false;
    response->print(F("{\"t\":"));
    response->print(s.t);
    response->print(F(",\"tv\":"));
    response->print(s.epoch_valid ? 1 : 0);
    response->print(F(",\"co2\":"));
    printJsonFloat(*response, s.co2, 1);
    response->print(F(",\"temp\":"));
    printJsonFloat(*response, s.temp, 1);
    response->print(F(",\"pressure\":"));
    printJsonFloat(*response, s.pressure, 1);
    response->print('}');
  }
  response->print(F("]}"));
  request->send(response);
}

void flushSerialInput() {
  while (Serial.available() > 0) (void)Serial.read();
}

char readCharBlocking(uint32_t timeoutMs) {
  uint32_t start = millis();
  while ((millis() - start) < timeoutMs) {
    if (Serial.available() > 0) {
      char c = (char)Serial.read();
      if (c == '\r' || c == '\n') continue;
      return c;
    }
    delay(10);
  }
  return 0;
}

String readLineBlocking(uint32_t timeoutMs) {
  uint32_t start = millis();
  String line;
  while ((millis() - start) < timeoutMs) {
    while (Serial.available() > 0) {
      char c = (char)Serial.read();
      if (c == '\r') continue;
      if (c == '\n') return line;
      line += c;
    }
    delay(10);
  }
  return "";
}

static bool timeIsValid() {
  time_t now = time(nullptr);
  return (now > 1577836800);
}

static String fmtTime(time_t t) {
  if (t <= 0) return "n/a";
  struct tm tmv;
  localtime_r(&t, &tmv);
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tmv);
  return String(buf);
}


static inline String csvFloatOrBlank(float v, uint8_t decimals = 1) {
  return isfinite(v) ? String((double)v, (unsigned int)decimals) : String("");
}

static inline bool sdLoggingEnabledFromSecrets() {
  return (bool)SECRET_SD_LOG_ENABLE;
}

static inline String sdRotationModeFromSecrets() {
  String s = String(SECRET_SD_LOG_ROTATION);
  s.trim();
  s.toUpperCase();
  if (s != "DAY" && s != "MONTH" && s != "OFF") s = "OFF";
  return s;
}

static inline uint32_t sdLogPeriodMsFromSecrets() {
  uint32_t sec = (uint32_t)SECRET_SD_LOG_PERIOD_SEC;
  if (sec == 0) sec = 60;
  return sec * 1000UL;
}

static inline uint16_t sdKeepFilesFromSecrets() {
#ifdef SECRET_SD_KEEP_FILES
  uint32_t n = (uint32_t)SECRET_SD_KEEP_FILES;
  if (n > 3650UL) n = 3650UL;
  return (uint16_t)n;
#else
  return 31;
#endif
}

static inline uint16_t screensaveMinutesFromSecrets() {
  uint32_t n = (uint32_t)SECRET_SCREENSAVE_TIME_MINUTES;
  if (n > 1440UL) n = 1440UL;
  return (uint16_t)n;
}

static inline int16_t touchWakePinFromSecrets() {
  return TOUCH_WAKE_PIN;
}

static void IRAM_ATTR onOledTouchWakeISR() {
  g_oled_touch_irq = true;
}

static void oledHardwareOn() {
  display.oled_command(SH110X_DISPLAYON);
}

static void oledHardwareOff() {
  display.oled_command(SH110X_DISPLAYOFF);
}

static void oledTurnOnIfNeeded(bool redraw = true) {
  if (oled_is_on) return;
  oledHardwareOn();
  oled_is_on = true;
  if (redraw) display.display();
}

static void oledTurnOffIfNeeded() {
  if (!oled_is_on) return;
  oledHardwareOff();
  oled_is_on = false;
}

static void resetOledAwakeDeadline() {
  if (cfg.screensave_time_minutes == 0) {
    oled_awake_deadline_ms = 0;
    oledTurnOnIfNeeded(true);
    return;
  }
  oled_awake_deadline_ms = millis() + ((uint32_t)cfg.screensave_time_minutes * 60000UL);
  oledTurnOnIfNeeded(true);
}

static bool calibrateOledTouchWake() {
  oled_touch_enabled = false;
  g_oled_touch_irq = false;
  oled_touch_baseline = 0;
  oled_touch_threshold = 0;

  bool validTouchPin = false;
  switch (cfg.touch_wake_pin) {
    case 0:
    case 2:
    case 4:
    case 12:
    case 13:
    case 14:
    case 15:
    case 27:
    case 32:
    case 33:
      validTouchPin = true;
      break;
    default:
      validTouchPin = false;
      break;
  }

  if (!validTouchPin) {
    if (cfg.touch_wake_pin >= 0) {
      Serial.print("OLED touch wake disabled: GPIO");
      Serial.print(cfg.touch_wake_pin);
      Serial.println(" is not an ESP32 touch pin.");
    }
    return false;
  }

  uint32_t sum = 0;
  const uint8_t samples = 12;
  for (uint8_t i = 0; i < samples; ++i) {
    sum += (uint32_t)touchRead((uint8_t)cfg.touch_wake_pin);
    delay(20);
  }

  oled_touch_baseline = (uint16_t)(sum / samples);
  uint32_t thr = ((uint32_t)oled_touch_baseline * 70UL) / 100UL;
  if (thr < 1UL) thr = 1UL;
  oled_touch_threshold = (uint16_t)thr;

  touchAttachInterrupt((uint8_t)cfg.touch_wake_pin, onOledTouchWakeISR, oled_touch_threshold);
  oled_touch_enabled = true;

  Serial.print("OLED touch wake: GPIO");
  Serial.print(cfg.touch_wake_pin);
  Serial.print(" baseline=");
  Serial.print(oled_touch_baseline);
  Serial.print(" threshold=");
  Serial.println(oled_touch_threshold);
  return true;
}

static void handleOledScreensaverNonBlocking() {
  if (cfg.screensave_time_minutes == 0) {
    oledTurnOnIfNeeded(false);
    return;
  }

  const uint32_t now = millis();

  if (oled_touch_enabled && (now - tOledTouchCheck >= OLED_TOUCH_CHECK_MS)) {
    tOledTouchCheck = now;
    if (g_oled_touch_irq && (now - oled_last_wake_ms >= OLED_TOUCH_WAKE_DEBOUNCE_MS)) {
      g_oled_touch_irq = false;
      oled_last_wake_ms = now;
      resetOledAwakeDeadline();
      if (cfg.verbose) {
        Serial.print("OLED wake by touch on GPIO");
        Serial.println(cfg.touch_wake_pin);
      }
    }
  }

  if (oled_is_on && oled_awake_deadline_ms != 0 && (int32_t)(now - oled_awake_deadline_ms) >= 0) {
    oledTurnOffIfNeeded();
    if (cfg.verbose) Serial.println("OLED screensaver: display OFF");
  }
}

static String makeSdLogKey() {
  if (!timeIsValid()) return "UNSYNCED";

  time_t now = time(nullptr);
  struct tm tmv;
  localtime_r(&now, &tmv);

  char buf[20];
  if (sd_rotation_mode == "DAY") {
    strftime(buf, sizeof(buf), "%Y-%m-%d", &tmv);
    return String(buf);
  }
  if (sd_rotation_mode == "MONTH") {
    strftime(buf, sizeof(buf), "%Y-%m", &tmv);
    return String(buf);
  }
  return "STATIC";
}

static String makeSdLogPath(const String& key) {
  if (key == "UNSYNCED") return "/logs/co2_unsynced.csv";
  if (key == "STATIC")   return "/logs/co2_log.csv";
  return "/logs/co2_" + key + ".csv";
}


static bool isManagedLogFileName(const char* name) {
  if (!name || !name[0]) return false;
  String s(name);
  s.toLowerCase();
  return s.startsWith("co2_") && s.endsWith(".csv");
}

static uint16_t countManagedLogFiles() {
  FsFile dir;
  if (!dir.open("/logs")) return 0;

  uint16_t count = 0;
  FsFile entry;
  while (entry.openNext(&dir, O_RDONLY)) {
    char name[96] = {0};
    entry.getName(name, sizeof(name));
    if (!entry.isDir() && isManagedLogFileName(name)) {
      count++;
    }
    entry.close();
  }
  dir.close();
  return count;
}

static bool pruneOneOldestLogFile(const String& currentPath) {
  FsFile dir;
  if (!dir.open("/logs")) return false;

  String oldestPath = "";
  String oldestName = "";

  FsFile entry;
  while (entry.openNext(&dir, O_RDONLY)) {
    char name[96] = {0};
    entry.getName(name, sizeof(name));
    if (!entry.isDir() && isManagedLogFileName(name)) {
      String fullPath = String("/logs/") + name;
      if (fullPath != currentPath) {
        if (oldestName.length() == 0 || String(name) < oldestName) {
          oldestName = String(name);
          oldestPath = fullPath;
        }
      }
    }
    entry.close();
  }
  dir.close();

  if (oldestPath.length() == 0) return false;

  bool ok = sd.remove(oldestPath.c_str());
  if (cfg.verbose) {
    Serial.print(ok ? "SD prune removed -> " : "SD prune FAILED -> ");
    Serial.println(oldestPath);
  }
  return ok;
}

static void pruneOldSdLogsIfNeeded(const String& currentPath) {
  if (!sd_log_enabled || !sd_card_ready) return;
  if (sd_keep_files == 0) return;

  uint16_t count = countManagedLogFiles();
  while (count > sd_keep_files) {
    if (!pruneOneOldestLogFile(currentPath)) break;
    count = countManagedLogFiles();
  }
}

static void printSdConfigSummary() {
  Serial.print("SD logging: ");
  Serial.println(sd_log_enabled ? "ENABLED" : "DISABLED");
  if (!sd_log_enabled) return;

  Serial.print("SD CS pin: "); Serial.println(SD_CS_PIN);
  Serial.print("SD SPI pins SCK/MISO/MOSI: ");
  Serial.print(SD_SCK_PIN); Serial.print("/");
  Serial.print(SD_MISO_PIN); Serial.print("/");
  Serial.println(SD_MOSI_PIN);
  Serial.print("SD SPI clock: "); Serial.print((int)SD_SPI_MHZ_); Serial.println(" MHz");
  Serial.print("SD log period: "); Serial.print(sd_log_period_ms / 1000UL); Serial.println(" s");
  Serial.print("SD rotation: "); Serial.println(sd_rotation_mode);
  Serial.print("SD keep files: "); Serial.println(sd_keep_files);
}

static bool initSdLogging() {
  sd_log_enabled = cfg.sd_log_enable;
  sd_log_period_ms = cfg.sd_log_period_sec * 1000UL;
  if (sd_log_period_ms == 0) sd_log_period_ms = 60000UL;
  sd_rotation_mode = cfg.sd_rotation_cfg;
  sd_keep_files = cfg.sd_keep_files;

  printSdConfigSummary();

  if (!sd_log_enabled) return false;

  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  delay(20);

  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

  SdSpiConfig cfgSd(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(SD_SPI_MHZ_));
  if (!sd.begin(cfgSd)) {
    Serial.println("SD init FAILED.");
    sd_card_ready = false;
    return false;
  }

  if (!sd.exists("/logs")) {
    if (!sd.mkdir("/logs")) {
      Serial.println("SD: failed to create /logs directory.");
      sd_card_ready = false;
      return false;
    }
  }

  sd_card_ready = true;
  Serial.println("SD card ready. Logs path: /logs");
  oledMessage("SD card ready", "/logs", sd_rotation_mode);
  delay(800);
  return true;
}

static bool appendCsvHeaderIfNeeded(FsFile& file, bool newFile) {
  if (!newFile && file.size() > 0) return true;

  file.println("hostname,timestamp,epoch,millis,co2_ppm,scd30_temp_c,scd30_rh_pct,bmx_temp_c,bmx_rh_pct,pressure_hpa,altitude_m,display_temp_c,display_rh_pct,ambient_sensor");
  return file.getWriteError() == 0;
}

static bool appendLogLineToSd(float displayTemp, float displayHum) {
  if (!sd_log_enabled || !sd_card_ready) return false;

  const String key = makeSdLogKey();
  const String path = makeSdLogPath(key);

  if (key == "UNSYNCED" && !sd_unsynced_notice_printed && cfg.verbose) {
    Serial.println("SD logging: time not synced yet, writing to /logs/co2_unsynced.csv");
    sd_unsynced_notice_printed = true;
  }

  const bool newFile = !sd.exists(path.c_str());
  FsFile file;
  if (!file.open(path.c_str(), O_WRONLY | O_CREAT | O_APPEND)) {
    if (!sd_write_error_printed || cfg.verbose) {
      Serial.print("SD open failed: ");
      Serial.println(path);
      sd_write_error_printed = true;
    }
    return false;
  }

  if (!appendCsvHeaderIfNeeded(file, newFile)) {
    file.close();
    if (!sd_write_error_printed || cfg.verbose) {
      Serial.println("SD header write failed.");
      sd_write_error_printed = true;
    }
    return false;
  }

  time_t now = time(nullptr);
  String ts = timeIsValid() ? fmtTime(now) : String("");

  String csvLine;
  csvLine.reserve(192);
  csvLine += cfg.hostname; csvLine += ',';
  csvLine += ts; csvLine += ',';
  csvLine += String((long)(timeIsValid() ? now : 0)); csvLine += ',';
  csvLine += String(millis()); csvLine += ',';
  csvLine += csvFloatOrBlank(scd30_CO2_s, 1); csvLine += ',';
  csvLine += csvFloatOrBlank(scd30_T_s, 1); csvLine += ',';
  csvLine += csvFloatOrBlank(scd30_H_s, 1); csvLine += ',';
  csvLine += csvFloatOrBlank(bme_T_s, 1); csvLine += ',';
  if (bme_has_humidity) csvLine += csvFloatOrBlank(bme_H_s, 1);
  csvLine += ',';
  csvLine += csvFloatOrBlank(bme_P_s, 1); csvLine += ',';
  csvLine += csvFloatOrBlank(alt_s, 1); csvLine += ',';
  csvLine += csvFloatOrBlank(displayTemp, 1); csvLine += ',';
  csvLine += csvFloatOrBlank(displayHum, 1); csvLine += ',';
  csvLine += bmeSensorName;

  file.println(csvLine);

  const bool ok = (file.getWriteError() == 0);
  file.close();

  if (ok) {
    sd_current_log_path = path;
    sd_log_success_count++;
    sd_write_error_printed = false;
    if (cfg.verbose) {
      Serial.print("SD log append -> ");
      Serial.println(sd_current_log_path);
      Serial.println(csvLine);
    }
  } else if (!sd_write_error_printed || cfg.verbose) {
    Serial.print("SD write failed: ");
    Serial.println(path);
    sd_write_error_printed = true;
  }

  return ok;
}

static void maybeLogSensorsToSd(float displayTemp, float displayHum) {
  if (!sd_log_enabled || !sd_card_ready) return;
  if (!scd30_outputs_ready) return;

  const uint32_t nowMs = millis();
  if (nowMs - tSdLog < sd_log_period_ms) return;
  tSdLog = nowMs;

  appendLogLineToSd(displayTemp, displayHum);
}

static void recordCalibrationApplied(const String& reason) {
  time_t now = timeIsValid() ? time(nullptr) : 0;
  lastCalEpoch = now;
  lastCalReason = reason;

  preferences.begin(PREF_NS, false);
  preferences.putLong("last_cal_epoch", (long)lastCalEpoch);
  preferences.putString("last_cal_reason", lastCalReason);
  preferences.end();

  key_last_cal_epoch = true;
  key_last_cal_reason = true;

  if (cfg.verbose) {
    Serial.print("Calibration recorded: ");
    Serial.print(reason);
    Serial.print(" @ ");
    Serial.println(fmtTime(lastCalEpoch));
  }
}

void wipePreferencesNVS() {
  preferences.begin(PREF_NS, false);
  preferences.clear();
  preferences.end();

  led_cached_ip_valid = false;
  led_cached_ip = IPAddress(0,0,0,0);

  Serial.println("FACTORY RESET: Preferences wiped (NVS cleared).");
  oledMessage("FACTORY RESET", "NVS cleared", "Rebooting...");
  delay(800);
  ESP.restart();
}

void bootWindow() {
  Serial.println();
  Serial.println("Boot window: 5 seconds");
  Serial.println("  !!            -> wipe NVS");
  Serial.println("  RESET + Enter -> wipe NVS");
  Serial.println("  ++            -> enable DEBUG this boot");
  Serial.println("  any key       -> enter setup menu next");
  oledMessage("Booting...", "See serial", "115200 baud");

  uint32_t start = millis();
  String buf;

  while ((millis() - start) < BOOT_WINDOW_MS) {
    while (Serial.available() > 0) {
      char c = (char)Serial.read();
      if (c == '\r') continue;

      enterMenuRequested = true;

      if (c == '\n') {
        String line = buf;
        buf = "";
        line.trim();
        if (line == "RESET") wipePreferencesNVS();
      } else {
        buf += c;
        if (buf.indexOf("!!") >= 0) wipePreferencesNVS();
        if (buf.indexOf("++") >= 0) { bootDebugRequested = true; buf = "++"; }
        if (buf.length() > 32) buf.remove(0, buf.length() - 32);
      }
    }
    delay(10);
  }

  flushSerialInput();
}

static ColorMode parseColorMode(const String& s) {
  if (strEqNoCase(s, "LEGACY")) return ColorMode::LEGACY_WAVELENGTH;
  if (strEqNoCase(s, "WAVELENGTH")) return ColorMode::LEGACY_WAVELENGTH;
  if (strEqNoCase(s, "OLD")) return ColorMode::LEGACY_WAVELENGTH;
  return ColorMode::AQI;
}

void loadConfig() {
  preferences.begin(PREF_NS, true);

  key_ssid       = preferences.isKey("ssid");
  key_pass       = preferences.isKey("ssid_pwd");
  key_led        = preferences.isKey("led");
  key_hostname   = preferences.isKey("hostname");
  key_altitude   = preferences.isKey("altitude");
  key_report     = preferences.isKey("report");
  key_report_txt = preferences.isKey("report_txt");
  key_verbose    = preferences.isKey("verbose");
  key_color_mode = preferences.isKey("color_mode");
  key_hyst       = preferences.isKey("hyst");
  key_auth_pass  = preferences.isKey("auth_pass");
  key_lan_only   = preferences.isKey("lan_only");
  key_scd30_asc  = preferences.isKey("scd30_asc");
  key_scd30_frc  = preferences.isKey("scd30_frc");
  key_sd_log_enable = preferences.isKey("sd_log_en");
  key_sd_log_period = preferences.isKey("sd_log_sec");
  key_sd_rotation   = preferences.isKey("sd_rotate");
  key_sd_keep_files = preferences.isKey("sd_keep");
  key_screensave_minutes = preferences.isKey("scrsave_min");
  key_touch_wake_pin     = preferences.isKey("touch_wake");
  key_last_cal_epoch   = preferences.isKey("last_cal_epoch");
  key_last_cal_reason  = preferences.isKey("last_cal_reason");
  key_asc_enabled_epoch = preferences.isKey("asc_en_epoch");

  ledHostFromNVS = key_led;

  cfg.ssid       = preferences.getString("ssid", SECRET_SSID);
  cfg.pass       = preferences.getString("ssid_pwd", SECRET_PASS);
  cfg.led_host   = preferences.getString("led", SECRET_LED_IP);
  cfg.hostname   = preferences.getString("hostname", SECRET_HOSTNAME);
  cfg.altitude_m = preferences.getInt("altitude", SECRET_BAROMETERAltitude_m);
  cfg.report     = preferences.getString("report", SECRET_REPORT);
  cfg.report_txt = preferences.getString("report_txt", SECRET_REPORT_TXT);
  cfg.verbose    = preferences.getBool("verbose", debugFromSecretsDefault());
  String cm      = preferences.getString("color_mode", "AQI");
  cfg.color_mode = parseColorMode(cm);
  cfg.co2_hyst_enabled = preferences.getBool("hyst", true);
  cfg.lan_only = preferences.getBool("lan_only", lanOnlyFromSecretsDefault());
  cfg.auth_pass = preferences.getString("auth_pass", "");
  if (cfg.auth_pass.length() == 0) cfg.auth_pass = cfg.pass;

  cfg.scd30_asc     = preferences.getBool("scd30_asc", true);
  cfg.scd30_frc_ppm = (int16_t)preferences.getInt("scd30_frc", -1);

  cfg.sd_log_enable = preferences.getBool("sd_log_en", sdLoggingEnabledFromSecrets());
  cfg.sd_log_period_sec = (uint32_t)preferences.getULong("sd_log_sec", (uint32_t)SECRET_SD_LOG_PERIOD_SEC);
  if (cfg.sd_log_period_sec == 0) cfg.sd_log_period_sec = 60;
  cfg.sd_rotation_cfg = preferences.getString("sd_rotate", sdRotationModeFromSecrets());
  cfg.sd_rotation_cfg.trim();
  cfg.sd_rotation_cfg.toUpperCase();
  if (cfg.sd_rotation_cfg != "DAY" && cfg.sd_rotation_cfg != "MONTH" && cfg.sd_rotation_cfg != "OFF") cfg.sd_rotation_cfg = "OFF";
  cfg.sd_keep_files = (uint16_t)preferences.getUInt("sd_keep", (uint32_t)sdKeepFilesFromSecrets());
  if (cfg.sd_keep_files > 3650U) cfg.sd_keep_files = 3650U;
  cfg.screensave_time_minutes = (uint16_t)preferences.getUInt("scrsave_min", screensaveMinutesFromSecrets());
  cfg.touch_wake_pin          = (int16_t)preferences.getInt("touch_wake", (int32_t)touchWakePinFromSecrets());

  if (cfg.scd30_frc_ppm >= 0 && cfg.scd30_asc) {
    if (cfg.verbose) Serial.println("Config: FRC pending -> disabling ASC (temporary safety)");
    cfg.scd30_asc = false;
  }

  lastCalEpoch  = (time_t)preferences.getLong("last_cal_epoch", 0);
  lastCalReason = preferences.getString("last_cal_reason", "");
  ascEnabledEpoch = (time_t)preferences.getLong("asc_en_epoch", 0);

  preferences.end();

  if (bootDebugRequested) cfg.verbose = true;

  cfg.led_host = sanitizeHost(cfg.led_host);
  cfg.hostname.trim();

  if (cfg.verbose) {
    Serial.print("LED host in use: "); Serial.println(cfg.led_host);
    Serial.print("LED host source: "); Serial.println(ledHostFromNVS ? "NVS override" : "Secrets default");
    Serial.print("Verbose: "); Serial.println(cfg.verbose ? "true" : "false");
    Serial.print("Color mode: "); Serial.println(colorModeToStr(cfg.color_mode));
    Serial.print("CO2 hysteresis: "); Serial.println(cfg.co2_hyst_enabled ? "ON" : "OFF");
    Serial.print("LAN-only guard: "); Serial.println(cfg.lan_only ? "ON" : "OFF");
    Serial.print("Auth pass source: "); Serial.println(key_auth_pass ? "NVS (auth_pass)" : "WiFi password fallback");
    Serial.print("SCD30 ASC: "); Serial.println(cfg.scd30_asc ? "ON" : "OFF");
    Serial.print("SCD30 FRC pending: "); Serial.println(cfg.scd30_frc_ppm);
    Serial.print("SD logging: "); Serial.println(cfg.sd_log_enable ? "ON" : "OFF");
    Serial.print("SD log period(s): "); Serial.println(cfg.sd_log_period_sec);
    Serial.print("SD rotation: "); Serial.println(cfg.sd_rotation_cfg);
    Serial.print("SD keep files: "); Serial.println(cfg.sd_keep_files);
    Serial.print("Last cal: "); Serial.print(fmtTime(lastCalEpoch)); Serial.print(" | "); Serial.println(lastCalReason);
  }
}

void saveConfigIfChanged(const Config& oldCfg) {
  preferences.begin(PREF_NS, false);

  if (cfg.ssid != oldCfg.ssid) preferences.putString("ssid", cfg.ssid);
  if (cfg.pass != oldCfg.pass) preferences.putString("ssid_pwd", cfg.pass);
  if (cfg.led_host != oldCfg.led_host) preferences.putString("led", cfg.led_host);
  if (cfg.hostname != oldCfg.hostname) preferences.putString("hostname", cfg.hostname);
  if (cfg.altitude_m != oldCfg.altitude_m) preferences.putInt("altitude", cfg.altitude_m);
  if (cfg.report != oldCfg.report) preferences.putString("report", cfg.report);
  if (cfg.report_txt != oldCfg.report_txt) preferences.putString("report_txt", cfg.report_txt);
  if (cfg.verbose != oldCfg.verbose) preferences.putBool("verbose", cfg.verbose);

  if (cfg.color_mode != oldCfg.color_mode) {
    preferences.putString("color_mode", colorModeToStr(cfg.color_mode));
  }

  if (cfg.co2_hyst_enabled != oldCfg.co2_hyst_enabled) {
    preferences.putBool("hyst", cfg.co2_hyst_enabled);
  }

  if (cfg.lan_only != oldCfg.lan_only) {
    preferences.putBool("lan_only", cfg.lan_only);
  }

  if (cfg.auth_pass != oldCfg.auth_pass) {
    preferences.putString("auth_pass", cfg.auth_pass);
  }

  if (cfg.scd30_asc != oldCfg.scd30_asc) {
    preferences.putBool("scd30_asc", cfg.scd30_asc);
    if (cfg.scd30_asc) {
      time_t now = timeIsValid() ? time(nullptr) : 0;
      preferences.putLong("asc_en_epoch", (long)now);
      ascEnabledEpoch = now;
      key_asc_enabled_epoch = true;
    }
  }

  if (cfg.scd30_frc_ppm != oldCfg.scd30_frc_ppm) {
    preferences.putInt("scd30_frc", (int)cfg.scd30_frc_ppm);
  }

  if (cfg.sd_log_enable != oldCfg.sd_log_enable) {
    preferences.putBool("sd_log_en", cfg.sd_log_enable);
  }

  if (cfg.sd_log_period_sec != oldCfg.sd_log_period_sec) {
    preferences.putULong("sd_log_sec", (unsigned long)cfg.sd_log_period_sec);
  }

  if (cfg.sd_rotation_cfg != oldCfg.sd_rotation_cfg) {
    preferences.putString("sd_rotate", cfg.sd_rotation_cfg);
  }
  if (cfg.sd_keep_files != oldCfg.sd_keep_files) {
    preferences.putUInt("sd_keep", (uint32_t)cfg.sd_keep_files);
  }

  if (cfg.screensave_time_minutes != oldCfg.screensave_time_minutes) {
    preferences.putUInt("scrsave_min", (uint32_t)cfg.screensave_time_minutes);
  }

  if (cfg.touch_wake_pin != oldCfg.touch_wake_pin) {
    preferences.putInt("touch_wake", (int32_t)cfg.touch_wake_pin);
  }

  preferences.end();
}

#if ENABLE_MDNS
bool mdnsStart() {
  if (!MDNS.begin(cfg.hostname.c_str())) {
    Serial.println("mDNS: FAILED to start (ESP mDNS).");
    return false;
  }
  MDNS.addService("http", "tcp", 80);
  Serial.println("mDNS: started (ESP mDNS).");
  return true;
}
#endif

void wifiConnectBlocking(uint32_t timeoutMs) {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(cfg.hostname.c_str());

  Serial.print("Connecting to SSID: "); Serial.println(cfg.ssid);
  oledMessage("Connecting WiFi...", cfg.ssid);

  WiFi.begin(cfg.ssid.c_str(), cfg.pass.c_str());

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeoutMs) delay(50);

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected.");
    ledResolveNotBefore = millis() + RESOLVER_WARMUP_MS;

    if (cfg.verbose) {
      Serial.print("DNS0: "); Serial.println(WiFi.dnsIP(0));
      Serial.print("DNS1: "); Serial.println(WiFi.dnsIP(1));
      Serial.print("GW  : "); Serial.println(WiFi.gatewayIP());
    }

    Serial.print("SSID: "); Serial.println(WiFi.SSID());
    Serial.print("IP  : "); Serial.println(WiFi.localIP());
    Serial.print("RSSI: "); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
#if ENABLE_MDNS
    Serial.print("ESP mDNS: http://"); Serial.print(cfg.hostname); Serial.println(".local/");
#endif
  } else {
    Serial.println("WiFi connect timeout.");
    oledMessage("WiFi timeout", "Will retry...");
  }
}

void wifiEnsureConnectedNonBlocking() {
  if (WiFi.status() == WL_CONNECTED) return;

  uint32_t now = millis();
  if (now - tWifiRetry < WIFI_RETRY_MS) return;
  tWifiRetry = now;

  Serial.println("WiFi disconnected, retrying...");
  WiFi.disconnect(true);
  WiFi.setHostname(cfg.hostname.c_str());
  WiFi.begin(cfg.ssid.c_str(), cfg.pass.c_str());

  ledResolveNotBefore = millis() + RESOLVER_WARMUP_MS;
  delay(200);
}

#if ENABLE_MDNS
void mdnsEnsureRunning() {
  static bool started = false;
  if (WiFi.status() != WL_CONNECTED) { started = false; return; }
  if (!started && millis() >= ledResolveNotBefore) started = mdnsStart();
}
#endif

static void ntpConfigureIfNeeded() {
  if (ntpConfigured) return;
  if (WiFi.status() != WL_CONNECTED) return;

  IPAddress gw = WiFi.gatewayIP();
  String server = (gw[0] || gw[1] || gw[2] || gw[3]) ? gw.toString() : String("pool.ntp.org");

  cfg.ntp_server = server;
  ntpServerUsed = server;

  configTime(0, 0, cfg.ntp_server.c_str(), "pool.ntp.org");

  ntpConfigured = true;
  ntpSynced = false;
  ntpLastSyncEpoch = 0;

  if (cfg.verbose) {
    Serial.print("NTP configured. Server="); Serial.println(cfg.ntp_server);
  }
}

static void ntpPollSyncNonBlocking() {
  if (!ntpConfigured) return;

  uint32_t nowMs = millis();
  if (nowMs - tNtpPoll < NTP_POLL_MS) return;
  tNtpPoll = nowMs;

  if (timeIsValid()) {
    if (!ntpSynced) {
      ntpSynced = true;
      ntpLastSyncEpoch = time(nullptr);
      if (cfg.verbose) {
        Serial.print("NTP synced @ "); Serial.println(fmtTime(ntpLastSyncEpoch));
      }
    }
  }
}

static bool dnsResolveWithFallback(const String& host, IPAddress& out) {
  if (WiFi.hostByName(host.c_str(), out)) return true;

  int dot = host.indexOf('.');
  if (dot > 0) {
    String shortHost = host.substring(0, dot);
    if (WiFi.hostByName(shortHost.c_str(), out)) return true;
  }
  return false;
}

bool resolveLedHostToIP(const String& hostIn, IPAddress& out) {
  String host = sanitizeHost(hostIn);

  if (WiFi.status() != WL_CONNECTED) return false;
  if (millis() < ledResolveNotBefore) return false;

  if (out.fromString(host)) return true;

  if (!host.endsWith(".local")) {
    for (int i = 0; i < 4; i++) {
      if (dnsResolveWithFallback(host, out)) return true;
      delay(150);
      yield();
    }
    return false;
  }

#if ENABLE_MDNS
  String base = host;
  base.replace(".local", "");
  for (int i = 0; i < 10; i++) {
    IPAddress ip = MDNS.queryHost(base);
    if ((uint32_t)ip != 0) { out = ip; return true; }
    delay(250);
    yield();
  }
#endif
  return false;
}

void resolveLedAtStartupBlocking() {
  if (WiFi.status() != WL_CONNECTED) return;

  const String host = sanitizeHost(cfg.led_host);
  IPAddress ip;

  Serial.print("Startup resolve LED host: "); Serial.println(host);

  const uint32_t start = millis();
  while ((millis() - start) < STARTUP_RESOLVE_MAX_MS) {
    if (millis() < ledResolveNotBefore) { delay(50); continue; }

#if ENABLE_MDNS
    mdnsEnsureRunning();
#endif

    if (resolveLedHostToIP(host, ip)) {
      led_cached_ip = ip;
      led_cached_ip_valid = true;
      ledResolveState = "ok";
      lastResolveMillis = millis();

      Serial.print("Startup resolve OK: "); Serial.print(host);
      Serial.print(" -> "); Serial.println(ip.toString());
      oledMessage("LED resolved", host, ip.toString());
      delay(250);
      return;
    }

    delay(STARTUP_RESOLVE_RETRY_MS);
    yield();
  }

  led_cached_ip_valid = false;
  led_cached_ip = IPAddress(0,0,0,0);
  ledResolveState = "fail";
  lastResolveMillis = millis();

  Serial.println("Startup resolve FAILED. Will retry at runtime.");
  oledMessage("LED resolve FAIL", host, "retrying...");
}

void resolveLedAtRuntimeNonBlocking() {
  if (led_cached_ip_valid) return;
  if (WiFi.status() != WL_CONNECTED) return;

  const uint32_t now = millis();
  if (now - tResolveRetry < RUNTIME_RESOLVE_RETRY_MS) return;
  tResolveRetry = now;

#if ENABLE_MDNS
  mdnsEnsureRunning();
#endif

  IPAddress ip;
  String host = sanitizeHost(cfg.led_host);
  if (resolveLedHostToIP(host, ip)) {
    led_cached_ip = ip;
    led_cached_ip_valid = true;
    ledResolveState = "ok";
    lastResolveMillis = millis();

    if (cfg.verbose) {
      Serial.print("Runtime resolve OK: "); Serial.print(host);
      Serial.print(" -> "); Serial.println(ip.toString());
    }
  } else {
    ledResolveState = "fail";
    lastResolveMillis = millis();
    if (cfg.verbose) {
      Serial.print("Runtime resolve FAILED for: "); Serial.println(host);
    }
  }
}

bool buildShellyGetUrl(char* url, size_t urlSize, const String& hostOrIp, int r, int g, int b) {
  int n = snprintf(
    url, urlSize,
    "http://%s/color/0?mode=color&turn=on&red=%d&green=%d&blue=%d&white=0&gain=100&effect=0&transition=500",
    hostOrIp.c_str(), r, g, b
  );
  return (n > 0 && (size_t)n < urlSize);
}

int shellySendRgb_GET(int r, int g, int b, const String& hostOrIp, uint32_t timeoutMs) {
  char url[320];
  if (!buildShellyGetUrl(url, sizeof(url), hostOrIp, r, g, b)) return -100;

  if (cfg.verbose) {
    Serial.print("HTTP GET -> ");
    Serial.println(url);
  }

  WiFiClient client;
  HTTPClient http;
  http.setTimeout(timeoutMs);
  http.begin(client, url);
  int httpCode = http.GET();

  lastShellyHttpCode = httpCode;
  lastShellyHostUsed = hostOrIp;
  lastShellyUrlSent  = String(url);
  lastShellyMillis   = millis();

  if (cfg.verbose) {
    Serial.print("HTTP code: ");
    Serial.println(httpCode);

    if (httpCode <= 0) {
      Serial.print("HTTP error: ");
      Serial.println(HTTPClient::errorToString(httpCode));
    } else {
      String payload = http.getString();
      if (payload.length() > 1200) payload = payload.substring(0, 1200) + "...";
      Serial.print("Shelly response body: ");
      Serial.println(payload);
    }
  }

  http.end();
  client.stop();
  return httpCode;
}

int shellySendRgb(int r, int g, int b, const String& hostOrIp, uint32_t timeoutMs) {
  return shellySendRgb_GET(r, g, b, hostOrIp, timeoutMs);
}

void co2ToRgb_AQI(float co2ppm) {
  float c = clampf(co2ppm, 400.0f, 2000.0f);

  const float c1 = 800.0f;
  const float c2 = 1200.0f;
  const float c3 = 2000.0f;

  const float green0[3]  = {  0, 255,   0};
  const float yellow1[3] = {255, 255,   0};
  const float orange2[3] = {255, 165,   0};
  const float red3[3]    = {255,   0,   0};

  float R, G, B;

  if (c <= c1) {
    float t = (c - 400.0f) / (c1 - 400.0f);
    R = lerpf(green0[0], yellow1[0], t);
    G = lerpf(green0[1], yellow1[1], t);
    B = lerpf(green0[2], yellow1[2], t);
  } else if (c <= c2) {
    float t = (c - c1) / (c2 - c1);
    R = lerpf(yellow1[0], orange2[0], t);
    G = lerpf(yellow1[1], orange2[1], t);
    B = lerpf(yellow1[2], orange2[2], t);
  } else {
    float t = (c - c2) / (c3 - c2);
    R = lerpf(orange2[0], red3[0], t);
    G = lerpf(orange2[1], red3[1], t);
    B = lerpf(orange2[2], red3[2], t);
  }

  const float gamma = 0.9f;
  auto gammaFix = [&](float v) {
    float x = clampf(v / 255.0f, 0.0f, 1.0f);
    return 255.0f * powf(x, gamma);
  };

  red   = gammaFix(R);
  green = gammaFix(G);
  blue  = gammaFix(B);
}

void co2ToRgb_LegacyWavelength(float co2ppm) {
  float wl = 380.0f + (co2ppm - 300.0f) * (780.0f - 380.0f) / (1000.0f - 300.0f);
  wl = clampf(wl, 380.0f, 780.0f);

  const float Gamma = 0.80f;
  const float IntensityMax = 255.0f;
  float factor;

  float r = 0, g = 0, b = 0;

  if (wl < 440) { r = -(wl - 440) / (440 - 380); g = 0; b = 1; }
  else if (wl < 490) { r = 0; g = (wl - 440) / (490 - 440); b = 1; }
  else if (wl < 510) { r = 0; g = 1; b = -(wl - 510) / (510 - 490); }
  else if (wl < 580) { r = (wl - 510) / (580 - 510); g = 1; b = 0; }
  else if (wl < 645) { r = 1; g = -(wl - 645) / (645 - 580); b = 0; }
  else { r = 1; g = 0; b = 0; }

  if (wl < 420) factor = 0.3f + 0.7f * (wl - 380) / (420 - 380);
  else if (wl < 701) factor = 1.0f;
  else factor = 0.3f + 0.7f * (780 - wl) / (780 - 700);

  red   = (r <= 0) ? 0 : (IntensityMax * powf(r * factor, Gamma));
  green = (g <= 0) ? 0 : (IntensityMax * powf(g * factor, Gamma));
  blue  = (b <= 0) ? 0 : (IntensityMax * powf(b * factor, Gamma));
}

void mapCO2ToRGB(float co2ppm) {
  if (cfg.color_mode == ColorMode::AQI) co2ToRgb_AQI(co2ppm);
  else co2ToRgb_LegacyWavelength(co2ppm);
}

float co2FilterAndHysteresis(float co2raw) {
  if (isnan(co2raw)) return NAN;

  if (isnan(co2_ema)) co2_ema = co2raw;
  else co2_ema = (CO2_EMA_ALPHA * co2raw) + ((1.0f - CO2_EMA_ALPHA) * co2_ema);

  bool adjusted = false;

  if (!cfg.co2_hyst_enabled) {
    co2_for_color = co2_ema;
    adjusted = false;
    lastCO2ema = co2_ema;
    lastCO2forColor = co2_for_color;
    lastCO2Adjusted = adjusted;
    return co2_for_color;
  }

  if (isnan(co2_for_color)) co2_for_color = co2_ema;

  if (fabsf(co2_ema - co2_for_color) >= CO2_HYST_PPM) {
    co2_for_color = co2_ema;
  } else {
    adjusted = true;
  }

  lastCO2ema = co2_ema;
  lastCO2forColor = co2_for_color;
  lastCO2Adjusted = adjusted;
  return co2_for_color;
}

// Robust SCD30 initialization that matches the proven minimal sketch.
bool initSCD30Robust(float altitude_m) {
  (void)altitude_m;  // applied later, after the sensor is already measuring

  scd30.begin(Wire, SCD30_I2C_ADDR_61);

  (void)scd30.stopPeriodicMeasurement();

  int16_t err = scd30.softReset();
  if (err != NO_ERROR) {
    logScd30Error("softReset", err);
    return false;
  }

  delay(2000);

  uint8_t fwMajor = 0, fwMinor = 0;
  err = scd30.readFirmwareVersion(fwMajor, fwMinor);
  if (err != NO_ERROR) {
    logScd30Error("readFirmwareVersion", err);
    return false;
  }

  Serial.print("SCD30 Found! FW=");
  Serial.print(fwMajor);
  Serial.print(".");
  Serial.println(fwMinor);

  err = scd30.startPeriodicMeasurement(0);
  if (err != NO_ERROR) {
    logScd30Error("startPeriodicMeasurement", err);
    return false;
  }

  scd30_config_applied = true;
  scd30_runtime_settings_done = false;
  scd30_outputs_ready = false;
  scd30_valid_sample_count = 0;
  co2_ema = NAN;
  co2_for_color = NAN;
  tScd30RuntimeCfg = 0;

  if (cfg.verbose) {
    uint16_t currentAsc = 0;
    err = scd30.getAutoCalibrationStatus(currentAsc);
    Serial.print("SCD30 ASC after start: ");
    if (err == NO_ERROR) Serial.println(currentAsc ? "ON" : "OFF");
    else { Serial.print("ERR "); Serial.println(err); }

    uint16_t currentAlt = 0;
    err = scd30.getAltitudeCompensation(currentAlt);
    Serial.print("SCD30 altitude after start: ");
    if (err == NO_ERROR) Serial.println(currentAlt);
    else { Serial.print("ERR "); Serial.println(err); }
  }

  return true;
}

// Apply non-critical SCD30 settings only after measurement is already running.
void maybeApplySCD30RuntimeSettingsNonBlocking() {
  if (!scd30_config_applied || scd30_runtime_settings_done) return;
  if (!isfinite(scd30_CO2_s) || scd30_CO2_s <= 0.0f) return;

  uint32_t now = millis();
  if (now - tScd30RuntimeCfg < SCD30_RUNTIME_CFG_RETRY_MS) return;
  tScd30RuntimeCfg = now;

  bool ok = true;
  int16_t err = 0;

  // Measurement interval default is already 2s from the datasheet.
  // Do not rewrite it at boot because the minimal proven startup path is more reliable.

  uint16_t targetAlt = (uint16_t)lroundf((float)cfg.altitude_m);
  uint16_t currentAlt = 0;
  err = scd30.getAltitudeCompensation(currentAlt);
  if (err != NO_ERROR) {
    ok = false;
    logScd30Error("getAltitudeCompensation", err);
  } else if (currentAlt != targetAlt) {
    err = scd30.setAltitudeCompensation(targetAlt);
    if (err != NO_ERROR) {
      ok = false;
      logScd30Error("setAltitudeCompensation", err);
    } else if (cfg.verbose) {
      Serial.print("SCD30 altitude applied at runtime: ");
      Serial.println(targetAlt);
    }
  }

  bool ascTarget = cfg.scd30_asc;
  if (cfg.scd30_frc_ppm >= 0) ascTarget = false;

  uint16_t currentAsc = 0;
  err = scd30.getAutoCalibrationStatus(currentAsc);
  if (err != NO_ERROR) {
    ok = false;
    logScd30Error("getAutoCalibrationStatus", err);
  } else if (((currentAsc != 0) ? true : false) != ascTarget) {
    err = scd30.activateAutoCalibration(ascTarget ? 1 : 0);
    if (err != NO_ERROR) {
      ok = false;
      logScd30Error("activateAutoCalibration", err);
    } else {
      cfg.scd30_asc = ascTarget;
      if (cfg.verbose) {
        Serial.print("SCD30 ASC applied at runtime: ");
        Serial.println(ascTarget ? "ON" : "OFF");
      }
    }
  }

  if (ok) {
    scd30_runtime_settings_done = true;
    if (cfg.verbose) Serial.println("SCD30 runtime settings complete.");
  }
}

static bool applyFrcNow400() {
  if (!isfinite(scd30_CO2_s) || scd30_CO2_s <= 0.0f) {
    if (cfg.verbose) Serial.println("FRC now refused: no valid CO2 reading yet.");
    return false;
  }

  const uint16_t target = 400;

  if (cfg.verbose) Serial.println("FRC now: disabling ASC before applying FRC");

  int16_t err = scd30.activateAutoCalibration(0);
  if (err != NO_ERROR) logScd30Error("activateAutoCalibration(0) for FRC", err);
  cfg.scd30_asc = false;

  err = scd30.forceRecalibration(target);
  if (err == NO_ERROR) {
    if (cfg.verbose) Serial.println("FRC now applied: 400 ppm");

    recordCalibrationApplied("FRC(now 400)");

    preferences.begin(PREF_NS, false);
    preferences.remove("scd30_frc");
    preferences.putBool("scd30_asc", false);
    preferences.end();

    cfg.scd30_frc_ppm = -1;
    key_scd30_frc = false;
    key_scd30_asc = true;
    return true;
  }

  logScd30Error("forceRecalibration(now)", err);
  return false;
}

void maybeApplyPendingFrcNonBlocking() {
  if (cfg.scd30_frc_ppm < 0) return;

  if (isnan(scd30_CO2_s) || scd30_CO2_s <= 0.0f) {
    if (cfg.verbose) Serial.println("Pending FRC: waiting for valid CO2 reading...");
    return;
  }

  const uint16_t target = (uint16_t)cfg.scd30_frc_ppm;

  if (cfg.verbose) {
    Serial.print("Applying pending FRC (Sensirion): ");
    Serial.print(target);
    Serial.println(" ppm");
  }

  int16_t err = scd30.activateAutoCalibration(0);
  if (err != NO_ERROR) logScd30Error("activateAutoCalibration(0) for pending FRC", err);
  cfg.scd30_asc = false;

  err = scd30.forceRecalibration(target);

  if (err == NO_ERROR) {
    recordCalibrationApplied(String("FRC(pending ") + target + " ppm)");

    preferences.begin(PREF_NS, false);
    preferences.remove("scd30_frc");
    preferences.putBool("scd30_asc", false);
    preferences.end();

    cfg.scd30_frc_ppm = -1;
    key_scd30_frc = false;
    key_scd30_asc = true;

    if (cfg.verbose) Serial.println("Pending FRC applied successfully.");
  } else {
    logScd30Error("forceRecalibration(pending)", err);
  }
}

void setLightsFromCO2(float co2ppmRaw) {
  const uint32_t now = millis();
  if (now - tShelly < SHELLY_PERIOD_MS) return;
  tShelly = now;

  if (WiFi.status() != WL_CONNECTED) {
    led_update_status = "x";
    return;
  }

  float co2ppm = co2FilterAndHysteresis(co2ppmRaw);
  if (isnan(co2ppm)) return;

  mapCO2ToRGB(co2ppm);

  const int r = (int)lroundf(red);
  const int g = (int)lroundf(green);
  const int b = (int)lroundf(blue);

  if (r == lastR && g == lastG && b == lastB) return;
  lastR = r; lastG = g; lastB = b;

  resolveLedAtRuntimeNonBlocking();

  const String host = sanitizeHost(cfg.led_host);

  int code = -1;

  if (led_cached_ip_valid) {
    code = shellySendRgb(r, g, b, led_cached_ip.toString(), 2000);
  }

  if (code != 200) {
    IPAddress resolved;
    if (resolveLedHostToIP(host, resolved)) {
      led_cached_ip = resolved;
      led_cached_ip_valid = true;
      ledResolveState = "ok";
      lastResolveMillis = millis();
      code = shellySendRgb(r, g, b, resolved.toString(), 2000);
    }
  }

  if (code != 200) {
    code = shellySendRgb(r, g, b, host, 2000);
  }

  led_update_status = (code == 200) ? ">" : "x";
}

static inline bool bmeHumidityLooksValid() {
  if (!bme_has_humidity) return false;
  return isfinite(bme_H_s) && (bme_H_s > 0.1f) && (bme_H_s <= 100.0f);
}

void displayOLED(float C, float H, float CO2) {
  oledTurnOnIfNeeded(false);
  if (!display.width()) return;

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);

  display.setCursor(0, 0);
  if (!isnan(C)) { display.print(C, 1); display.print("c"); }
  else display.print("--.- C");

  display.setCursor(44, 0);
  if (!isnan(H)) { display.print(H, 1); display.print("%"); }
  else display.print("--.-%");

  display.setCursor(90, 0);
  if (bme_sensor_found && isfinite(bme_P_s)) {
    display.print((int)lroundf(bme_P_s));
    display.print("hPa");
  } else {
    display.print("---");
  }

  display.setCursor(0, 10); display.setTextSize(1);  display.print("CO2: ");
  display.setCursor(110, 10); display.setTextSize(1);  display.print("ppm");
  display.setCursor(104, 20); display.setTextSize(1);  display.print(getSdLogIndicator());

  display.setCursor(24, 10); display.setTextSize(3);
  if (!isnan(CO2)) display.print((int)lroundf(CO2));
  else display.print("---");

  display.setTextSize(1);
  display.setCursor(0, 33);
  display.print("Mode: ");
  display.print(colorModeToStr(cfg.color_mode));

  display.setCursor(64, 33);
  display.print("Hyst: ");
  display.print(cfg.co2_hyst_enabled ? "ON" : "OFF");

  display.setCursor(0, 48);
  display.print("IP:");
  display.println(WiFi.localIP());

  display.setCursor(0, 56);
  display.print(led_update_status);
  display.print("LED:");
  if (led_cached_ip_valid) display.print(led_cached_ip);
  else display.print(ledResolveState);

  display.display();
}

void printSensorData() {
  const bool hdr = strYes(cfg.report_txt);

  if (cfg.report == "A" || cfg.report == "T") {
    if (hdr) Serial.print("Temperature: ");
    Serial.print(scd30_T_s, 1);
    if (hdr) Serial.print(" C     ");
  }

  if (cfg.report == "A" || cfg.report == "H") {
    if (hdr) Serial.print("Relative Humidity: ");
    Serial.print(scd30_H_s, 1);
    if (hdr) Serial.print(" %     ");
  }

  if (cfg.report == "A" || cfg.report == "C") {
    if (hdr) Serial.print("CO2: ");
    Serial.print(scd30_CO2_s, 0);
    if (hdr) Serial.print(" ppm");
  }

  Serial.println();
}

bool readBMEandUpdateAltitude() {
  if (!bme_sensor_found) return false;

  float temp(NAN), hum(NAN), pres(NAN);
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_hPa);

  bme.read(pres, temp, hum, tempUnit, presUnit);
  if (isnan(pres) || isnan(temp)) return false;

  bme_T_s = temp;
  bme_H_s = hum;
  bme_P_s = pres;

  EnvironmentCalculations::AltitudeUnit envAltUnit  = EnvironmentCalculations::AltitudeUnit_Meters;
  EnvironmentCalculations::TempUnit     envTempUnit = EnvironmentCalculations::TempUnit_Celsius;

  alt_s = EnvironmentCalculations::Altitude(pres, envAltUnit, referencePressure, outdoorTemp, envTempUnit);

  if (cfg.verbose && !bme_humidity_warning_printed) {
    if (!bme_has_humidity) {
      Serial.println("BMP280 detected -> no humidity channel, OLED humidity will fall back to SCD30.");
      bme_humidity_warning_printed = true;
    } else if (isfinite(bme_H_s) && fabsf(bme_H_s) < 0.05f) {
      Serial.println("BME280 humidity is 0.0% -> keeping BME temperature/pressure, but OLED humidity will fall back to SCD30.");
      bme_humidity_warning_printed = true;
    }
  }

  return true;
}

void printBmeSensorDataVerbose() {
  if (!cfg.verbose || !bme_sensor_found) return;
  if (!isfinite(bme_T_s) || !isfinite(bme_P_s)) return;

  Serial.print(bmeSensorName);
  Serial.print(": T=");
  Serial.print(bme_T_s, 1);
  Serial.print(" C  RH=");
  if (bme_has_humidity && isfinite(bme_H_s)) Serial.print(bme_H_s, 1);
  else Serial.print("n/a");
  Serial.print(" %  P=");
  Serial.print(bme_P_s, 1);
  Serial.print(" hPa  Alt=");
  if (isfinite(alt_s)) Serial.print(alt_s, 1);
  else Serial.print("n/a");
  Serial.print(" m");
  if (!bmeHumidityLooksValid()) Serial.print("  [OLED RH->SCD30]");
  Serial.println();
}

void printScd30SensorDataVerbose(float co2, float temp, float rh) {
  if (!cfg.verbose) return;
  if (!isfinite(co2) || !isfinite(temp) || !isfinite(rh)) return;

  Serial.print("SCD30 : T=");
  Serial.print(temp, 1);
  Serial.print(" C  RH=");
  Serial.print(rh, 1);
  Serial.print(" %  CO2=");
  Serial.print(co2, 1);
  Serial.println(" ppm");
}

static bool requireAdmin(AsyncWebServerRequest* request) {
  if (cfg.lan_only) {
    IPAddress client = request->client()->remoteIP();
    IPAddress local  = WiFi.localIP();
    bool same24 = (client[0]==local[0] && client[1]==local[1] && client[2]==local[2]);
    if (!same24) {
      request->send(403, "text/plain; charset=utf-8", "Forbidden (LAN-only)\n");
      return false;
    }
  }

  const char* user = "admin";
  const char* pass = cfg.auth_pass.c_str();
  if (!request->authenticate(user, pass)) {
    request->requestAuthentication();
    return false;
  }
  return true;
}

void showMenu() {
  Serial.println();
  Serial.println("|***************************************|");
  Serial.println("|**|           CO2 Sensor            |**|");
  Serial.println("|**|             Setup               |**|");
  Serial.println("|***************************************|");
  Serial.println(MY_VERSION);
  Serial.println();
  Serial.print("0 - Set WiFi SSID ["); Serial.print(cfg.ssid); Serial.println("]");
  Serial.print("1 - Set WiFi Password ["); Serial.print("(hidden)"); Serial.println("]");
  Serial.print("2 - Set LED host/IP ["); Serial.print(cfg.led_host); Serial.println("]");
  Serial.print("3 - Set Hostname ["); Serial.print(cfg.hostname); Serial.println("]");
  Serial.print("4 - Set Altitude (m) ["); Serial.print(cfg.altitude_m); Serial.println("]");
  Serial.print("5 - Report (A/T/H/C) ["); Serial.print(cfg.report); Serial.println("]");
  Serial.print("6 - Report headers+units (Y/N) ["); Serial.print(cfg.report_txt); Serial.println("]");
  Serial.print("7 - Color mode (toggle) ["); Serial.print(colorModeToStr(cfg.color_mode)); Serial.println("]");
  Serial.print("8 - Toggle verbose ["); Serial.print(cfg.verbose ? "true" : "false"); Serial.println("]");
  Serial.print("H - Toggle CO2 hysteresis ["); Serial.print(cfg.co2_hyst_enabled ? "ON" : "OFF"); Serial.println("]");
  Serial.print("L - Toggle LAN-only guard ["); Serial.print(cfg.lan_only ? "ON" : "OFF"); Serial.println("]");
  Serial.println("9 - Exit menu");
  Serial.println();
  Serial.print("Select (no Enter needed): ");
}

void runMenuWindow(uint32_t windowMs) {
  Serial.println();
  Serial.println("Serial Setup Menu available for 10 seconds.");
  Serial.println("Press any key to enter...");

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.setTextSize(3);
  display.print("CO2");
  display.setTextSize(2);
  display.print("Sensor");
  display.setCursor(0, 25);
  display.setTextSize(1);
  display.print(String("Version: ") + MY_VERSION);
  display.setCursor(0, 40);
  display.print("Press key to config..");
  display.display();

  if (!enterMenuRequested) {
    char c = readCharBlocking(windowMs);
    if (c == 0) {
      Serial.println("Menu timeout; continuing boot.");
      return;
    }
  }

  flushSerialInput();
  Config oldCfg = cfg;

  for (;;) {
    showMenu();
    char choice = readCharBlocking(30000);
    if (choice == 0) return;
    Serial.println(choice);

    if (choice == '9') break;

    if (choice == '0') {
      flushSerialInput();
      Serial.print("Enter SSID (then Enter): ");
      String v = readLineBlocking(60000);
      v.trim();
      if (v.length()) cfg.ssid = v;

    } else if (choice == '1') {
      flushSerialInput();
      Serial.print("Enter WiFi Password (then Enter): ");
      String v = readLineBlocking(60000);
      v.trim();
      if (v.length()) { cfg.pass = v; if (!key_auth_pass) cfg.auth_pass = cfg.pass; }

    } else if (choice == '2') {
      flushSerialInput();
      Serial.print("Enter LED host/IP (then Enter): ");
      String v = readLineBlocking(60000);
      v.trim();
      if (v.length()) {
        cfg.led_host = sanitizeHost(v);
        led_cached_ip_valid = false;
        led_cached_ip = IPAddress(0,0,0,0);
        ledResolveState = "unknown";
      }

    } else if (choice == '3') {
      flushSerialInput();
      Serial.print("Enter Hostname (then Enter): ");
      String v = readLineBlocking(60000);
      v.trim();
      if (v.length()) cfg.hostname = v;

    } else if (choice == '4') {
      flushSerialInput();
      Serial.print("Enter Altitude (m) (then Enter): ");
      String v = readLineBlocking(60000);
      v.trim();
      if (v.length()) cfg.altitude_m = v.toInt();

    } else if (choice == '5') {
      flushSerialInput();
      Serial.print("Enter Report (A/T/H/C) (then Enter): ");
      String v = readLineBlocking(60000);
      v.trim();
      if (v.length()) cfg.report = v;

    } else if (choice == '6') {
      flushSerialInput();
      Serial.print("Headers+Units (Y/N) (then Enter): ");
      String v = readLineBlocking(60000);
      v.trim();
      if (v.length()) cfg.report_txt = v;

    } else if (choice == '7') {
      cfg.color_mode = (cfg.color_mode == ColorMode::AQI) ? ColorMode::LEGACY_WAVELENGTH : ColorMode::AQI;
      Serial.print("Color mode now: ");
      Serial.println(colorModeToStr(cfg.color_mode));

    } else if (choice == '8') {
      cfg.verbose = !cfg.verbose;
      Serial.print("Verbose now: ");
      Serial.println(cfg.verbose ? "true" : "false");

    } else if (choice == 'H' || choice == 'h') {
      cfg.co2_hyst_enabled = !cfg.co2_hyst_enabled;
      Serial.print("CO2 hysteresis now: ");
      Serial.println(cfg.co2_hyst_enabled ? "ON" : "OFF");

    } else if (choice == 'L' || choice == 'l') {
      cfg.lan_only = !cfg.lan_only;
      Serial.print("LAN-only guard now: ");
      Serial.println(cfg.lan_only ? "ON" : "OFF");

    } else {
      Serial.println("Unknown option.");
    }

    saveConfigIfChanged(oldCfg);
    oldCfg = cfg;
  }

  saveConfigIfChanged(oldCfg);
  Serial.println("Exiting menu.");
}

const char* src(bool keyPresent) { return keyPresent ? "NVS" : "Secrets"; }

String htmlEscape(const String& s) {
  String o; o.reserve(s.length() + 16);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '&') o += F("&amp;");
    else if (c == '<') o += F("&lt;");
    else if (c == '>') o += F("&gt;");
    else if (c == '"') o += F("&quot;");
    else o += c;
  }
  return o;
}

static String cssRgbTriplet(int r, int g, int b) {
  if (r < 0) r = 0;
  if (r > 255) r = 255;
  if (g < 0) g = 0;
  if (g > 255) g = 255;
  if (b < 0) b = 0;
  if (b > 255) b = 255;
  return "rgb(" + String(r) + "," + String(g) + "," + String(b) + ")";
}

static String htmlInputRow(const String& label, const String& name, const String& value,
                           const String& srcTxt, bool isPassword=false, const String& placeholder="") {
  String r;
  r += "<div class='row'><label>";
  r += htmlEscape(label);
  r += "</label><div class='cell'>";
  r += "<input ";
  r += "name='" + htmlEscape(name) + "' ";
  r += "value='" + (isPassword ? "" : htmlEscape(value)) + "' ";
  r += "placeholder='" + htmlEscape(placeholder) + "' ";
  r += "type='";
  r += (isPassword ? "password" : "text");
  r += "'>";
  r += "<div class='src'>src: ";
  r += htmlEscape(srcTxt);
  r += "</div></div></div>";
  return r;
}

static String htmlSelectRow(const String& label, const String& name,
                            const String& optA, const String& optB,
                            bool isA, const String& srcTxt) {
  String r;
  r += "<div class='row'><label>";
  r += htmlEscape(label);
  r += "</label><div class='cell'>";
  r += "<select name='" + htmlEscape(name) + "'>";
  r += "<option value='" + htmlEscape(optA) + "' " + (isA ? "selected" : "") + ">" + htmlEscape(optA) + "</option>";
  r += "<option value='" + htmlEscape(optB) + "' " + (!isA ? "selected" : "") + ">" + htmlEscape(optB) + "</option>";
  r += "</select>";
  r += "<div class='src'>src: ";
  r += htmlEscape(srcTxt);
  r += "</div></div></div>";
  return r;
}


static String htmlSelectRow3(const String& label, const String& name,
                             const String& optA, const String& optB, const String& optC,
                             const String& current, const String& srcTxt) {
  String r;
  r += "<div class='row'><label>";
  r += htmlEscape(label);
  r += "</label><div class='cell'>";
  r += "<select name='" + htmlEscape(name) + "'>";
  r += "<option value='" + htmlEscape(optA) + "' " + ((current == optA) ? "selected" : "") + ">" + htmlEscape(optA) + "</option>";
  r += "<option value='" + htmlEscape(optB) + "' " + ((current == optB) ? "selected" : "") + ">" + htmlEscape(optB) + "</option>";
  r += "<option value='" + htmlEscape(optC) + "' " + ((current == optC) ? "selected" : "") + ">" + htmlEscape(optC) + "</option>";
  r += "</select>";
  r += "<div class='src'>src: ";
  r += htmlEscape(srcTxt);
  r += "</div></div></div>";
  return r;
}


static bool ensureSdCardMountedForConfigExport() {
  if (sd_card_ready) return true;

  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  delay(20);
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

  SdSpiConfig cfgSd(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(SD_SPI_MHZ_));
  if (!sd.begin(cfgSd)) {
    Serial.println("SD init FAILED for config export.");
    return false;
  }

  sd_card_ready = true;
  return true;
}

static inline void cfgWriteLine(FsFile& file, const char* key, const String& value) {
  file.print(key);
  file.print('=');
  file.println(value);
}

static inline String boolToOnOff(bool v) {
  return v ? String("ON") : String("OFF");
}

static inline bool cfgParseBool(const String& s, bool defaultValue=false) {
  String v = s;
  v.trim();
  v.toUpperCase();
  if (v == "ON" || v == "TRUE" || v == "1" || v == "YES" || v == "Y") return true;
  if (v == "OFF" || v == "FALSE" || v == "0" || v == "NO" || v == "N") return false;
  return defaultValue;
}

static bool importConfigSnapshotFromSd(String& msg) {
  if (!ensureSdCardMountedForConfigExport()) {
    msg = "SD init failed";
    return false;
  }

  if (!sd.exists("/config/nvm_config.cfg")) {
    msg = "File not found: /config/nvm_config.cfg";
    return false;
  }

  FsFile file;
  if (!file.open("/config/nvm_config.cfg", O_RDONLY)) {
    msg = "Failed to open /config/nvm_config.cfg";
    return false;
  }

  Config oldCfg = cfg;
  Config newCfg = cfg;

  String line;
  while (file.available()) {
    int ch = file.read();
    if (ch < 0) break;
    if (ch == '\r') continue;
    if (ch != '\n') {
      line += (char)ch;
      continue;
    }

    String work = line;
    line = "";
    work.trim();
    if (work.length() == 0 || work[0] == '#') continue;

    int eq = work.indexOf('=');
    if (eq <= 0) continue;

    String key = work.substring(0, eq);
    String val = work.substring(eq + 1);
    key.trim();
    val.trim();

    if (key == "ssid") newCfg.ssid = val;
    else if (key == "ssid_pwd") newCfg.pass = val;
    else if (key == "led") newCfg.led_host = sanitizeHost(val);
    else if (key == "hostname") newCfg.hostname = val;
    else if (key == "altitude") newCfg.altitude_m = val.toInt();
    else if (key == "report") newCfg.report = val;
    else if (key == "report_txt") newCfg.report_txt = val;
    else if (key == "verbose") newCfg.verbose = cfgParseBool(val, newCfg.verbose);
    else if (key == "color_mode") newCfg.color_mode = parseColorMode(val);
    else if (key == "hyst") newCfg.co2_hyst_enabled = cfgParseBool(val, newCfg.co2_hyst_enabled);
    else if (key == "lan_only") newCfg.lan_only = cfgParseBool(val, newCfg.lan_only);
    else if (key == "auth_pass") newCfg.auth_pass = val;
    else if (key == "scd30_asc") newCfg.scd30_asc = cfgParseBool(val, newCfg.scd30_asc);
    else if (key == "scd30_frc") {
      if (val.length() == 0) newCfg.scd30_frc_ppm = -1;
      else {
        int v = val.toInt();
        if (v < 350) v = 350;
        if (v > 2000) v = 2000;
        newCfg.scd30_frc_ppm = (int16_t)v;
      }
    }
    else if (key == "sd_log_enable") newCfg.sd_log_enable = cfgParseBool(val, newCfg.sd_log_enable);
    else if (key == "sd_log_period_sec") {
      uint32_t v = (uint32_t)val.toInt();
      if (v == 0) v = 60;
      newCfg.sd_log_period_sec = v;
    }
    else if (key == "sd_rotation_cfg") {
      val.toUpperCase();
      if (val != "DAY" && val != "MONTH" && val != "OFF") val = "OFF";
      newCfg.sd_rotation_cfg = val;
    }
    else if (key == "sd_keep_files") {
      uint32_t v = (uint32_t)val.toInt();
      if (v > 3650UL) v = 3650UL;
      newCfg.sd_keep_files = (uint16_t)v;
    }
    else if (key == "screensave_time_minutes") {
      uint32_t v = (uint32_t)val.toInt();
      if (v > 1440UL) v = 1440UL;
      newCfg.screensave_time_minutes = (uint16_t)v;
    }
  }

  if (line.length()) {
    String work = line;
    work.trim();
    if (work.length() > 0 && work[0] != '#') {
      int eq = work.indexOf('=');
      if (eq > 0) {
        String key = work.substring(0, eq);
        String val = work.substring(eq + 1);
        key.trim();
        val.trim();
        if (key == "ssid") newCfg.ssid = val;
        else if (key == "ssid_pwd") newCfg.pass = val;
        else if (key == "led") newCfg.led_host = sanitizeHost(val);
        else if (key == "hostname") newCfg.hostname = val;
        else if (key == "altitude") newCfg.altitude_m = val.toInt();
        else if (key == "report") newCfg.report = val;
        else if (key == "report_txt") newCfg.report_txt = val;
        else if (key == "verbose") newCfg.verbose = cfgParseBool(val, newCfg.verbose);
        else if (key == "color_mode") newCfg.color_mode = parseColorMode(val);
        else if (key == "hyst") newCfg.co2_hyst_enabled = cfgParseBool(val, newCfg.co2_hyst_enabled);
        else if (key == "lan_only") newCfg.lan_only = cfgParseBool(val, newCfg.lan_only);
        else if (key == "auth_pass") newCfg.auth_pass = val;
        else if (key == "scd30_asc") newCfg.scd30_asc = cfgParseBool(val, newCfg.scd30_asc);
        else if (key == "scd30_frc") {
          if (val.length() == 0) newCfg.scd30_frc_ppm = -1;
          else {
            int v = val.toInt();
            if (v < 350) v = 350;
            if (v > 2000) v = 2000;
            newCfg.scd30_frc_ppm = (int16_t)v;
          }
        }
        else if (key == "sd_log_enable") newCfg.sd_log_enable = cfgParseBool(val, newCfg.sd_log_enable);
        else if (key == "sd_log_period_sec") {
          uint32_t v = (uint32_t)val.toInt();
          if (v == 0) v = 60;
          newCfg.sd_log_period_sec = v;
        }
        else if (key == "sd_rotation_cfg") {
          val.toUpperCase();
          if (val != "DAY" && val != "MONTH" && val != "OFF") val = "OFF";
          newCfg.sd_rotation_cfg = val;
        }
        else if (key == "sd_keep_files") {
          uint32_t v = (uint32_t)val.toInt();
          if (v > 3650UL) v = 3650UL;
          newCfg.sd_keep_files = (uint16_t)v;
        }
        else if (key == "screensave_time_minutes") {
          uint32_t v = (uint32_t)val.toInt();
          if (v > 1440UL) v = 1440UL;
          newCfg.screensave_time_minutes = (uint16_t)v;
        }
      }
    }
  }

  file.close();

  if (newCfg.auth_pass.length() == 0) newCfg.auth_pass = newCfg.pass;
  if (newCfg.scd30_frc_ppm >= 0 && newCfg.scd30_asc) newCfg.scd30_asc = false;

  cfg = newCfg;
  saveConfigIfChanged(oldCfg);
  loadConfig();

  // Apply key runtime derivatives without requiring reboot yet.
  sd_log_enabled = cfg.sd_log_enable;
  sd_log_period_ms = cfg.sd_log_period_sec * 1000UL;
  if (sd_log_period_ms == 0) sd_log_period_ms = 60000UL;
  sd_rotation_mode = cfg.sd_rotation_cfg;
  sd_keep_files = cfg.sd_keep_files;

  msg = "Loaded /config/nvm_config.cfg into NVS";
  if (cfg.verbose) {
    Serial.println(msg);
  }
  return true;
}

static bool exportConfigSnapshotToSd(String& msg) {
  if (!ensureSdCardMountedForConfigExport()) {
    msg = "SD init failed";
    return false;
  }

  if (!sd.exists("/config")) {
    if (!sd.mkdir("/config")) {
      msg = "Failed to create /config";
      return false;
    }
  }

  FsFile runtimeFile;
  if (!runtimeFile.open("/config/nvm_config.cfg", O_WRONLY | O_CREAT | O_TRUNC)) {
    msg = "Failed to open /config/nvm_config.cfg";
    return false;
  }

  runtimeFile.println("# Current effective runtime config");
  runtimeFile.print("# firmware="); runtimeFile.println(MY_VERSION);
  runtimeFile.print("# exported_at="); runtimeFile.println(timeIsValid() ? fmtTime(time(nullptr)) : String("unsynced"));
  cfgWriteLine(runtimeFile, "ssid", cfg.ssid);
  cfgWriteLine(runtimeFile, "ssid_pwd", cfg.pass);
  cfgWriteLine(runtimeFile, "led", cfg.led_host);
  cfgWriteLine(runtimeFile, "hostname", cfg.hostname);
  cfgWriteLine(runtimeFile, "altitude", String(cfg.altitude_m));
  cfgWriteLine(runtimeFile, "report", cfg.report);
  cfgWriteLine(runtimeFile, "report_txt", cfg.report_txt);
  cfgWriteLine(runtimeFile, "verbose", boolToOnOff(cfg.verbose));
  cfgWriteLine(runtimeFile, "color_mode", String(colorModeToStr(cfg.color_mode)));
  cfgWriteLine(runtimeFile, "hyst", boolToOnOff(cfg.co2_hyst_enabled));
  cfgWriteLine(runtimeFile, "lan_only", boolToOnOff(cfg.lan_only));
  cfgWriteLine(runtimeFile, "auth_pass", cfg.auth_pass);
  cfgWriteLine(runtimeFile, "scd30_asc", boolToOnOff(cfg.scd30_asc));
  cfgWriteLine(runtimeFile, "scd30_frc", (cfg.scd30_frc_ppm >= 0) ? String(cfg.scd30_frc_ppm) : String(""));
  cfgWriteLine(runtimeFile, "sd_log_enable", boolToOnOff(cfg.sd_log_enable));
  cfgWriteLine(runtimeFile, "sd_log_period_sec", String((unsigned long)cfg.sd_log_period_sec));
  cfgWriteLine(runtimeFile, "sd_rotation_cfg", cfg.sd_rotation_cfg);
  cfgWriteLine(runtimeFile, "sd_keep_files", String((unsigned long)cfg.sd_keep_files));
  cfgWriteLine(runtimeFile, "screensave_time_minutes", String((unsigned long)cfg.screensave_time_minutes));
  bool runtimeOk = (runtimeFile.getWriteError() == 0);
  runtimeFile.close();
  if (!runtimeOk) {
    msg = "Failed writing /config/nvm_config.cfg";
    return false;
  }

  msg = "Saved /config/nvm_config.cfg";
  if (cfg.verbose) {
    Serial.println(msg);
  }
  return true;
}

static String buildConfigEditorHtml() {
  String h; h.reserve(11000);

  bool frcPending = (cfg.scd30_frc_ppm >= 0);

  h += F("<!DOCTYPE html><html><head><meta charset='utf-8'>");
  h += F("<meta name='viewport' content='width=device-width, initial-scale=1'>");
  h += F("<title>CO2 Sensor Config</title>");
  h += F("<style>");
  h += F("body{font-family:Arial;margin:14px;} .box{padding:12px;border:1px solid #ccc;border-radius:10px;margin:10px 0;}");
  h += F(".row{display:grid;grid-template-columns:200px 1fr;gap:10px;align-items:center;margin:10px 0;}");
  h += F("label{color:#333;font-weight:bold;} input,select{width:100%;padding:8px;border:1px solid #bbb;border-radius:8px;}");
  h += F(".src{font-size:12px;color:#666;margin-top:4px;}");
  h += F(".btn{padding:10px 14px;border:0;border-radius:10px;cursor:pointer;}");
  h += F(".btn-save{background:#2d7;} .btn-warn{background:#f96;} .btn-danger{background:#f55;color:#fff;} .btn-cal{background:#59f;color:#fff;}");
  h += F(".banner{padding:10px;border-radius:10px;margin:10px 0;}");
  h += F(".warn{background:#fff2cc;border:1px solid #e6c200;}");
  h += F("code{word-break:break-all;}");
  h += F("</style></head><body>");

  h += F("<h2>CO2 Sensor Config</h2>");

  if (frcPending) {
    h += F("<div class='banner warn'><b>Warning:</b> FRC is pending (");
    h += String(cfg.scd30_frc_ppm);
    h += F(" ppm). ASC is forced OFF until calibration is applied.</div>");
  }

  h += F("<div class='box'>");
  h += F("<div><b>Firmware:</b> v"); h += MY_VERSION; h += F("</div>");
  h += F("<div><b>Device:</b> "); h += htmlEscape(cfg.hostname); h += F("</div>");
  h += F("<div><b>IP:</b> "); h += htmlEscape(WiFi.localIP().toString()); h += F("</div>");
#if ENABLE_MDNS
  h += F("<div><b>mDNS:</b> <code>http://"); h += htmlEscape(cfg.hostname); h += F(".local/</code></div>");
#endif
  h += F("</div>");

  h += F("<div class='box'><h3>Calibration Instructions</h3><ol>");
  h += F("<li>For <b>FRC</b>, place sensor in stable fresh outdoor air (~400 ppm).</li>");
  h += F("<li>Keep it there for at least <b>2–5 minutes</b> before pressing Calibrate.</li>");
  h += F("<li>Disable fans/vents blowing directly onto the sensor during calibration.</li>");
  h += F("<li>After calibration, wait <b>1–2 minutes</b> for readings to stabilize.</li>");
  h += F("<li>Use <b>ASC ON</b> for long-running devices in spaces that periodically reach ~400 ppm.</li>");
  h += F("<li>If the space never reaches fresh air levels, keep <b>ASC OFF</b>.</li>");
  h += F("<li>When an FRC value is pending, the firmware forces <b>ASC OFF</b> for safety.</li>");
  h += F("<li>FRC pending will be applied automatically once a valid CO2 reading is available.</li>");
  h += F("<li>You can also press <b>Calibrate now (400 ppm)</b> to apply immediately (requires valid reading).</li>");
  h += F("</ol></div>");

  h += F("<form class='box' method='POST' action='/config'>");
  h += F("<h3>Network</h3>");
  h += htmlInputRow("WiFi SSID", "ssid", cfg.ssid, src(key_ssid));
  h += htmlInputRow("WiFi Password", "pass", "", src(key_pass), true, "leave blank to keep current");
  h += htmlInputRow("ESP Hostname", "hostname", cfg.hostname, src(key_hostname));
  h += htmlInputRow("LED host/IP", "led_host", cfg.led_host, src(key_led), false, "IP, FQDN, or *.local");

  h += F("<h3>Sensor / Reporting</h3>");
  h += htmlInputRow("Altitude (m)", "altitude", String(cfg.altitude_m), src(key_altitude));
  h += htmlInputRow("Report (A/T/H/C)", "report", cfg.report, src(key_report));
  h += htmlInputRow("Report headers (Y/N)", "report_txt", cfg.report_txt, src(key_report_txt));

  h += F("<h3>Behavior</h3>");
  h += htmlSelectRow("Color mode", "color_mode", "AQI", "LEGACY", (cfg.color_mode == ColorMode::AQI), src(key_color_mode));
  h += htmlSelectRow("CO2 hysteresis", "hyst", "ON", "OFF", cfg.co2_hyst_enabled, src(key_hyst));
  h += htmlSelectRow("Verbose debug", "verbose", "ON", "OFF", cfg.verbose, src(key_verbose));
  h += htmlSelectRow("LAN-only guard", "lan_only", "ON", "OFF", cfg.lan_only, src(key_lan_only));

  h += F("<h3>Calibration</h3>");
  h += htmlSelectRow("SCD30 ASC", "scd30_asc", "ON", "OFF", cfg.scd30_asc, key_scd30_asc ? "NVS" : "Default");
  h += htmlInputRow("SCD30 FRC pending (ppm)", "scd30_frc", (cfg.scd30_frc_ppm >= 0 ? String(cfg.scd30_frc_ppm) : ""), key_scd30_frc ? "NVS" : "None", false, "blank clears; range 350..2000");

  h += F("<h3>SD Logging</h3>");
  h += htmlSelectRow("SD logging", "sd_log_enable", "ON", "OFF", cfg.sd_log_enable, key_sd_log_enable ? "NVS" : "Secrets");
  h += htmlInputRow("SD log period (sec)", "sd_log_period_sec", String((unsigned long)cfg.sd_log_period_sec), key_sd_log_period ? "NVS" : "Secrets", false, "e.g. 60");
  h += htmlSelectRow3("SD rotation", "sd_rotation_cfg", "DAY", "MONTH", "OFF", cfg.sd_rotation_cfg, key_sd_rotation ? "NVS" : "Secrets");
  h += htmlInputRow("SD Keep last x files", "sd_keep_files", String((unsigned long)cfg.sd_keep_files), key_sd_keep_files ? "NVS" : "Secrets", false, "0 = keep all; e.g. 31");

  h += F("<h3>Display</h3>");
  h += htmlInputRow("Screensave time (min)", "screensave_time_minutes", String((unsigned long)cfg.screensave_time_minutes), key_screensave_minutes ? "NVS" : "Secrets", false, "0 disables screensaver");
  h += F("<div class='src'>ESP32 touch-capable GPIOs: 0, 2, 4, 12, 13, 14, 15, 27, 32, 33</div>");

  h += F("<div style='margin-top:12px;'>");
  h += F("<button class='btn btn-save' type='submit'>Save to NVS</button>");
  h += F("</div>");
  h += F("</form>");

  h += F("<div class='box'>");
  h += F("<h3>Calibrate Now</h3>");
  h += F("<form method='POST' action='/calibrate400'>");
  h += F("<button class='btn btn-cal' type='submit'>Calibrate now (400 ppm)</button>");
  h += F("</form>");
  h += F("</div>");

  h += F("<div class='box'>");
  h += F("<h3>SD Config</h3>");
  h += F("<div><b>SD card present:</b> ");
  h += (sd_card_ready ? "YES" : "NO");
  h += F("</div>");
  h += F("<div style='height:10px;'></div>");
  h += F("<form method='POST' action='/exportconfigsd'>");
  h += F("<button class='btn btn-save' type='submit'");
  if (!sd_card_ready) h += F(" disabled");
  h += F(">Save NVS to SD</button>");
  h += F("</form>");
  h += F("<div style='height:10px;'></div>");
  h += F("<form method='POST' action='/importconfigsd'>");
  h += F("<button class='btn btn-save' type='submit'");
  if (!sd_card_ready) h += F(" disabled");
  h += F(">Load SD config into NVM</button>");
  h += F("</form>");
  h += F("</div>");

  h += F("<div class='box'>");
  h += F("<h3>Maintenance</h3>");
  h += F("<form method='POST' action='/reboot'><button class='btn btn-warn' type='submit'>Reboot</button></form>");
  h += F("<div style='height:10px;'></div>");
  h += F("<form method='POST' action='/factoryreset' onsubmit=\"return confirm('Wipe NVS and reboot?');\">");
  h += F("<button class='btn btn-danger' type='submit'>Factory Reset (wipe NVS)</button></form>");
  h += F("</div>");

  h += F("<div class='box'><a href='/'>Back to Status</a></div>");
  h += F("</body></html>");
  return h;
}

static void htmlPageBegin(String &h, const char *title) {
  h += F("<!DOCTYPE html><html><head><meta charset='utf-8'>");
  h += F("<meta name='viewport' content='width=device-width, initial-scale=1'>");
  h += F("<title>");
  h += title;
  h += F("</title>");

  h += F("<style>");
  h += F("body{font-family:Arial;margin:14px;} ");
  h += F(".box{padding:10px;border:1px solid #ccc;border-radius:8px;margin:10px 0;} ");
  h += F(".k{color:#555;} code{word-break:break-all;} ");
  h += F(".banner{padding:10px;border-radius:10px;margin:10px 0;} .warn{background:#fff2cc;border:1px solid #e6c200;} ");
  h += F(".topnav{display:flex;gap:10px;flex-wrap:wrap;margin:12px 0;} ");
  h += F(".btn{display:inline-block;padding:10px 14px;border:1px solid #bbb;border-radius:10px;text-decoration:none;color:#111;background:#f6f6f6;} ");
  h += F(".btn:hover{background:#eee;} ");
  h += F(".btn-primary{border-color:#6aa0ff;background:#eaf1ff;} ");
  h += F(".back{margin-top:12px;} ");
  h += F("canvas.spark{width:100%;height:180px;border:1px solid #ddd;border-radius:8px;background:#fff;margin-top:8px;} ");
  h += F(".muted{color:#666;font-size:13px;} ");
  h += F("</style></head><body>");
}

static void htmlPageEnd(String &h) {
  h += F("</body></html>");
}

String buildStatusPageHtml() {
  String h;
  h.reserve(16000);
  htmlPageBegin(h, "CO2 Sensor");

  h += F("<h2>CO2 Sensor v");
  h += MY_VERSION;
  h += F("</h2>");

  h += F("<div class='topnav'>");
  h += F("<a class='btn btn-primary' href='/sensor'>Sensor</a>");
  h += F("<a class='btn' href='/led'>LED Shelly</a>");
  h += F("<a class='btn' href='/system'>System Config</a>");
  h += F("</div>");

  h += F("<div class='box'>Use the buttons above to view Sensor data, LED/Shelly diagnostics, or System configuration.</div>");

  const String ssid = WiFi.SSID();
  const String ip = WiFi.localIP().toString();
  const long rssi = WiFi.RSSI();

  h += F("<div class='box'><b>ESP32 Wireless Connection</b><br/>");
  h += F("<span class='k'>SSID:</span> ");
  h += htmlEscape(ssid);
  h += F("<br/>");
  h += F("<span class='k'>IP:</span> ");
  h += htmlEscape(ip);
  h += F("<br/>");
  h += F("<span class='k'>RSSI:</span> ");
  h += String(rssi);
  h += F(" dBm<br/>");
#if ENABLE_MDNS
  h += F("<span class='k'>ESP mDNS:</span> <code>http://");
  h += htmlEscape(cfg.hostname);
  h += F(".local/</code><br/>");
#endif
  h += F("</div>");

  h += F("<div class='box'><b>Recent History</b><br/>");
  h += F("<div class='muted'>Recent CO2, temperature, and pressure samples are kept in RAM for the web graph. Long-term history stays on the SD card CSV logs.</div>");
  h += F("<canvas id='co2Chart' class='spark' width='800' height='180'></canvas>");
  h += F("<canvas id='tempChart' class='spark' width='800' height='180'></canvas>");
  h += F("<canvas id='pressureChart' class='spark' width='800' height='180'></canvas>");
  h += F("<div id='historyMeta' class='muted'>Loading history...</div>");
  h += F(R"rawliteral(<script>
(function(){
  const MAX_POINTS = 120;
  const SAMPLE_MS = )rawliteral");
  h += String(CO2_PERIOD_MS);
  h += F(R"rawliteral(;
  function drawChart(id,label,unit,key){
    const canvas=document.getElementById(id);
    const ctx=canvas.getContext('2d');
    return function(data){
      const pts=(data&&data.points)||[];
      ctx.clearRect(0,0,canvas.width,canvas.height);
      ctx.fillStyle='#111';
      ctx.font='12px Arial';
      ctx.fillText(label,8,14);
      if(!pts.length){ctx.fillText('No data yet',8,32);return;}
      const vals=pts.map(p=>p[key]).filter(v=>v!==null&&v!==undefined);
      if(!vals.length){ctx.fillText('No valid data',8,32);return;}
      let min=Math.min.apply(null,vals), max=Math.max.apply(null,vals);
      if(min===max){min-=1;max+=1;}
      const L=48,R=10,T=22,B=18,W=canvas.width-L-R,H=canvas.height-T-B;
      ctx.strokeStyle='#d0d0d0';
      ctx.lineWidth=1;
      ctx.beginPath();
      ctx.moveTo(L,T); ctx.lineTo(L,T+H); ctx.lineTo(L+W,T+H); ctx.stroke();
      ctx.fillStyle='#666';
      ctx.fillText(max.toFixed(1)+' '+unit,6,T+4);
      ctx.fillText(min.toFixed(1)+' '+unit,6,T+H);
      ctx.strokeStyle='#0a66c2';
      ctx.lineWidth=2;
      let started=false;
      pts.forEach((p,i)=>{
        const v=p[key];
        if(v===null||v===undefined) return;
        const x=L+(pts.length===1?W/2:(i*(W/(pts.length-1))));
        const y=T+H-((v-min)/(max-min))*H;
        if(!started){ctx.beginPath();ctx.moveTo(x,y);started=true;} else {ctx.lineTo(x,y);} 
      });
      if(started) ctx.stroke();
    };
  }
  const drawCo2=drawChart('co2Chart','CO2','ppm','co2');
  const drawTemp=drawChart('tempChart','Temperature','C','temp');
  const drawPressure=drawChart('pressureChart','Pressure','hPa','pressure');
  async function load(){
    try{
      const res=await fetch('/history.json?points='+MAX_POINTS,{cache:'no-store'});
      const data=await res.json();
      drawCo2(data); drawTemp(data); drawPressure(data);
      const pts=(data&&data.points)||[];
      const shown=pts.length;
      const count=(data&&data.count)||0;
      const sampleSec=(SAMPLE_MS/1000).toFixed(1);
      const spanMin=shown?(((shown-1)*SAMPLE_MS)/60000).toFixed(1):'0.0';
      let msg='Sampling: every '+sampleSec+' s | Showing: '+shown+' of '+count+' samples';
      msg+=' | RAM capacity: '+((data&&data.capacity)||0);
      msg+=' | Approx span: '+spanMin+' min';
      if(pts.length){
        const last=pts[pts.length-1];
        msg += last.tv ? ' | Time source: NTP' : ' | Time source: uptime';
      }
      document.getElementById('historyMeta').textContent=msg;
    }catch(e){
      document.getElementById('historyMeta').textContent='History load failed';
    }
  }
  load();
  setInterval(load,15000);
})();
</script>)rawliteral");
  h += F("</div>");

  htmlPageEnd(h);
  return h;
}

String buildSensorPageHtml() {
  String h;
  h.reserve(17000);
  htmlPageBegin(h, "Sensor");

  h += F("<div class='topnav'>");
  h += F("<a class='btn btn-primary' href='/sensor'>Sensor</a>");
  h += F("<a class='btn' href='/led'>LED Shelly</a>");
  h += F("<a class='btn' href='/system'>System Config</a>");
  h += F("</div>");

  h += F("<h2>Sensor</h2>");

  bool frcPending = (cfg.scd30_frc_ppm >= 0);
  if (frcPending) {
    h += F("<div class='banner warn'><b>Warning:</b> FRC pending (");
    h += String(cfg.scd30_frc_ppm);
    h += F(" ppm). ASC is forced OFF until calibration is applied.</div>");
  }

  h += F("<div class='box'><b>SCD30 Sensor Readings</b><br/>");
  h += F("<span class='k'>Temperature:</span> ");
  h += String(scd30_T_s, 1);
  h += F(" &deg;C<br/>");
  h += F("<span class='k'>Humidity:</span> ");
  h += String(scd30_H_s, 1);
  h += F(" %<br/>");
  h += F("<span class='k'>CO2 raw:</span> ");
  h += String(lastCO2raw, 0);
  h += F(" ppm<br/>");
  h += F("<span class='k'>CO2 EMA:</span> ");
  h += String(lastCO2ema, 1);
  h += F(" ppm<br/>");
  h += F("<span class='k'>CO2 used-for-color:</span> ");
  h += String(lastCO2forColor, 1);
  h += F(" ppm<br/>");
  h += F("<span class='k'>Hysteresis holding?:</span> ");
  h += (lastCO2Adjusted ? "YES" : "NO");
  h += F("<br/>");
  h += F("</div>");

  h += F("<div class='box'><b>");
  h += htmlEscape(bmeSensorName);
  h += F(" Ambient Sensor</b><br/>");
  h += F("<span class='k'>Present:</span> ");
  h += (bme_sensor_found ? "YES" : "NO");
  h += F("<br/>");
  h += F("<span class='k'>Temperature:</span> ");
  h += isfinite(bme_T_s) ? String(bme_T_s, 1) : String("n/a");
  h += F(" &deg;C<br/>");
  h += F("<span class='k'>Humidity:</span> ");
  if (bme_has_humidity) h += isfinite(bme_H_s) ? String(bme_H_s, 1) : String("n/a");
  else h += F("n/a (BMP280)");
  h += F(" %<br/>");
  h += F("<span class='k'>Pressure:</span> ");
  h += isfinite(bme_P_s) ? String(bme_P_s, 1) : String("n/a");
  h += F(" hPa<br/>");
  h += F("<span class='k'>Altitude:</span> ");
  h += isfinite(alt_s) ? String(alt_s, 1) : String("n/a");
  h += F(" m<br/>");
  h += F("</div>");

  String ascUptime = "n/a";
  if (cfg.scd30_asc && ascEnabledEpoch > 0 && timeIsValid()) {
    time_t now = time(nullptr);
    long sec = (long)(now - ascEnabledEpoch);
    if (sec < 0) sec = 0;
    long hr = sec / 3600;
    long mn = (sec % 3600) / 60;
    long ss = sec % 60;
    ascUptime = String(hr) + "h " + String(mn) + "m " + String(ss) + "s";
  }

  h += F("<div class='box'><b>Sensor Calibration</b><br/>");
  h += F("<span class='k'>ASC enabled:</span> ");
  h += (cfg.scd30_asc ? "YES" : "NO");
  h += F("<br/>");
  h += F("<span class='k'>ASC uptime:</span> ");
  h += htmlEscape(ascUptime);
  h += F("<br/>");
  h += F("<span class='k'>FRC pending:</span> ");
  h += (cfg.scd30_frc_ppm >= 0 ? String(cfg.scd30_frc_ppm) : "none");
  h += F("<br/>");
  h += F("<span class='k'>Last calibration:</span> ");
  h += htmlEscape(fmtTime(lastCalEpoch));
  h += F("<br/>");
  h += F("<span class='k'>Reason:</span> ");
  h += htmlEscape(lastCalReason);
  h += F("<br/>");
  h += F("<form method='POST' action='/calibrate400' style='margin-top:8px;'>");
  h += F("<button type='submit'>Calibrate now (400 ppm)</button>");
  h += F("</form>");
  h += F("</div>");

  h += F("<div class='box back'><a href='/'>Back to Menu</a></div>");

  htmlPageEnd(h);
  return h;
}

String buildLedShellyPageHtml() {
  String h;
  h.reserve(9000);
  htmlPageBegin(h, "LED Shelly");

  h += F("<div class='topnav'>");
  h += F("<a class='btn' href='/sensor'>Sensor</a>");
  h += F("<a class='btn btn-primary' href='/led'>LED Shelly</a>");
  h += F("<a class='btn' href='/system'>System Config</a>");
  h += F("</div>");

  h += F("<h2>LED / Shelly</h2>");

  h += F("<div class='box'><b>LED / Shelly</b><br/>");
  h += F("<span class='k'>Configured LED host:</span> ");
  h += htmlEscape(cfg.led_host);
  h += F("<br/>");
  h += F("<span class='k'>Host source:</span> ");
  h += (ledHostFromNVS ? "NVS override" : "Secrets default");
  h += F("<br/>");
  h += F("<span class='k'>Resolve state:</span> ");
  h += htmlEscape(ledResolveState);
  h += F("<br/>");
  h += F("<span class='k'>Cached IP:</span> ");
  h += (led_cached_ip_valid ? htmlEscape(led_cached_ip.toString()) : "none");
  h += F("<br/>");
  h += F("<span class='k'>Last Shelly status:</span> ");
  h += htmlEscape(led_update_status);
  h += F("<br/>");
  h += F("<span class='k'>Last HTTP code:</span> ");
  h += String(lastShellyHttpCode);
  h += F("<br/>");
  h += F("<span class='k'>Last host used:</span> ");
  h += htmlEscape(lastShellyHostUsed);
  h += F("<br/>");
  h += F("<span class='k'>Last URL sent:</span> <code>");
  h += htmlEscape(lastShellyUrlSent);
  h += F("</code><br/>");
  h += F("</div>");

  h += F("<div class='box'><b>Color</b><br/>");
  h += F("<span class='k'>Mode:</span> ");
  h += htmlEscape(colorModeToStr(cfg.color_mode));
  h += F("<br/>");
  h += F("<span class='k'>Hysteresis:</span> ");
  h += (cfg.co2_hyst_enabled ? "ON" : "OFF");
  h += F("<br/>");
  h += F("<span class='k'>RGB last:</span> ");
  h += String(lastR);
  h += ",";
  h += String(lastG);
  h += ",";
  h += String(lastB);
  h += F("<br/>");

  if (lastR >= 0 && lastG >= 0 && lastB >= 0) {
    String css = cssRgbTriplet(lastR, lastG, lastB);
    h += F("<div style='margin-top:10px; display:flex; align-items:center; gap:10px;'>");
    h += F("<div style='width:44px; height:22px; border:1px solid #888; border-radius:6px; background:");
    h += htmlEscape(css);
    h += F("'></div>");
    h += F("<div><span class='k'>Result:</span> <code>");
    h += htmlEscape(css);
    h += F("</code></div>");
    h += F("</div>");
  } else {
    h += F("<div style='margin-top:10px; color:#777;'>No RGB value yet.</div>");
  }

  h += F("</div>");

  h += F("<div class='box back'><a href='/'>Back to Menu</a></div>");

  htmlPageEnd(h);
  return h;
}

String buildSystemConfigPageHtml() {
  String h;
  h.reserve(12000);
  htmlPageBegin(h, "System Config");

  h += F("<div class='topnav'>");
  h += F("<a class='btn' href='/sensor'>Sensor</a>");
  h += F("<a class='btn' href='/led'>LED Shelly</a>");
  h += F("<a class='btn btn-primary' href='/system'>System Config</a>");
  h += F("</div>");

  h += F("<h2>System Config</h2>");
  h += F("<div class='box'><a href='/config'>Open Config Editor</a></div>");

  h += F("<div class='box'><b>Network Time Protocol</b><br/>");
  h += F("<span class='k'>Server:</span> "); h += htmlEscape(ntpServerUsed); h += F("<br/>");
  h += F("<span class='k'>Configured:</span> "); h += (ntpConfigured ? "YES" : "NO"); h += F("<br/>");
  h += F("<span class='k'>Synced:</span> "); h += (ntpSynced ? "YES" : "NO"); h += F("<br/>");
  h += F("<span class='k'>Epoch now:</span> "); h += String((long)time(nullptr)); h += F("<br/>");
  h += F("<span class='k'>Last sync:</span> "); h += htmlEscape(fmtTime(ntpLastSyncEpoch)); h += F("<br/>");
  h += F("</div>");

  h += F("<div class='box'><b>NVS / Config</b><br/>");
  h += F("<span class='k'>ssid:</span> "); h += htmlEscape(cfg.ssid); h += F(" (src: "); h += src(key_ssid); h += F(")<br/>");
  h += F("<span class='k'>ssid_pwd:</span> ******** (src: "); h += src(key_pass); h += F(")<br/>");
  h += F("<span class='k'>led:</span> "); h += htmlEscape(cfg.led_host); h += F(" (src: "); h += src(key_led); h += F(")<br/>");
  h += F("<span class='k'>hostname:</span> "); h += htmlEscape(cfg.hostname); h += F(" (src: "); h += src(key_hostname); h += F(")<br/>");
  h += F("<span class='k'>altitude:</span> "); h += String(cfg.altitude_m); h += F(" (src: "); h += src(key_altitude); h += F(")<br/>");
  h += F("<span class='k'>verbose:</span> "); h += (cfg.verbose ? "true" : "false"); h += F(" (src: "); h += src(key_verbose); h += F(")<br/>");
  h += F("<span class='k'>color_mode:</span> "); h += htmlEscape(colorModeToStr(cfg.color_mode)); h += F(" (src: "); h += src(key_color_mode); h += F(")<br/>");
  h += F("<span class='k'>hyst:</span> "); h += (cfg.co2_hyst_enabled ? "ON" : "OFF"); h += F(" (src: "); h += src(key_hyst); h += F(")<br/>");
  h += F("<span class='k'>lan_only:</span> "); h += (cfg.lan_only ? "ON" : "OFF"); h += F(" (src: "); h += src(key_lan_only); h += F(")<br/>");
  h += F("<span class='k'>auth_pass:</span> ******** (src: "); h += (key_auth_pass ? "NVS" : "WiFi fallback"); h += F(")<br/>");
  h += F("<span class='k'>scd30_asc:</span> "); h += (cfg.scd30_asc ? "ON" : "OFF"); h += F(" (src: "); h += (key_scd30_asc ? "NVS" : "Default"); h += F(")<br/>");
  h += F("<span class='k'>scd30_frc:</span> "); h += (cfg.scd30_frc_ppm >= 0 ? String(cfg.scd30_frc_ppm) : "none"); h += F(" (src: "); h += (key_scd30_frc ? "NVS" : "None"); h += F(")<br/>");
  h += F("<span class='k'>sd_keep_files:</span> "); h += String((unsigned long)cfg.sd_keep_files); h += F(" (src: "); h += (key_sd_keep_files ? "NVS" : "Secrets/default"); h += F(")<br/>");
  h += F("<span class='k'>screensave_time_minutes:</span> "); h += String((unsigned long)cfg.screensave_time_minutes); h += F(" (src: "); h += (key_screensave_minutes ? "NVS" : "Secrets"); h += F(")<br/>");
  h += F("</div>");

  h += F("<div class='box'><b>SD Logging Config</b><br/>");
  h += F("<span class='k'>enabled:</span> "); h += (cfg.sd_log_enable ? "ON" : "OFF"); h += F(" (src: "); h += (key_sd_log_enable ? "NVS" : "Secrets"); h += F(")<br/>");
  h += F("<span class='k'>period:</span> "); h += String((unsigned long)cfg.sd_log_period_sec); h += F(" s (src: "); h += (key_sd_log_period ? "NVS" : "Secrets"); h += F(")<br/>");
  h += F("<span class='k'>rotation:</span> "); h += htmlEscape(cfg.sd_rotation_cfg); h += F(" (src: "); h += (key_sd_rotation ? "NVS" : "Secrets"); h += F(")<br/>");
  h += F("<span class='k'>SD Keep last x files:</span> "); h += String((unsigned long)cfg.sd_keep_files); h += F(" (src: "); h += (key_sd_keep_files ? "NVS" : "Secrets/default"); h += F(")<br/>");
  h += F("<span class='k'>card ready:</span> "); h += (sd_card_ready ? "YES" : "NO"); h += F("<br/>");
  h += F("<span class='k'>current path:</span> "); h += htmlEscape(sd_current_log_path); h += F("<br/>");
  h += F("</div>");

  h += F("<div class='box back'><a href='/'>Back to Menu</a></div>");

  htmlPageEnd(h);
  return h;
}

void setupAsyncWebServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html; charset=utf-8", buildStatusPageHtml());
  });

  server.on("/sensor", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html; charset=utf-8", buildSensorPageHtml());
  });

  server.on("/history.json", HTTP_GET, [](AsyncWebServerRequest *request) {
    sendHistoryJson(request);
  });

  server.on("/led", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html; charset=utf-8", buildLedShellyPageHtml());
  });

  server.on("/system", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!requireAdmin(request)) return;
    request->send(200, "text/html; charset=utf-8", buildSystemConfigPageHtml());
  });

  server.on("/health", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain; charset=utf-8", "OK\n");
  });

  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!requireAdmin(request)) return;
    request->send(200, "text/html; charset=utf-8", buildConfigEditorHtml());
  });

  server.on("/config", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!requireAdmin(request)) return;

    Config oldCfg = cfg;

    auto getArg = [&](const char* name) -> String {
      if (!request->hasParam(name, true)) return "";
      return request->getParam(name, true)->value();
    };

    String ssid     = getArg("ssid"); ssid.trim();
    String pass     = getArg("pass"); pass.trim();
    String hostname = getArg("hostname"); hostname.trim();
    String led      = getArg("led_host"); led.trim();
    String altitude = getArg("altitude"); altitude.trim();
    String report   = getArg("report"); report.trim();
    String rptTxt   = getArg("report_txt"); rptTxt.trim();
    String cmode    = getArg("color_mode"); cmode.trim();
    String hyst     = getArg("hyst"); hyst.trim();
    String verb     = getArg("verbose"); verb.trim();
    String lan      = getArg("lan_only"); lan.trim();
    String authp    = getArg("auth_pass"); authp.trim();
    String asc      = getArg("scd30_asc"); asc.trim();
    String frc      = getArg("scd30_frc"); frc.trim();
    String sdEn     = getArg("sd_log_enable"); sdEn.trim();
    String sdSec    = getArg("sd_log_period_sec"); sdSec.trim();
    String sdRot    = getArg("sd_rotation_cfg"); sdRot.trim();
    String sdKeep   = getArg("sd_keep_files"); sdKeep.trim();
    String scrMin   = getArg("screensave_time_minutes"); scrMin.trim();
    String touchPin = getArg("touch_wake_pin"); touchPin.trim();

    if (ssid.length()) cfg.ssid = ssid;

    if (pass.length()) {
      cfg.pass = pass;
      if (!key_auth_pass && authp.length() == 0) cfg.auth_pass = cfg.pass;
    }

    if (hostname.length()) cfg.hostname = hostname;

    if (led.length()) {
      cfg.led_host = sanitizeHost(led);
      led_cached_ip_valid = false;
      led_cached_ip = IPAddress(0,0,0,0);
      ledResolveState = "unknown";
    }

    if (altitude.length()) cfg.altitude_m = altitude.toInt();
    if (report.length()) cfg.report = report;
    if (rptTxt.length()) cfg.report_txt = rptTxt;
    if (cmode.length()) cfg.color_mode = parseColorMode(cmode);

    if (hyst.length()) {
      hyst.toUpperCase();
      cfg.co2_hyst_enabled = (hyst == "ON");
    }

    if (verb.length()) {
      verb.toUpperCase();
      cfg.verbose = (verb == "ON");
    }

    if (lan.length()) {
      lan.toUpperCase();
      cfg.lan_only = (lan == "ON");
    }

    if (authp.length()) {
      cfg.auth_pass = authp;
    }

    if (asc.length()) {
      asc.toUpperCase();
      cfg.scd30_asc = (asc == "ON");
    }

    if (request->hasParam("scd30_frc", true)) {
      if (frc.length() == 0) {
        cfg.scd30_frc_ppm = -1;
      } else {
        int v = frc.toInt();
        if (v < 350) v = 350;
        if (v > 2000) v = 2000;
        cfg.scd30_frc_ppm = (int16_t)v;
      }
    }

    if (sdEn.length()) {
      sdEn.toUpperCase();
      cfg.sd_log_enable = (sdEn == "ON");
    }

    if (sdSec.length()) {
      uint32_t v = (uint32_t)sdSec.toInt();
      if (v == 0) v = 60;
      cfg.sd_log_period_sec = v;
    }

    if (sdRot.length()) {
      sdRot.toUpperCase();
      if (sdRot != "DAY" && sdRot != "MONTH" && sdRot != "OFF") sdRot = "OFF";
      cfg.sd_rotation_cfg = sdRot;
    }

    if (sdKeep.length()) {
      uint32_t v = (uint32_t)sdKeep.toInt();
      if (v > 3650UL) v = 3650UL;
      cfg.sd_keep_files = (uint16_t)v;
    }

    if (scrMin.length()) {
      uint32_t v = (uint32_t)scrMin.toInt();
      if (v > 1440UL) v = 1440UL;
      cfg.screensave_time_minutes = (uint16_t)v;
    }

    if (touchPin.length()) {
      int32_t v = (int32_t)touchPin.toInt();
      if (v < -1) v = -1;
      if (v > 39) v = 39;
      cfg.touch_wake_pin = (int16_t)v;
    }

    if (cfg.scd30_frc_ppm >= 0 && cfg.scd30_asc) {
      cfg.scd30_asc = false;
    }

    saveConfigIfChanged(oldCfg);

    request->send(200, "text/html; charset=utf-8",
                  "<html><body><h3>Saved to NVS.</h3>"
                  "<p>No reboot was performed.</p>"
                  "<p><a href='/config'>Back to Config</a> | <a href='/'>Menu</a></p></body></html>");
  });

  server.on("/exportconfigsd", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!requireAdmin(request)) return;

    String msg;
    bool ok = exportConfigSnapshotToSd(msg);

    String html;
    html.reserve(600);
    html += F("<html><body><h3>");
    html += ok ? F("Config export complete") : F("Config export failed");
    html += F("</h3><p>");
    html += htmlEscape(msg);
    html += F("</p><p><a href='/system'>Back to System Config</a> | <a href='/config'>Back to Edit Config</a></p></body></html>");
    request->send(ok ? 200 : 500, "text/html; charset=utf-8", html);
  });

  server.on("/importconfigsd", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!requireAdmin(request)) return;

    String msg;
    bool ok = importConfigSnapshotFromSd(msg);

    String html;
    html.reserve(700);
    html += F("<html><body><h3>");
    html += ok ? F("SD config loaded into NVS") : F("SD config load failed");
    html += F("</h3><p>");
    html += htmlEscape(msg);
    html += F("</p><p>Review the values in the GUI, then use the existing reboot action when ready.</p>");
    html += F("<p><a href='/system'>Back to System Config</a> | <a href='/config'>Back to Edit Config</a></p></body></html>");
    request->send(ok ? 200 : 500, "text/html; charset=utf-8", html);
  });

  server.on("/calibrate400", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!requireAdmin(request)) return;

    g_requestFrcNow400 = true;

    request->send(200, "text/html; charset=utf-8",
      "<html><body><h3>Calibration requested: FRC 400 ppm</h3>"
      "<p>The device will apply this shortly in the main loop (safe context).</p>"
      "<p>Ensure the sensor is in fresh outdoor air (~400 ppm). Wait 1-2 minutes for stability.</p>"
      "<p><a href='/sensor'>Back to Sensor</a> | <a href='/system'>System Config</a></p>"
      "</body></html>");
  });

  server.on("/reboot", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!requireAdmin(request)) return;
    request->send(200, "text/plain; charset=utf-8", "Rebooting...\n");
    delay(200);
    ESP.restart();
  });

  server.on("/factoryreset", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!requireAdmin(request)) return;
    request->send(200, "text/plain; charset=utf-8", "Wiping NVS and rebooting...\n");
    delay(200);
    wipePreferencesNVS();
  });

  server.begin();
}

void showStartupConfigOLED(uint32_t ms = 5000) {
  oledTurnOnIfNeeded(false);
  if (!display.width()) return;

  const char* srcLed  = ledHostFromNVS ? "NVS" : "SEC";
  const char* srcSsid = key_ssid ? "NVS" : "SEC";

  String ssidStr = cfg.ssid;
  if (ssidStr.length() > 16) ssidStr = ssidStr.substring(0, 16);

  String ledHostStr = cfg.led_host;
  if (ledHostStr.length() > 16) ledHostStr = ledHostStr.substring(0, 16);

  String ledIpStr = led_cached_ip_valid ? led_cached_ip.toString() : ledResolveState;

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);

  display.setCursor(0, 0);   display.print("Host: ");  display.println(cfg.hostname);
  display.setCursor(0, 9);   display.print("SSID("); display.print(srcSsid); display.print("): "); display.println(ssidStr);
  display.setCursor(0, 17);  display.print("LED("); display.print(srcLed); display.print("): "); display.println(ledHostStr);
  display.setCursor(0, 25);  display.print("LED IP:"); display.println(ledIpStr);

  display.setCursor(0, 34);
  display.print("ASC:"); display.print(cfg.scd30_asc ? "ON" : "OFF");
  display.print(" FRC:");
  display.print(cfg.scd30_frc_ppm >= 0 ? String(cfg.scd30_frc_ppm) : String("-"));

  display.setCursor(0, 43);
  display.print("DBG:"); display.print(cfg.verbose ? "ON" : "OFF");
  display.print(" LAN:"); display.print(cfg.lan_only ? "ON" : "OFF");

  display.setCursor(0, 52);
  display.print("SCR:"); display.print((unsigned long)cfg.screensave_time_minutes);
  display.print(" T:"); display.print((long)cfg.touch_wake_pin);

  display.display();
  delay(ms);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_FREQ_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);

  pinMode(OLED_RESET_PIN, OUTPUT);
  digitalWrite(OLED_RESET_PIN, HIGH);
  delay(10);
  digitalWrite(OLED_RESET_PIN, LOW);
  delay(10);
  digitalWrite(OLED_RESET_PIN, HIGH);

  if (!display.begin(SCREEN_ADDRESS, true)) {
    Serial.print("OLED Display not found at ");
    Serial.print(SCREEN_ADDRESS);
    Serial.println(". Continuing without display.");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setRotation(1);
    display.display();
  }

  Serial.println();
  Serial.println("CO2 Ventilation Sensor");
  Serial.print("Version: ");
  Serial.println(MY_VERSION);
  Serial.print("I2C SDA: "); Serial.println(I2C_SDA_PIN);
  Serial.print("I2C SCL: "); Serial.println(I2C_SCL_PIN);
  Serial.print("I2C clock: "); Serial.print(I2C_FREQ_HZ); Serial.println(" Hz");
  Serial.print("I2C timeout: "); Serial.print(I2C_TIMEOUT_MS); Serial.println(" ms");

  bootWindow();
  loadConfig();
  Serial.print("Screensave time: "); Serial.print((unsigned long)cfg.screensave_time_minutes); Serial.println(" min");
  Serial.print("Touch wake pin: "); Serial.println((long)cfg.touch_wake_pin);
  runMenuWindow(10000);

  resetOledAwakeDeadline();
  calibrateOledTouchWake();

  showStartupConfigOLED(2000);

  oledMessage("Init sensors...", "SCD30 + BME280");
  const uint32_t t0 = millis();

  if (!initSCD30Robust((float)cfg.altitude_m)) {
    Serial.println("FATAL: SCD30 init failed.");
    oledMessage("FATAL", "SCD30 init failed", "Check serial");
    while (true) delay(50);
  }

  if (cfg.verbose) {
    Serial.print("SCD30 init+verify at ms=");
    Serial.println(millis() - t0);
  }

  if (!bme.begin()) {
    Serial.println("BME280/BMP280 not found. Altitude from config only.");
    bme_sensor_found = false;
    bme_has_humidity = false;
    bmeSensorName = "NONE";
  } else {
    bme_sensor_found = true;

    switch (bme.chipModel()) {
      case BME280::ChipModel_BME280:
        bme_has_humidity = true;
        bmeSensorName = "BME280";
        Serial.println("Found BME280 sensor! Success.");
        break;
      case BME280::ChipModel_BMP280:
        bme_has_humidity = false;
        bmeSensorName = "BMP280";
        Serial.println("Found BMP280 sensor! No Humidity available.");
        break;
      default:
        bme_has_humidity = false;
        bmeSensorName = "UNKNOWN";
        Serial.println("Found UNKNOWN sensor! Error!");
        break;
    }

    if (readBMEandUpdateAltitude()) {
      printBmeSensorDataVerbose();
      if (isfinite(alt_s)) {
        Serial.print("Startup altitude from ");
        Serial.print(bmeSensorName);
        Serial.print(": ");
        Serial.print((int)lroundf(alt_s));
        Serial.println(" m");
      }
      showBmeStartupInfoOLED(1800);
    }
  }

  initSdLogging();

  oledMessage("WiFi...", cfg.ssid);
  wifiConnectBlocking(15000);

#if ENABLE_MDNS
  mdnsEnsureRunning();
#endif

  ntpConfigureIfNeeded();
  resolveLedAtStartupBlocking();
  setupAsyncWebServer();

  frcPendingBanner = (cfg.scd30_frc_ppm >= 0);

  showStartupConfigOLED(2500);

  Serial.println("Setup complete.");
  oledMessage("Setup complete", "Running...");
  resetOledAwakeDeadline();
}

void loop() {
  const uint32_t now = millis();

  handleOledScreensaverNonBlocking();

  wifiEnsureConnectedNonBlocking();

#if ENABLE_MDNS
  mdnsEnsureRunning();
#endif

  ntpConfigureIfNeeded();
  ntpPollSyncNonBlocking();

  if (g_requestFrcNow400) {
    g_requestFrcNow400 = false;
    bool ok = applyFrcNow400();
    if (cfg.verbose) Serial.printf("FRC request handled -> %s\n", ok ? "OK" : "FAIL");
  }

  if (now - tCO2 >= CO2_PERIOD_MS) {
    tCO2 = now;

    if (bme_sensor_found) {
      if (readBMEandUpdateAltitude()) {
        printBmeSensorDataVerbose();
      } else if (cfg.verbose) {
        Serial.println("BME280 read failed.");
      }
    }

    float co2 = NAN, temp = NAN, rh = NAN;
    int16_t err = scd30.blockingReadMeasurementData(co2, temp, rh);

    if (err != NO_ERROR) {
      logScd30Error("blockingReadMeasurementData", err);
    } else {
      if (isfinite(co2)) lastCO2raw = co2;
      printScd30SensorDataVerbose(co2, temp, rh);

      if (!isfinite(co2) || co2 < SCD30_CO2_MIN_VALID_PPM) {
        if (isfinite(co2)) lastCO2RejectedRaw = co2;
        if (cfg.verbose) {
          Serial.print("SCD30 CO2 invalid/implausible (<");
          Serial.print(SCD30_CO2_MIN_VALID_PPM, 0);
          Serial.print(" ppm or NaN) -> sample ignored");
          if (isfinite(co2)) {
            Serial.print(" [RAW REJECTED: ");
            Serial.print(co2, 1);
            Serial.print(" ppm]");
          }
          Serial.println();
        }
      } else {
        // Keep runtime settings/FRC logic tied to the first valid sensor data,
        // but delay user-facing CO2 output decisions for a few startup samples.
        scd30_T_s = temp;
        scd30_H_s = rh;

        float scd30_CO2_valid_for_runtime = co2;

        if (isnan(scd30_CO2_s)) {
          scd30_CO2_s = scd30_CO2_valid_for_runtime;
        }

        maybeApplySCD30RuntimeSettingsNonBlocking();

        if (cfg.scd30_frc_ppm >= 0) {
          maybeApplyPendingFrcNonBlocking();
        }

        if (!scd30_outputs_ready) {
          scd30_valid_sample_count++;
          if (cfg.verbose) {
            Serial.print("SCD30 warmup sample ");
            Serial.print(scd30_valid_sample_count);
            Serial.print("/");
            Serial.println(SCD30_WARMUP_SAMPLES);
          }

          if (scd30_valid_sample_count >= SCD30_WARMUP_SAMPLES) {
            scd30_outputs_ready = true;
            co2_ema = scd30_CO2_valid_for_runtime;
            co2_for_color = scd30_CO2_valid_for_runtime;
            scd30_CO2_s = scd30_CO2_valid_for_runtime;
            lastCO2ema = co2_ema;
            lastCO2forColor = co2_for_color;
            lastCO2Adjusted = false;
            printSensorData();
            float historyTemp = (bme_sensor_found && isfinite(bme_T_s)) ? bme_T_s : scd30_T_s;
            historyAddSample(scd30_CO2_s, historyTemp, bme_P_s);
          }
        } else {
          scd30_CO2_s = scd30_CO2_valid_for_runtime;
          printSensorData();
          float historyTemp = (bme_sensor_found && isfinite(bme_T_s)) ? bme_T_s : scd30_T_s;
          historyAddSample(scd30_CO2_s, historyTemp, bme_P_s);

          float used = co2FilterAndHysteresis(scd30_CO2_s);
          lastCO2ema      = co2_ema;
          lastCO2forColor = used;
        }
      }
    }
  }

  float displayTemp = (bme_sensor_found && isfinite(bme_T_s)) ? bme_T_s : scd30_T_s;
  float displayHum  = (bme_sensor_found && bmeHumidityLooksValid()) ? bme_H_s : scd30_H_s;

  maybeLogSensorsToSd(displayTemp, displayHum);

  if (oled_is_on && (now - tOLED >= OLED_PERIOD_MS)) {
    tOLED = now;
    displayOLED(displayTemp, displayHum, scd30_outputs_ready ? scd30_CO2_s : NAN);
  }

  setLightsFromCO2(scd30_outputs_ready ? scd30_CO2_s : NAN);
}
