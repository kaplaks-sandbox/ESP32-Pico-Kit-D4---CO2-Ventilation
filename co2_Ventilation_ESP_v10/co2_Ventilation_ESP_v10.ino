// ESP32-PICO-D4 CO2 Ventilation Sensor
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

#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <math.h>
#include <time.h>

#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>
//#define  WHITE SSD1306_WHITE 

#include <Adafruit_SH110X.h>
//https://github.com/adafruit/Adafruit_SH110x
#define  WHITE SH110X_WHITE 

//#include <Adafruit_SCD30.h>
#include <SensirionI2cScd30.h>
//#include <sensirion_common.h>

//https://github.com/finitespace/BME280
#include <BME280I2C.h>
#include <EnvironmentCalculations.h>

#include "arduino_secrets.h"

// -------- Optional mDNS --------
#ifndef ENABLE_MDNS
  #define ENABLE_MDNS 1
#endif

#if ENABLE_MDNS
  #include <ESPmDNS.h>
#endif

// -------- Async Web Server --------
//https://github.com/ESP32Async/AsyncTCP
#include <AsyncTCP.h>
//https://github.com/ESP32Async/ESPAsyncWebServer
#include <ESPAsyncWebServer.h>

// -----------------------------
// Version
// -----------------------------
static const char* MY_VERSION = "10.0.0";

// -----------------------------
// CO2 smoothing / anti-flicker tuning
// -----------------------------
static constexpr float CO2_EMA_ALPHA = 0.20f; // 0..1
static constexpr float CO2_HYST_PPM  = 25.0f; // deadband (only when hysteresis enabled)

// -----------------------------
// IMPORTANT: types MUST appear before any functions (Arduino auto-prototype issues)
// -----------------------------
enum class ColorMode : uint8_t {
  AQI = 0,
  LEGACY_WAVELENGTH = 1
};

struct Config {
  String ssid;
  String pass;
  String led_host;     // IP, FQDN, or *.local (sanitized)
  String hostname;     // ESP hostname (mDNS too)
  int    altitude_m;
  String report;       // A/T/H/C
  String report_txt;   // Y/N
  bool   verbose;

  bool co2_hyst_enabled; // ON/OFF (persisted)
  ColorMode color_mode;      // AQI or LEGACY

  String auth_pass;   // HTTP Basic Auth password (optional). If empty -> WiFi pass fallback
  bool   lan_only;    // LAN-only guard enable/disable (persisted). Default from secrets LAN_only

  // ---- Calibration ----
  bool     scd30_asc;        // persisted
  int16_t  scd30_frc_ppm;    // persisted; -1 none pending

  // ---- Time / NTP ----
  String   ntp_server;       // derived: gw ip string or fallback
};

static const char* colorModeToStr(ColorMode m) {
  return (m == ColorMode::AQI) ? "AQI" : "LEGACY";
}

// -----------------------------
// Environmental assumptions
// -----------------------------
float referencePressure = 1013.25f;                   // hPa
float outdoorTemp = 4.7f;                             // °C
float barometerAltitude = SECRET_BAROMETERAltitude_m; // meters

// -----------------------------
// OLED
// -----------------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET_PIN 33
#define SCREEN_ADDRESS 0x3C   //Sensor 1 to 3 = 3D, Sensor 4 = 77
//Aafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET_PIN);
//Adafruit_SH1107 display = Adafruit_SH1107(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET_PIN, 1000000, 100000);
// SH1107 panel native geometry is 64x128
Adafruit_SH1107 display(64, 128, &Wire, OLED_RESET_PIN, 400000, 100000);


// -----------------------------
// BME280 I2C settings
// -----------------------------
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
float alt_s = NAN;

// -----------------------------
// SCD30
// -----------------------------

#ifndef SCD30_I2C_ADDR_61
  #define SCD30_I2C_ADDR_61 0x61
#endif


//Adafruit_SCD30 scd30;
SensirionI2cScd30 scd30;
#define NO_ERROR 0
// -----------------------------
// Preferences / Config
// -----------------------------
Preferences preferences;
Config cfg;
static const char* PREF_NS = "my-app";

// Track which keys exist in NVS (for webpage "source")
bool key_ssid=false, key_pass=false, key_led=false, key_hostname=false, key_altitude=false;
bool key_report=false, key_report_txt=false, key_verbose=false, key_color_mode=false, key_hyst=false;
bool key_auth_pass=false, key_lan_only=false;

// Calibration keys
bool key_scd30_asc=false, key_scd30_frc=false;
bool key_last_cal_epoch=false, key_last_cal_reason=false;
bool key_asc_enabled_epoch=false;

// LED host source
bool ledHostFromNVS = false;

// Boot window / menu coordination
bool enterMenuRequested = false;
bool bootDebugRequested = false;

// -----------------------------
// Async web server
// -----------------------------
AsyncWebServer server(80);

// -----------------------------
// Runtime state
// -----------------------------
float scd30_T_s = NAN;
float scd30_H_s = NAN;
float scd30_CO2_s = NAN;

String led_update_status = "";
String ledResolveState   = "unknown";   // "ok" | "fail" | "unknown"
bool   scd30_config_applied = false;

// RGB computed values
float red = 0, green = 0, blue = 0;
int lastR = -1, lastG = -1, lastB = -1;

// CO2 smoothing/hysteresis state
float co2_ema = NAN;
float co2_for_color = NAN;

// DNS/mDNS cache for LED host
IPAddress led_cached_ip(0, 0, 0, 0);
bool      led_cached_ip_valid = false;

// ---- Web diagnostics / last actions ----
int      lastShellyHttpCode = 0;
String   lastShellyHostUsed = "";
String   lastShellyUrlSent  = "";
uint32_t lastShellyMillis   = 0;
uint32_t lastResolveMillis  = 0;

// CO2 diagnostics (for webpage)
float    lastCO2raw         = NAN;
float    lastCO2ema         = NAN;
float    lastCO2forColor    = NAN;
bool     lastCO2Adjusted    = false;       // true if hysteresis held previous

// -----------------------------
// Scheduling
// -----------------------------
static uint32_t tCO2 = 0;
static uint32_t tOLED = 0;
static uint32_t tShelly = 0;
static uint32_t tWifiRetry = 0;
static uint32_t tResolveRetry = 0;
static uint32_t tNtpPoll = 0;

static constexpr uint32_t CO2_PERIOD_MS    = 2500;
static constexpr uint32_t OLED_PERIOD_MS   = 500;
static constexpr uint32_t SHELLY_PERIOD_MS = 2500;
static constexpr uint32_t WIFI_RETRY_MS    = 10000;

static constexpr uint32_t BOOT_WINDOW_MS = 5000;

// Resolver warmup after WiFi connects
static uint32_t ledResolveNotBefore = 0;
static constexpr uint32_t RESOLVER_WARMUP_MS = 2000;

// Startup resolve bounded time
static constexpr uint32_t STARTUP_RESOLVE_MAX_MS = 15000;
static constexpr uint32_t STARTUP_RESOLVE_RETRY_MS = 500;

// Runtime resolve retry interval if startup failed
static constexpr uint32_t RUNTIME_RESOLVE_RETRY_MS = 10000;

// NTP polling until synced
static constexpr uint32_t NTP_POLL_MS = 2000;

// -----------------------------
// Calibration runtime bookkeeping
// -----------------------------
volatile bool g_requestFrcNow400 = false;
bool frcPendingBanner = false;

// Persisted “last calibration”
time_t lastCalEpoch = 0;
String lastCalReason = "";

// Persisted “ASC enabled since”
time_t ascEnabledEpoch = 0;

// NTP status
bool   ntpConfigured = false;
bool   ntpSynced = false;
time_t ntpLastSyncEpoch = 0;
String ntpServerUsed = "";

// -----------------------------
// Utility
// -----------------------------
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
  return true; // safe default
#endif
}

// Remove scheme/path and keep only host token
String sanitizeHost(String h) {
  h.trim();
  h.replace("http://", "");
  h.replace("https://", "");
  int slash = h.indexOf('/');
  if (slash >= 0) h = h.substring(0, slash);
  h.trim();
  return h;
}

void oledMessage(const String& a, const String& b = "", const String& c = "") {
  if (!display.width()) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE); 
  //display.setTextColor(SSD1306_WHITE); 
  display.setCursor(0, 0);
  display.println(a);
  if (b.length()) display.println(b);
  if (c.length()) display.println(c);
  display.display();
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

// -----------------------------
// Time helpers
// -----------------------------
static bool timeIsValid() {
  time_t now = time(nullptr);
  // 2020-01-01 as “valid enough”
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

// -----------------------------
// Persist calibration event
// -----------------------------
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

// -----------------------------
// Factory reset (wipe NVS)
// -----------------------------
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

// Boot window combos
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

// -----------------------------
// Config load/save
// -----------------------------
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

  // Calibration keys
  key_scd30_asc  = preferences.isKey("scd30_asc");
  key_scd30_frc  = preferences.isKey("scd30_frc");
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

  // ---- Calibration (FIX #1: no duplicates) ----
  cfg.scd30_asc     = preferences.getBool("scd30_asc", true);
  cfg.scd30_frc_ppm = (int16_t)preferences.getInt("scd30_frc", -1);

  // Safety: if FRC pending -> disable ASC
  if (cfg.scd30_frc_ppm >= 0 && cfg.scd30_asc) {
    if (cfg.verbose) Serial.println("Config: FRC pending -> disabling ASC (temporary safety)");
    cfg.scd30_asc = false;
  }

  // Load calibration record
  lastCalEpoch  = (time_t)preferences.getLong("last_cal_epoch", 0);
  lastCalReason = preferences.getString("last_cal_reason", "");

  // ASC enabled epoch
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

  // Calibration
  if (cfg.scd30_asc != oldCfg.scd30_asc) {
    preferences.putBool("scd30_asc", cfg.scd30_asc);
    // track ASC enabled epoch when turning ON
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

  preferences.end();
}

// -----------------------------
// WiFi + optional mDNS
// -----------------------------
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

// -----------------------------
// NTP
// -----------------------------
static void ntpConfigureIfNeeded() {
  if (ntpConfigured) return;
  if (WiFi.status() != WL_CONNECTED) return;

  IPAddress gw = WiFi.gatewayIP();
  String server = (gw[0] || gw[1] || gw[2] || gw[3]) ? gw.toString() : String("pool.ntp.org");

  cfg.ntp_server = server;
  ntpServerUsed = server;

  // CET/CEST could be set, but we keep UTC unless you want TZ rules here.
  // configTime(gmtOffset_sec, daylightOffset_sec, server1, server2)
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

// -----------------------------
// LED host resolution (IP / DNS / mDNS) with fallback
// -----------------------------
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

  // Regular DNS first for non-.local
  if (!host.endsWith(".local")) {
    for (int i = 0; i < 4; i++) {
      if (dnsResolveWithFallback(host, out)) return true;
      delay(150);
      yield();
    }
    return false;
  }

  // mDNS: query without ".local"
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

// -----------------------------
// Shelly RGBW2 Gen1 (GET /color/0?...)
// -----------------------------
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

// -----------------------------
// CO2 -> RGB mappings
// -----------------------------
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

// -----------------------------
// CO2 smoothing + hysteresis
// -----------------------------
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

// -----------------------------
// SCD30 config (apply once)
// -----------------------------
// Sensirion SCD30: apply configuration once
void applySCD30ConfigOnce(float altitude_m) {
  if (scd30_config_applied) return;

  // Sensirion library uses error codes: 0 == success
  int16_t err = 0;
  bool okAll = true;

  // If an FRC is pending, ASC must be OFF (we also persist ASC off when FRC is applied)
  bool ascWanted = cfg.scd30_asc;
  if (cfg.scd30_frc_ppm >= 0) ascWanted = false;

  // 1) Stop measurement before reconfig (recommended)
  err = scd30.stopPeriodicMeasurement();
  // stop may fail if not started yet; don't hard-fail
  if (cfg.verbose && err != NO_ERROR) {
    Serial.print("SCD30: stopPeriodicMeasurement err="); Serial.println(err);
  }

  // 2) Measurement interval (seconds)
  err = scd30.setMeasurementInterval(2);
  okAll &= (err == NO_ERROR);
  if (cfg.verbose && err != NO_ERROR) {
    Serial.print("SCD30: setMeasurementInterval err="); Serial.println(err);
  }

  // 3) Altitude compensation (meters)
  uint16_t alt = (uint16_t)lroundf(altitude_m);
  err = scd30.setAltitudeCompensation(alt);
  okAll &= (err == NO_ERROR);
  if (cfg.verbose && err != NO_ERROR) {
    Serial.print("SCD30: setAltitudeCompensation err="); Serial.println(err);
  }

  // 4) ASC on/off
  err = scd30.activateAutoCalibration(ascWanted ? 1 : 0);
  okAll &= (err == NO_ERROR);
  if (cfg.verbose && err != NO_ERROR) {
    Serial.print("SCD30: activateAutoCalibration err="); Serial.println(err);
  }

  // Reflect what we actually applied
  cfg.scd30_asc = ascWanted;

  // 5) Start periodic measurement (pressure=0 disables pressure compensation)
  err = scd30.startPeriodicMeasurement(0);
  okAll &= (err == NO_ERROR);
  if (cfg.verbose && err != NO_ERROR) {
    Serial.print("SCD30: startPeriodicMeasurement err="); Serial.println(err);
  }

  scd30_config_applied = true;

  // ---- Status dump (verbose) ----
  if (cfg.verbose) {
    Serial.println("SCD30 config/status (Sensirion):");
    Serial.print("  okAll: "); Serial.println(okAll ? "true" : "false");

    uint16_t interval_s = 0;
    err = scd30.getMeasurementInterval(interval_s);
    Serial.print("  interval(s): ");
    if (err == NO_ERROR) Serial.println(interval_s);
    else { Serial.print("ERR "); Serial.println(err); }

    uint16_t alt_m = 0;
    err = scd30.getAltitudeCompensation(alt_m);
    Serial.print("  altitude_comp(m): ");
    if (err == NO_ERROR) Serial.println(alt_m);
    else { Serial.print("ERR "); Serial.println(err); }

    uint16_t asc = 0;
    err = scd30.getAutoCalibrationStatus(asc);
    Serial.print("  ASC enabled: ");
    if (err == NO_ERROR) Serial.println(asc ? "true" : "false");
    else { Serial.print("ERR "); Serial.println(err); }

    uint16_t frc = 0;
    err = scd30.getForceRecalibrationStatus(frc);
    Serial.print("  FRC ref(ppm): ");
    if (err == NO_ERROR) Serial.println(frc);
    else { Serial.print("ERR "); Serial.println(err); }

    uint16_t tOff = 0;
    err = scd30.getTemperatureOffset(tOff); // hundredths of °C, depending on lib
    Serial.print("  temp offset(raw): ");
    if (err == NO_ERROR) Serial.println(tOff);
    else { Serial.print("ERR "); Serial.println(err); }

    Serial.println();
  }
}



// -----------------------------
// Calibration action: Calibrate now (400ppm) [SAFE loop-context]
// -----------------------------
static bool applyFrcNow400() {
  // Require a valid reading first
  if (!isfinite(scd30_CO2_s) || scd30_CO2_s <= 0.0f) {
    if (cfg.verbose) Serial.println("FRC now refused: no valid CO2 reading yet.");
    return false;
  }

  const uint16_t target = 400;

  if (cfg.verbose) Serial.println("FRC now: disabling ASC before applying FRC");

  // Disable ASC
  int16_t err = scd30.activateAutoCalibration(0);
  if (err != NO_ERROR && cfg.verbose) {
    Serial.print("FRC now: activateAutoCalibration(0) err="); Serial.println(err);
  }
  cfg.scd30_asc = false;

  // Apply FRC
  err = scd30.forceRecalibration(target);
  if (err == NO_ERROR) {
    if (cfg.verbose) Serial.println("FRC now applied: 400 ppm");

    recordCalibrationApplied("FRC(now 400)");

    // Clear pending FRC key + persist ASC off
    preferences.begin(PREF_NS, false);
    preferences.remove("scd30_frc");
    preferences.putBool("scd30_asc", false);
    preferences.end();

    cfg.scd30_frc_ppm = -1;
    key_scd30_frc = false;
    key_scd30_asc = true;
    return true;
  }

  if (cfg.verbose) {
    Serial.print("FRC now FAILED, err="); Serial.println(err);
  }
  return false;
}



// Apply pending FRC (from NVS) once we have a stable valid reading

// Apply a pending forced recalibration reference (ppm), once.
void maybeApplyPendingFrcNonBlocking() {
  if (cfg.scd30_frc_ppm < 0) return;  // nothing pending

  // Require at least one valid CO2 reading
  if (isnan(scd30_CO2_s) || scd30_CO2_s <= 0.0f) {
    if (cfg.verbose) Serial.println("Pending FRC: waiting for valid CO2 reading...");
    return;
  }

  const uint16_t target = (uint16_t)cfg.scd30_frc_ppm;

  // Sensirion recommendation: run in periodic measurement with 2s interval
  // for ~2 minutes before applying FRC (you already gate on “valid reading”).
  if (cfg.verbose) {
    Serial.print("Applying pending FRC (Sensirion): ");
    Serial.print(target);
    Serial.println(" ppm");
  }

  // FRC overrides ASC, so disable ASC first (and persist that)
  int16_t err = scd30.activateAutoCalibration(0);
  if (err != NO_ERROR && cfg.verbose) {
    Serial.print("Pending FRC: activateAutoCalibration(0) failed, err=");
    Serial.println(err);
  }
  cfg.scd30_asc = false;

  // Apply FRC (returns error code, 0 = success)
  err = scd30.forceRecalibration(target);

  if (err == NO_ERROR) {
    recordCalibrationApplied(String("FRC(pending ") + target + " ppm)");

    preferences.begin(PREF_NS, false);
    preferences.remove("scd30_frc");
    preferences.putBool("scd30_asc", false); // persist ASC off
    preferences.end();

    cfg.scd30_frc_ppm = -1;
    key_scd30_frc = false;
    key_scd30_asc = true;

    if (cfg.verbose) Serial.println("Pending FRC applied successfully.");
  } else {
    if (cfg.verbose) {
      Serial.print("Pending FRC FAILED, err=");
      Serial.println(err);
    }
  }
}


// -----------------------------
// Shelly update with cached-IP + resolver fallback
// -----------------------------
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

// -----------------------------
// OLED display
// -----------------------------
void displayOLED(float C, float H, float CO2) {
  if (!display.width()) return;

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);

  display.setCursor(0, 0);
  if (!isnan(C)) { display.print(C, 1); display.print(" C"); }
  else display.print("--.- C");

  display.setCursor(55, 0);
  if (!isnan(H)) { display.print(H, 1); display.print("%"); }
  else display.print("--.-%");

  display.setCursor(0, 10); display.setTextSize(1);  display.print("CO2: ");
  display.setCursor(110, 10); display.setTextSize(1);  display.print("ppm");

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

// -----------------------------
// Serial reporting
// -----------------------------
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

// -----------------------------
// BME280 read (altitude)
// -----------------------------
bool readBMEandUpdateAltitude() {
  if (!bme_sensor_found) return false;

  float temp(NAN), hum(NAN), pres(NAN);
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_hPa);

  bme.read(pres, temp, hum, tempUnit, presUnit);
  if (isnan(pres) || isnan(temp) || isnan(hum)) return false;

  EnvironmentCalculations::AltitudeUnit envAltUnit  = EnvironmentCalculations::AltitudeUnit_Meters;
  EnvironmentCalculations::TempUnit     envTempUnit = EnvironmentCalculations::TempUnit_Celsius;

  alt_s = EnvironmentCalculations::Altitude(pres, envAltUnit, referencePressure, outdoorTemp, envTempUnit);
  return true;
}

// -----------------------------
// Admin protection (LAN-only + Basic Auth)
// -----------------------------
static bool requireAdmin(AsyncWebServerRequest* request) {
  // LAN-only guard
  if (cfg.lan_only) {
    IPAddress client = request->client()->remoteIP();
    IPAddress local  = WiFi.localIP();
    bool same24 = (client[0]==local[0] && client[1]==local[1] && client[2]==local[2]);
    if (!same24) {
      request->send(403, "text/plain; charset=utf-8", "Forbidden (LAN-only)\n");
      return false;
    }
  }

  // Basic auth
  const char* user = "admin";
  const char* pass = cfg.auth_pass.c_str();
  if (!request->authenticate(user, pass)) {
    request->requestAuthentication();
    return false;
  }
  return true;
}



// -----------------------------
// Startup Serial Menu
// -----------------------------
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
  //oledMessage("CO2 Sensor");

  //oledMessage(String("Version: ") + MY_VERSION);
  //oledMessage("Press key for menu");
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
    if (choice == 0) return; // timeout
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
        cfg.led_host = sanitizeHost(v); // replace, do not append
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

// -----------------------------
// Web helpers
// -----------------------------
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

static String cssRgbTriplet(int r, int g, int b)
{
  // Clamp just in case
  if (r < 0)
    r = 0;
  if (r > 255)
    r = 255;
  if (g < 0)
    g = 0;
  if (g > 255)
    g = 255;
  if (b < 0)
    b = 0;
  if (b > 255)
    b = 255;
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

// -----------------------------
// Config Editor HTML
// -----------------------------
static String buildConfigEditorHtml() {
  String h; h.reserve(9000);

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

  // Instructions block (your “Changes 1–9” style guidance, embedded)
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
  h += htmlSelectRow("Color mode", "color_mode", "AQI", "LEGACY",
                     (cfg.color_mode == ColorMode::AQI), src(key_color_mode));
  h += htmlSelectRow("CO2 hysteresis", "hyst", "ON", "OFF",
                     cfg.co2_hyst_enabled, src(key_hyst));
  h += htmlSelectRow("Verbose debug", "verbose", "ON", "OFF",
                     cfg.verbose, src(key_verbose));
  h += htmlSelectRow("LAN-only guard", "lan_only", "ON", "OFF",
                     cfg.lan_only, src(key_lan_only));

  h += F("<h3>Calibration</h3>");
  // ASC (may show forced OFF if FRC pending)
  h += htmlSelectRow("SCD30 ASC", "scd30_asc", "ON", "OFF",
                     cfg.scd30_asc, key_scd30_asc ? "NVS" : "Default");

  // FRC pending input
  h += htmlInputRow("SCD30 FRC pending (ppm)", "scd30_frc",
                    (cfg.scd30_frc_ppm >= 0 ? String(cfg.scd30_frc_ppm) : ""),
                    key_scd30_frc ? "NVS" : "None",
                    false, "blank clears; range 350..2000");

  h += F("<div style='margin-top:12px;'>");
  h += F("<button class='btn btn-save' type='submit'>Save to NVS and Reboot</button>");
  h += F("</div>");
  h += F("</form>");

  // Calibrate now button
  h += F("<div class='box'>");
  h += F("<h3>Calibrate Now</h3>");
  h += F("<form method='POST' action='/calibrate400'>");
  h += F("<button class='btn btn-cal' type='submit'>Calibrate now (400 ppm)</button>");
  h += F("</form>");
  h += F("</div>");

  // Maintenance
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

// -----------------------------
// Status Page HTML
// -----------------------------
// -----------------------------
// Small helper: shared page chrome
// -----------------------------
static void htmlPageBegin(String &h, const char *title)
{
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

  // top menu buttons
  h += F(".topnav{display:flex;gap:10px;flex-wrap:wrap;margin:12px 0;} ");
  h += F(".btn{display:inline-block;padding:10px 14px;border:1px solid #bbb;border-radius:10px;text-decoration:none;color:#111;background:#f6f6f6;} ");
  h += F(".btn:hover{background:#eee;} ");
  h += F(".btn-primary{border-color:#6aa0ff;background:#eaf1ff;} ");

  h += F(".back{margin-top:12px;} ");
  h += F("</style></head><body>");
}

static void htmlPageEnd(String &h)
{
  h += F("</body></html>");
}

// -----------------------------
// Home / Menu page (was old buildStatusPageHtml)
// -----------------------------
String buildStatusPageHtml()
{
  String h;
  h.reserve(4500);
  htmlPageBegin(h, "CO2 Sensor");

  h += F("<h2>CO2 Sensor v");
  h += MY_VERSION;
  h += F("</h2>");

  // Top-level menu
  h += F("<div class='topnav'>");
  h += F("<a class='btn btn-primary' href='/sensor'>Sensor</a>");
  h += F("<a class='btn' href='/led'>LED Shelly</a>");
  h += F("<a class='btn' href='/system'>System Config</a>");
  h += F("</div>");

  // Keep the quick “ESP32 snapshot” here so / still feels useful
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

  // Optional: quick “where to go” hint
  h += F("<div class='box'>Use the buttons above to view Sensor data, LED/Shelly diagnostics, or System configuration.</div>");

  htmlPageEnd(h);
  return h;
}

// -----------------------------
// Sensor page: SCD30 + Calibration moved here
// -----------------------------
String buildSensorPageHtml()
{
  String h;
  h.reserve(9000);
  htmlPageBegin(h, "Sensor");

  // Top nav
  h += F("<div class='topnav'>");
  h += F("<a class='btn btn-primary' href='/sensor'>Sensor</a>");
  h += F("<a class='btn' href='/led'>LED Shelly</a>");
  h += F("<a class='btn' href='/system'>System Config</a>");
  h += F("</div>");

  h += F("<h2>Sensor</h2>");

  // Warning banner (same logic as before)
  bool frcPending = (cfg.scd30_frc_ppm >= 0);
  if (frcPending)
  {
    h += F("<div class='banner warn'><b>Warning:</b> FRC pending (");
    h += String(cfg.scd30_frc_ppm);
    h += F(" ppm). ASC is forced OFF until calibration is applied.</div>");
  }

  // --- SCD30 Sensor Readings (moved from old status page) ---
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

  // --- Calibration (moved from old status page) ---
  // ASC uptime
  String ascUptime = "n/a";
  if (cfg.scd30_asc && ascEnabledEpoch > 0 && timeIsValid())
  {
    time_t now = time(nullptr);
    long sec = (long)(now - ascEnabledEpoch);
    if (sec < 0)
      sec = 0;
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

// -----------------------------
// LED Shelly page: LED/Shelly section moved here
// -----------------------------
String buildLedShellyPageHtml()
{
  String h;
  h.reserve(9000);
  htmlPageBegin(h, "LED Shelly");

  h += F("<div class='topnav'>");
  h += F("<a class='btn' href='/sensor'>Sensor</a>");
  h += F("<a class='btn btn-primary' href='/led'>LED Shelly</a>");
  h += F("<a class='btn' href='/system'>System Config</a>");
  h += F("</div>");

  h += F("<h2>LED / Shelly</h2>");

  // LED / Shelly (moved from old status page)
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

  // Keep the Color box here (since it’s tightly coupled to LED behavior)
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

  if (lastR >= 0 && lastG >= 0 && lastB >= 0)
  {
    String css = cssRgbTriplet(lastR, lastG, lastB);

    h += F("<div style='margin-top:10px; display:flex; align-items:center; gap:10px;'>");
    h += F("<div style='width:44px; height:22px; border:1px solid #888; border-radius:6px; background:");
    h += htmlEscape(css);
    h += F("'></div>");
    h += F("<div><span class='k'>Result:</span> <code>");
    h += htmlEscape(css);
    h += F("</code></div>");
    h += F("</div>");
  }
  else
  {
    h += F("<div style='margin-top:10px; color:#777;'>No RGB value yet.</div>");
  }

  h += F("</div>");

  h += F("<div class='box back'><a href='/'>Back to Menu</a></div>");

  htmlPageEnd(h);
  return h;
}

// -----------------------------
// System Config page: Open Config Editor link + NVS/Config moved here
// -----------------------------
String buildSystemConfigPageHtml()
{
  String h;
  h.reserve(10000);
  htmlPageBegin(h, "System Config");

  h += F("<div class='topnav'>");
  h += F("<a class='btn' href='/sensor'>Sensor</a>");
  h += F("<a class='btn' href='/led'>LED Shelly</a>");
  h += F("<a class='btn btn-primary' href='/system'>System Config</a>");
  h += F("</div>");

  h += F("<h2>System Config</h2>");

  // Move “Open Config Editor” here (top of this page)
  h += F("<div class='box'><a href='/config'>Open Config Editor</a></div>");

  // NTP status (keeping here makes sense for "System")
  h += F("<div class='box'><b>Network Time Protocol</b><br/>");
  h += F("<span class='k'>Server:</span> ");
  h += htmlEscape(ntpServerUsed);
  h += F("<br/>");
  h += F("<span class='k'>Configured:</span> ");
  h += (ntpConfigured ? "YES" : "NO");
  h += F("<br/>");
  h += F("<span class='k'>Synced:</span> ");
  h += (ntpSynced ? "YES" : "NO");
  h += F("<br/>");
  h += F("<span class='k'>Epoch now:</span> ");
  h += String((long)time(nullptr));
  h += F("<br/>");
  h += F("<span class='k'>Last sync:</span> ");
  h += htmlEscape(fmtTime(ntpLastSyncEpoch));
  h += F("<br/>");
  h += F("</div>");

  // NVS / Config (moved from old status page)
  h += F("<div class='box'><b>NVS / Config</b><br/>");
  h += F("<span class='k'>ssid:</span> ");
  h += htmlEscape(cfg.ssid);
  h += F(" (src: ");
  h += src(key_ssid);
  h += F(")<br/>");
  h += F("<span class='k'>ssid_pwd:</span> ******** (src: ");
  h += src(key_pass);
  h += F(")<br/>");
  h += F("<span class='k'>led:</span> ");
  h += htmlEscape(cfg.led_host);
  h += F(" (src: ");
  h += src(key_led);
  h += F(")<br/>");
  h += F("<span class='k'>hostname:</span> ");
  h += htmlEscape(cfg.hostname);
  h += F(" (src: ");
  h += src(key_hostname);
  h += F(")<br/>");
  h += F("<span class='k'>altitude:</span> ");
  h += String(cfg.altitude_m);
  h += F(" (src: ");
  h += src(key_altitude);
  h += F(")<br/>");
  h += F("<span class='k'>verbose:</span> ");
  h += (cfg.verbose ? "true" : "false");
  h += F(" (src: ");
  h += src(key_verbose);
  h += F(")<br/>");
  h += F("<span class='k'>color_mode:</span> ");
  h += htmlEscape(colorModeToStr(cfg.color_mode));
  h += F(" (src: ");
  h += src(key_color_mode);
  h += F(")<br/>");
  h += F("<span class='k'>hyst:</span> ");
  h += (cfg.co2_hyst_enabled ? "ON" : "OFF");
  h += F(" (src: ");
  h += src(key_hyst);
  h += F(")<br/>");
  h += F("<span class='k'>lan_only:</span> ");
  h += (cfg.lan_only ? "ON" : "OFF");
  h += F(" (src: ");
  h += src(key_lan_only);
  h += F(")<br/>");
  h += F("<span class='k'>auth_pass:</span> ******** (src: ");
  h += (key_auth_pass ? "NVS" : "WiFi fallback");
  h += F(")<br/>");
  h += F("<span class='k'>scd30_asc:</span> ");
  h += (cfg.scd30_asc ? "ON" : "OFF");
  h += F(" (src: ");
  h += (key_scd30_asc ? "NVS" : "Default");
  h += F(")<br/>");
  h += F("<span class='k'>scd30_frc:</span> ");
  h += (cfg.scd30_frc_ppm >= 0 ? String(cfg.scd30_frc_ppm) : "none");
  h += F(" (src: ");
  h += (key_scd30_frc ? "NVS" : "None");
  h += F(")<br/>");
  h += F("</div>");

  h += F("<div class='box back'><a href='/'>Back to Menu</a></div>");

  htmlPageEnd(h);
  return h;
}


String buildStatusPageHtmlold() {
  String h;
  h.reserve(9000);

  const String ssid = WiFi.SSID();
  const String ip   = WiFi.localIP().toString();
  const long rssi   = WiFi.RSSI();

  bool frcPending = (cfg.scd30_frc_ppm >= 0);

  // ASC uptime
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

  h += F("<!DOCTYPE html><html><head><meta charset='utf-8'>");
  h += F("<meta name='viewport' content='width=device-width, initial-scale=1'>");
  h += F("<title>CO2 Sensor</title>");
  h += F("<style>");
  h += F("body{font-family:Arial;margin:14px;} .box{padding:10px;border:1px solid #ccc;border-radius:8px;margin:10px 0;} .k{color:#555;} code{word-break:break-all;}");
  h += F(".banner{padding:10px;border-radius:10px;margin:10px 0;} .warn{background:#fff2cc;border:1px solid #e6c200;}");
  h += F("</style></head><body>");

  h += F("<h2>CO2 Sensor v");
  h += MY_VERSION;
  h += F("</h2>");

  if (frcPending) {
    h += F("<div class='banner warn'><b>Warning:</b> FRC pending (");
    h += String(cfg.scd30_frc_ppm);
    h += F(" ppm). ASC is forced OFF until calibration is applied.</div>");
  }

  h += F("<div class='box'><a href='/config'>Open Config Editor</a></div>");

  // ESP status
  h += F("<div class='box'><b>ESP32 Wireless Connection</b><br/>");
  h += F("<span class='k'>SSID:</span> "); h += htmlEscape(ssid); h += F("<br/>");
  h += F("<span class='k'>IP:</span> ");   h += htmlEscape(ip);   h += F("<br/>");
  h += F("<span class='k'>RSSI:</span> "); h += String(rssi);     h += F(" dBm<br/>");
#if ENABLE_MDNS
  h += F("<span class='k'>ESP mDNS:</span> <code>http://");
  h += htmlEscape(cfg.hostname);
  h += F(".local/</code><br/>");
#endif
  h += F("</div>");

  // NTP status
  h += F("<div class='box'><b>Network Time Protocol</b><br/>");
  h += F("<span class='k'>Server:</span> "); h += htmlEscape(ntpServerUsed); h += F("<br/>");
  h += F("<span class='k'>Configured:</span> "); h += (ntpConfigured ? "YES" : "NO"); h += F("<br/>");
  h += F("<span class='k'>Synced:</span> "); h += (ntpSynced ? "YES" : "NO"); h += F("<br/>");
  h += F("<span class='k'>Epoch now:</span> "); h += String((long)time(nullptr)); h += F("<br/>");
  h += F("<span class='k'>Last sync:</span> "); h += htmlEscape(fmtTime(ntpLastSyncEpoch)); h += F("<br/>");
  h += F("</div>");

  // Sensor readings
  h += F("<div class='box'><b>SCD30 Sensor Readings</b><br/>");
  h += F("<span class='k'>Temperature:</span> "); h += String(scd30_T_s, 1); h += F(" &deg;C<br/>");
  h += F("<span class='k'>Humidity:</span> ");    h += String(scd30_H_s, 1); h += F(" %<br/>");
  h += F("<span class='k'>CO2 raw:</span> ");     h += String(lastCO2raw, 0); h += F(" ppm<br/>");
  h += F("<span class='k'>CO2 EMA:</span> ");     h += String(lastCO2ema, 1); h += F(" ppm<br/>");
  h += F("<span class='k'>CO2 used-for-color:</span> "); h += String(lastCO2forColor, 1); h += F(" ppm<br/>");
  h += F("<span class='k'>Hysteresis holding?:</span> "); h += (lastCO2Adjusted ? "YES" : "NO"); h += F("<br/>");
  h += F("</div>");

 

  // LED / Shelly
  h += F("<div class='box'><b>LED / Shelly</b><br/>");
  h += F("<span class='k'>Configured LED host:</span> "); h += htmlEscape(cfg.led_host); h += F("<br/>");
  h += F("<span class='k'>Host source:</span> "); h += (ledHostFromNVS ? "NVS override" : "Secrets default"); h += F("<br/>");
  h += F("<span class='k'>Resolve state:</span> "); h += htmlEscape(ledResolveState); h += F("<br/>");
  h += F("<span class='k'>Cached IP:</span> ");
  h += (led_cached_ip_valid ? htmlEscape(led_cached_ip.toString()) : "none");
  h += F("<br/>");
  h += F("<span class='k'>Last Shelly status:</span> "); h += htmlEscape(led_update_status); h += F("<br/>");
  h += F("<span class='k'>Last HTTP code:</span> "); h += String(lastShellyHttpCode); h += F("<br/>");
  h += F("<span class='k'>Last host used:</span> "); h += htmlEscape(lastShellyHostUsed); h += F("<br/>");
  h += F("<span class='k'>Last URL sent:</span> <code>"); h += htmlEscape(lastShellyUrlSent); h += F("</code><br/>");
  h += F("</div>");

  // Config dump (masked)
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
  h += F("</div>");

  // Color
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

  // Swatch (only show if we have a valid RGB yet)
  if (lastR >= 0 && lastG >= 0 && lastB >= 0)
  {
    String css = cssRgbTriplet(lastR, lastG, lastB);

    h += F("<div style='margin-top:10px; display:flex; align-items:center; gap:10px;'>");
    h += F("<div style='width:44px; height:22px; border:1px solid #888; border-radius:6px; ");
    h += F("background:");
    h += htmlEscape(css);
    h += F("'></div>");

    h += F("<div><span class='k'>Result:</span> <code>");
    h += htmlEscape(css);
    h += F("</code></div>");
    h += F("</div>");
  }
  else
  {
    h += F("<div style='margin-top:10px; color:#777;'>No RGB value yet.</div>");
  }

  h += F("</div>");


  // Calibration status
  h += F("<div class='box'><b>Sensor Calibration</b><br/>");
  h += F("<span class='k'>ASC enabled:</span> "); h += (cfg.scd30_asc ? "YES" : "NO"); h += F("<br/>");
  h += F("<span class='k'>ASC uptime:</span> "); h += htmlEscape(ascUptime); h += F("<br/>");
  h += F("<span class='k'>FRC pending:</span> "); h += (cfg.scd30_frc_ppm >= 0 ? String(cfg.scd30_frc_ppm) : "none"); h += F("<br/>");
  h += F("<span class='k'>Last calibration:</span> "); h += htmlEscape(fmtTime(lastCalEpoch)); h += F("<br/>");
  h += F("<span class='k'>Reason:</span> "); h += htmlEscape(lastCalReason); h += F("<br/>");
  h += F("<form method='POST' action='/calibrate400' style='margin-top:8px;'>");
  h += F("<button type='submit'>Calibrate now (400 ppm)</button>");
  h += F("</form>");
  h += F("</div>");


  h += F("</body></html>");
  return h;
}

// -----------------------------
// Setup AsyncWebServer
// -----------------------------
// -----------------------------
// Setup AsyncWebServer
// -----------------------------
void setupAsyncWebServer()
{
  // Home menu
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/html; charset=utf-8", buildStatusPageHtml()); });

  // New pages
  server.on("/sensor", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/html; charset=utf-8", buildSensorPageHtml()); });

  server.on("/led", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/html; charset=utf-8", buildLedShellyPageHtml()); });

  // System page is admin-protected (recommended because it includes NVS dump + config editor link)
  server.on("/system", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    if (!requireAdmin(request)) return;
    request->send(200, "text/html; charset=utf-8", buildSystemConfigPageHtml()); });

  server.on("/health", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain; charset=utf-8", "OK\n"); });

  // Existing config editor (keep as-is, already protected)
  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    if (!requireAdmin(request)) return;
    request->send(200, "text/html; charset=utf-8", buildConfigEditorHtml()); });

  server.on("/config", HTTP_POST, [](AsyncWebServerRequest *request)
            {
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

    // Calibration inputs
    String asc      = getArg("scd30_asc"); asc.trim();     // "ON"/"OFF"
    String frc      = getArg("scd30_frc"); frc.trim();     // ppm, blank clears

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

    // ASC
    if (asc.length()) {
      asc.toUpperCase();
      cfg.scd30_asc = (asc == "ON");
    }

    // FRC pending (blank clears)
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

    // Safety: if FRC pending -> force ASC off
    if (cfg.scd30_frc_ppm >= 0 && cfg.scd30_asc) {
      cfg.scd30_asc = false;
    }

    saveConfigIfChanged(oldCfg);

    request->send(200, "text/html; charset=utf-8",
                  "<html><body><h3>Saved to NVS. Rebooting...</h3>"
                  "<p><a href='/'>Menu</a></p></body></html>");

    delay(300);
    ESP.restart(); });

  // Calibrate now (400 ppm) - safe: request handled in loop
  server.on("/calibrate400", HTTP_POST, [](AsyncWebServerRequest *request)
            {
    if (!requireAdmin(request)) return;

    g_requestFrcNow400 = true;

    request->send(200, "text/html; charset=utf-8",
      "<html><body><h3>Calibration requested: FRC 400 ppm</h3>"
      "<p>The device will apply this shortly in the main loop (safe context).</p>"
      "<p>Ensure the sensor is in fresh outdoor air (~400 ppm). Wait 1-2 minutes for stability.</p>"
      "<p><a href='/sensor'>Back to Sensor</a> | <a href='/system'>System Config</a></p>"
      "</body></html>"); });

  server.on("/reboot", HTTP_POST, [](AsyncWebServerRequest *request)
            {
    if (!requireAdmin(request)) return;
    request->send(200, "text/plain; charset=utf-8", "Rebooting...\n");
    delay(200);
    ESP.restart(); });

  server.on("/factoryreset", HTTP_POST, [](AsyncWebServerRequest *request)
            {
    if (!requireAdmin(request)) return;
    request->send(200, "text/plain; charset=utf-8", "Wiping NVS and rebooting...\n");
    delay(200);
    wipePreferencesNVS(); });

  server.begin();
}


// -----------------------------
// Startup OLED config screen
// -----------------------------
void showStartupConfigOLED(uint32_t ms = 5000) {
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
  display.setCursor(0, 9);   display.print("SSID("); display.print(srcSsid); display.print("):"); display.println(ssidStr);
  display.setCursor(0, 17);  display.print("LED("); display.print(srcLed); display.print("):"); display.println(ledHostStr);
  display.setCursor(0, 25);  display.print("LED IP:"); display.println(ledIpStr);

  display.setCursor(0, 34);
  display.print("ASC:"); display.print(cfg.scd30_asc ? "ON" : "OFF");
  display.print(" FRC:");
  display.print(cfg.scd30_frc_ppm >= 0 ? String(cfg.scd30_frc_ppm) : String("-"));

  display.setCursor(0, 43);
  display.print("DBG:"); display.print(cfg.verbose ? "ON" : "OFF");
  display.print(" LAN:"); display.print(cfg.lan_only ? "ON" : "OFF");

  display.setCursor(0, 52);
  display.print("NTP:"); display.print(ntpConfigured ? "cfg" : "-");
  display.print(" ");
  display.print(ntpSynced ? "sync" : "-");

  display.display();
  delay(ms);
}

// -----------------------------
// Setup
// -----------------------------
void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(21, 22);

  // OLED init
  pinMode(OLED_RESET_PIN, OUTPUT);
  digitalWrite(OLED_RESET_PIN, HIGH);
  delay(10);
  digitalWrite(OLED_RESET_PIN, LOW);
  delay(10);
  digitalWrite(OLED_RESET_PIN, HIGH);

  //if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
  if (!display.begin(SCREEN_ADDRESS, true)){  
    // Address 0x3D default
    Serial.print("OLED Display not found at" );
    Serial.print( SCREEN_ADDRESS);
    Serial.println(". Continuing without display.");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    //display.setrotation(1);
    display.setRotation(1);
    display.display();

  }

  Serial.println();
  Serial.println("CO2 Ventilation Sensor");
  Serial.print("Version: ");
  Serial.println(MY_VERSION);

  // Boot/menu
  bootWindow();
  loadConfig();
  runMenuWindow(10000);

  // Show config before doing anything else
  showStartupConfigOLED(2000);

  // Start sensors early
  oledMessage("Init sensors...", "SCD30 + BME280");

  const uint32_t t0 = millis();

  // ---- SCD30 init (SensirionI2cScd30) ----
  scd30.begin(Wire, SCD30_I2C_ADDR_61);

  // Verify sensor presence by reading firmware version (err=0 => OK)
  uint8_t fwMajor = 0, fwMinor = 0;
  int16_t err = scd30.readFirmwareVersion(fwMajor, fwMinor);
  if (err != NO_ERROR) {
    Serial.print("FATAL: SCD30 not responding. readFirmwareVersion err=");
    Serial.println(err);
    oledMessage("FATAL", "SCD30 not found", "I2C error");
    while (true) delay(50);
  }

  Serial.print("SCD30 Found! FW=");
  Serial.print(fwMajor);
  Serial.print(".");
  Serial.println(fwMinor);

  if (cfg.verbose) {
    Serial.print("SCD30 init+verify at ms=");
    Serial.println(millis() - t0);
  }

  // Apply SCD30 config (Sensirion lib APIs)
  applySCD30ConfigOnce((float)cfg.altitude_m);

  // BME280 optional
  if (!bme.begin()) {
    Serial.println("BME280 not found. Altitude from config only.");
    bme_sensor_found = false;
  } else {
    Serial.println("BME280 Found!");
    bme_sensor_found = true;
    readBMEandUpdateAltitude();
  }

  // WiFi after sensors
  oledMessage("WiFi...", cfg.ssid);
  wifiConnectBlocking(15000);

#if ENABLE_MDNS
  mdnsEnsureRunning();
#endif

  // Configure NTP (GW as default server)
  ntpConfigureIfNeeded();

  // Resolve LED host
  resolveLedAtStartupBlocking();

  // Web server
  setupAsyncWebServer();

  // If FRC pending, show banner state
  frcPendingBanner = (cfg.scd30_frc_ppm >= 0);

  // Show final startup config
  showStartupConfigOLED(2500);

  Serial.println("Setup complete.");
  oledMessage("Setup complete", "Running...");
}


// -----------------------------
// Loop
// -----------------------------
void loop() {
  const uint32_t now = millis();

  wifiEnsureConnectedNonBlocking();

#if ENABLE_MDNS
  mdnsEnsureRunning();
#endif

  // NTP
  ntpConfigureIfNeeded();
  ntpPollSyncNonBlocking();

  // Handle requested FRC from web button (safe loop context)
  if (g_requestFrcNow400) {
    g_requestFrcNow400 = false;
    bool ok = applyFrcNow400();
    if (cfg.verbose) Serial.printf("FRC request handled -> %s\n", ok ? "OK" : "FAIL");
  }

// Read SCD30 (Sensirion arduino-i2c-scd30)
if (now - tCO2 >= CO2_PERIOD_MS) {
  tCO2 = now;

  uint16_t ready = 0;
  int16_t err = scd30.getDataReady(ready);

  if (err != 0) {
    if (cfg.verbose) {
      Serial.print("SCD30: getDataReady err="); Serial.println(err);
    }
  } else if (ready == 0) {
    if (cfg.verbose) {
      Serial.println("SCD30: dataReady = false");
    }
  } else {
    float co2 = NAN, temp = NAN, rh = NAN;
    err = scd30.readMeasurementData(co2, temp, rh);

    if (err != 0) {
      Serial.println("Error reading SCD30");
      if (cfg.verbose) {
        Serial.print("SCD30: readMeasurementData err="); Serial.println(err);
      }
    } else {
      // record raw sample for diagnostics
      lastCO2raw = co2;

      // Ignore invalid 0 ppm samples
      if (!isfinite(co2) || co2 <= 0.0f) {
        if (cfg.verbose) Serial.println("SCD30 CO2 invalid (<=0 or NaN) -> sample ignored");
      } else {
        scd30_T_s   = temp;
        scd30_H_s   = rh;
        scd30_CO2_s = co2;

        // If an FRC is pending in NVS, try apply once we have valid reading
        if (cfg.scd30_frc_ppm >= 0) {
          maybeApplyPendingFrcNonBlocking();
        }

        printSensorData();

        float used = co2FilterAndHysteresis(scd30_CO2_s);
        lastCO2ema      = co2_ema;
        lastCO2forColor = used;
        // lastCO2Adjusted updated inside co2FilterAndHysteresis()
      }
    }
  }
}


  // OLED update
  if (now - tOLED >= OLED_PERIOD_MS) {
    tOLED = now;
    displayOLED(scd30_T_s, scd30_H_s, scd30_CO2_s);
  }

  // Shelly update
  setLightsFromCO2(scd30_CO2_s);
}
