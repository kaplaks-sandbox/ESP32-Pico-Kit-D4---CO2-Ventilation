// ESP32-PICO-D4 CO2 Ventilation Sensor
// Version 7.2.1
//
// Key improvements:
// - Arduino auto-prototype fix (Config/ColorMode before any functions)
// - Boot window combos:
//     "!!" -> wipe NVS (Preferences)
//     "RESET" + Enter -> wipe NVS
//     "++" -> enable verbose/debug this boot (temporary)
//     any key -> enter setup menu
// - Verbose/debug enabled by:
//     - DEBUG in arduino_secrets.h (default)
//     - NVS saved "verbose"
//     - boot "++" (temporary)
// - Robust LED host resolution:
//     - Supports IP, FQDN, or *.local (mDNS)
//     - Startup verifies resolution (as-is, then short hostname fallback)
//     - Cached IP fallback + runtime retry
// - Shelly RGBW2 Gen1 control via HTTP GET /color/0?... (port 80)
// - Verbose prints full URL and Shelly response body
// - SCD30: explicitly starts continuous measurement to avoid CO2=0 issue
// - SCD30 status/calibration dump at boot if verbose enabled
// - Ignores invalid CO2 samples (0 ppm) and retains last valid
// - CO2 smoothing + hysteresis to reduce flicker
// - Menu: reliable single-key choice; text entry replaces old value
// - Menu: toggle color mapping (AQI vs legacy wavelength)
//
// TO DO:
// - AsyncWebServer
// - OTA updates
// - MQTT instead of HTTP

#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <math.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <Adafruit_SCD30.h>
#include <BME280I2C.h>
#include <EnvironmentCalculations.h>

#include "arduino_secrets.h"  // MUST provide: SECRET_SSID, SECRET_PASS, SECRET_LED_IP, SECRET_HOSTNAME,
                              // SECRET_BAROMETERAltitude_m, SECRET_REPORT, SECRET_REPORT_TXT
                              // Optional: DEBUG

// -------- Optional mDNS --------
#ifndef ENABLE_MDNS
#define ENABLE_MDNS 1
#endif

#if ENABLE_MDNS
  #include <ESPmDNS.h>
#endif

// -----------------------------
// Version
// -----------------------------
static const char* MY_VERSION = "7.2.1";

// -----------------------------
// CO2 anti-flicker tuning
// -----------------------------
static constexpr float CO2_EMA_ALPHA = 0.20f; // 0..1, higher reacts faster
static constexpr float CO2_HYST_PPM  = 25.0f; // deadband around last-used CO2-for-color

// -----------------------------
// IMPORTANT: types MUST appear before any functions (Arduino auto-prototype fix)
// -----------------------------
enum class ColorMode : uint8_t {
  AQI = 0,
  LEGACY_WAVELENGTH = 1
};

struct Config {
  String ssid;
  String pass;
  String led_host;     // IP, FQDN, or *.local. Also tolerates http://... and paths (sanitized).
  String hostname;     // ESP32 hostname (mDNS name too)
  int    altitude_m;
  String report;       // A/T/H/C
  String report_txt;   // Y/N
  bool   verbose;
  ColorMode color_mode;
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
#define SCREEN_ADDRESS 0x3D
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET_PIN);

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
Adafruit_SCD30 scd30;

// -----------------------------
// Preferences / Config
// -----------------------------
Preferences preferences;
Config cfg;
static const char* PREF_NS = "my-app";

// Source flags (startup log)
bool ledHostFromNVS = false;

// Boot window / menu coordination
bool enterMenuRequested = false;
bool bootDebugRequested = false;

// -----------------------------
// Web server
// -----------------------------
WiFiServer server(80);
// ---- Web diagnostics / last actions ----
int      lastShellyHttpCode = 0;
String   lastShellyHostUsed = "";
String   lastShellyUrlSent  = "";
uint32_t lastShellyMillis   = 0;

String   ledResolveState    = "unknown";   // "ok" | "fail" | "unknown"
uint32_t lastResolveMillis  = 0;

// CO2 diagnostics
float    lastCO2raw         = NAN;
float    lastCO2ema         = NAN;
float    lastCO2forColor    = NAN;
bool     lastCO2Adjusted    = false;       // true if hysteresis prevented immediate change




// -----------------------------
// Runtime state
// -----------------------------
float scd30_T_s = NAN;
float scd30_H_s = NAN;
float scd30_CO2_s = NAN;

String led_update_status = "";
bool   scd30_config_applied = false;

// RGB computed values
float red = 0, green = 0, blue = 0;

// Last sent RGB to Shelly (avoid spamming)
int lastR = -1, lastG = -1, lastB = -1;

// CO2 smoothing/hysteresis state
float co2_ema = NAN;
float co2_for_color = NAN;

// DNS/mDNS cache for LED host
IPAddress led_cached_ip(0, 0, 0, 0);
bool      led_cached_ip_valid = false;

// -----------------------------
// Scheduling
// -----------------------------
static uint32_t tCO2 = 0;
static uint32_t tOLED = 0;
static uint32_t tShelly = 0;
static uint32_t tWifiRetry = 0;
static uint32_t tResolveRetry = 0;

static constexpr uint32_t CO2_PERIOD_MS    = 2000;
static constexpr uint32_t OLED_PERIOD_MS   = 500;
static constexpr uint32_t SHELLY_PERIOD_MS = 5000;
static constexpr uint32_t WIFI_RETRY_MS    = 10000;

// Boot window
static constexpr uint32_t BOOT_WINDOW_MS = 5000;

// Resolver warmup after WiFi connects
static uint32_t ledResolveNotBefore = 0;
static constexpr uint32_t RESOLVER_WARMUP_MS = 2000;

// Startup resolve bounded time
static constexpr uint32_t STARTUP_RESOLVE_MAX_MS = 15000;
static constexpr uint32_t STARTUP_RESOLVE_RETRY_MS = 500;

// Runtime resolve retry interval if startup failed
static constexpr uint32_t RUNTIME_RESOLVE_RETRY_MS = 10000;

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
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(a);
  if (b.length()) display.println(b);
  if (c.length()) display.println(c);
  display.display();
}

void flushSerialInput() {
  while (Serial.available() > 0) (void)Serial.read();
}

// Reads a single non-newline char (menu choice). No Enter needed.
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
  return 0; // timeout
}

// Reads a full line (Enter required)
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

// Boot window combos:
// - "!!"            -> wipe NVS
// - "RESET"+Enter   -> wipe NVS
// - "++"            -> enable debug this boot (temporary)
// Any key -> request entering menu next
void bootWindow() {
  Serial.println();
  Serial.println("Boot window: 5 seconds");
  Serial.println("  !!            -> wipe NVS");
  Serial.println("  RESET + Enter -> wipe NVS");
  Serial.println("  ++            -> enable DEBUG for this boot");
  Serial.println("  any key       -> enter setup menu next");
  oledMessage("Boot window", "!!=wipe, ++=debug", "Any key=menu");

  uint32_t start = millis();
  String buf;

  while ((millis() - start) < BOOT_WINDOW_MS) {
    while (Serial.available() > 0) {
      char c = (char)Serial.read();
      if (c == '\r') continue;

      // Any key means "go to menu after window"
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

  // Prevent boot-window keypress from becoming a menu selection
  flushSerialInput();
}

// -----------------------------
// Config load/save (with source logging)
// -----------------------------
static ColorMode parseColorMode(const String& s) {
  if (strEqNoCase(s, "LEGACY")) return ColorMode::LEGACY_WAVELENGTH;
  if (strEqNoCase(s, "WAVELENGTH")) return ColorMode::LEGACY_WAVELENGTH;
  if (strEqNoCase(s, "OLD")) return ColorMode::LEGACY_WAVELENGTH;
  return ColorMode::AQI;
}

void loadConfig() {
  preferences.begin(PREF_NS, true);

  ledHostFromNVS = preferences.isKey("led");
  const bool verboseFromNVS = preferences.isKey("verbose");
  const bool colorFromNVS = preferences.isKey("color_mode");

  cfg.ssid       = preferences.getString("ssid", SECRET_SSID);
  cfg.pass       = preferences.getString("ssid_pwd", SECRET_PASS);
  cfg.led_host   = preferences.getString("led", SECRET_LED_IP);     // uses SECRET_LED_IP default
  cfg.hostname   = preferences.getString("hostname", SECRET_HOSTNAME);
  cfg.altitude_m = preferences.getInt("altitude", SECRET_BAROMETERAltitude_m);
  cfg.report     = preferences.getString("report", SECRET_REPORT);
  cfg.report_txt = preferences.getString("report_txt", SECRET_REPORT_TXT);

  cfg.verbose    = preferences.getBool("verbose", debugFromSecretsDefault());
  String cm      = preferences.getString("color_mode", "AQI");
  cfg.color_mode = parseColorMode(cm);

  preferences.end();

  if (bootDebugRequested) cfg.verbose = true;

  cfg.led_host = sanitizeHost(cfg.led_host);
  cfg.hostname.trim();

  Serial.print("LED host in use: ");
  Serial.println(cfg.led_host);
  Serial.print("LED host source: ");
  Serial.println(ledHostFromNVS ? "NVS (Preferences override)" : "SECRETS (SECRET_LED_IP default)");

  Serial.print("Verbose source : ");
  if (bootDebugRequested) Serial.println("BOOT ++ (temporary)");
  else Serial.println(verboseFromNVS ? "NVS (Preferences)" : "SECRETS (DEBUG default)");
  Serial.print("Verbose: ");
  Serial.println(cfg.verbose ? "true" : "false");

  Serial.print("Color mode source: ");
  Serial.println(colorFromNVS ? "NVS (Preferences)" : "DEFAULT (AQI)");
  Serial.print("Color mode: ");
  Serial.println(colorModeToStr(cfg.color_mode));
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

  Serial.print("Connecting to SSID: ");
  Serial.println(cfg.ssid);
  oledMessage("Connecting WiFi...", cfg.ssid);

  WiFi.begin(cfg.ssid.c_str(), cfg.pass.c_str());

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeoutMs) {
    delay(50);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected.");

    // Warmup resolver after connect
    ledResolveNotBefore = millis() + RESOLVER_WARMUP_MS;

    if (cfg.verbose) {
      Serial.print("DNS0: "); Serial.println(WiFi.dnsIP(0));
      Serial.print("DNS1: "); Serial.println(WiFi.dnsIP(1));
      Serial.println("Resolver warmup started (2s)...");
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

  const uint32_t now = millis();
  if (now - tWifiRetry < WIFI_RETRY_MS) return;
  tWifiRetry = now;

  Serial.println("WiFi disconnected, retrying...");
  WiFi.disconnect(true);
  WiFi.setHostname(cfg.hostname.c_str());
  WiFi.begin(cfg.ssid.c_str(), cfg.pass.c_str());

  ledResolveNotBefore = millis() + RESOLVER_WARMUP_MS;
}

#if ENABLE_MDNS
void mdnsEnsureRunning() {
  static bool started = false;
  if (WiFi.status() != WL_CONNECTED) { started = false; return; }

  // Delay mDNS start until after warmup (helps stability)
  if (!started && millis() >= ledResolveNotBefore) {
    started = mdnsStart();
    if (cfg.verbose) Serial.println(started ? "mDNS running." : "mDNS failed.");
  }
}
#endif

// -----------------------------
// LED host resolution (IP / DNS / mDNS) with fallback
// -----------------------------
static bool dnsResolveWithFallback(const String& host, IPAddress& out) {
  // Try as-is first (FQDN)
  if (WiFi.hostByName(host.c_str(), out)) return true;

  // Then try short hostname (strip domain) using DHCP-provided DNS
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

  // If it's already an IP
  if (out.fromString(host)) return true;

  // Not *.local => normal DNS (plus short-host fallback)
  if (!host.endsWith(".local")) {
    // A few retries helps on some networks
    for (int i = 0; i < 4; i++) {
      if (dnsResolveWithFallback(host, out)) return true;
      delay(150);
      yield();
    }
    return false;
  }

  // *.local => mDNS query
#if ENABLE_MDNS
  String base = host;
  base.replace(".local", "");

  // More retries; mDNS can be slow right after WiFi connect
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

  Serial.print("Startup resolve LED host: ");
  Serial.println(host);

  const uint32_t start = millis();
  while ((millis() - start) < STARTUP_RESOLVE_MAX_MS) {
    if (millis() < ledResolveNotBefore) { delay(50); continue; }

#if ENABLE_MDNS
    mdnsEnsureRunning();
#endif

    if (resolveLedHostToIP(host, ip)) {
      led_cached_ip = ip;
      led_cached_ip_valid = true;
      Serial.print("Startup resolve OK: "); Serial.print(host);
      Serial.print(" -> "); Serial.println(ip.toString());
      oledMessage("LED resolved", host, ip.toString());
	  ledResolveState = "ok";
	  lastResolveMillis = millis();

      delay(300);
      return;
    }

    delay(STARTUP_RESOLVE_RETRY_MS);
    yield();
  }

  led_cached_ip_valid = false;
  led_cached_ip = IPAddress(0,0,0,0);
  Serial.println("Startup resolve FAILED. Will retry at runtime.");
  oledMessage("LED resolve FAIL", host, "retrying...");
  ledResolveState = "fail";
  lastResolveMillis = millis();

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
      Serial.print("Runtime resolve OK: ");
      Serial.print(host);
      Serial.print(" -> ");
      Serial.println(ip.toString());
    }
  } else 
	  ledResolveState = "fail";
		lastResolveMillis = millis();  
  if (cfg.verbose) {
    Serial.print("Runtime resolve FAILED for: ");
    Serial.println(host);
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
  // Suggested AQI-style mapping:
  // 400..800   green->yellow
  // 800..1200  yellow->orange
  // 1200..2000 orange->red
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

  // Mild gamma to avoid “too dim” mid colors
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
  // Your original wavelength-ish mapping.
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

  if (cfg.verbose) {
    Serial.print("Mode "); Serial.print(colorModeToStr(cfg.color_mode));
    Serial.print(" | CO2 "); Serial.print(co2ppm);
    Serial.print(" -> RGB ");
    Serial.print((int)lroundf(red)); Serial.print(":");
    Serial.print((int)lroundf(green)); Serial.print(":");
    Serial.println((int)lroundf(blue));
  }
}

// -----------------------------
// CO2 smoothing + hysteresis
// -----------------------------
float co2FilterAndHysteresis(float co2raw) {
  if (isnan(co2raw)) return NAN;

  // EMA
  if (isnan(co2_ema)) co2_ema = co2raw;
  else co2_ema = (CO2_EMA_ALPHA * co2raw) + ((1.0f - CO2_EMA_ALPHA) * co2_ema);

  bool adjusted = false;

  // Deadband hysteresis for color input
  if (isnan(co2_for_color)) co2_for_color = co2_ema;

  if (fabsf(co2_ema - co2_for_color) >= CO2_HYST_PPM) {
    co2_for_color = co2_ema;
  } else {
    // hysteresis holding the previous color value
    adjusted = true;
  }

  // Expose diagnostics for the web page
  lastCO2ema      = co2_ema;
  lastCO2forColor = co2_for_color;
  lastCO2Adjusted = adjusted;

  return co2_for_color;
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

  // Use filtered/hysteresis-stable CO2 for mapping
  float co2ppm = co2FilterAndHysteresis(co2ppmRaw);
  if (isnan(co2ppm)) return;

  mapCO2ToRGB(co2ppm);

  const int r = (int)lroundf(red);
  const int g = (int)lroundf(green);
  const int b = (int)lroundf(blue);

  // only-send-if-changed
  if (r == lastR && g == lastG && b == lastB) return;
  lastR = r; lastG = g; lastB = b;

  // Keep trying to resolve if we don't have a cached IP yet
  resolveLedAtRuntimeNonBlocking();

  const String host = sanitizeHost(cfg.led_host);

  int code = -1;

  // 1) Cached IP first (most reliable)
  if (led_cached_ip_valid) {
    code = shellySendRgb(r, g, b, led_cached_ip.toString(), 2000);
  }

  // 2) Resolve then send
  if (code != 200) {
    IPAddress resolved;
    if (resolveLedHostToIP(host, resolved)) {
      led_cached_ip = resolved;
      led_cached_ip_valid = true;
      code = shellySendRgb(r, g, b, resolved.toString(), 2000);
    }
  }

  // 3) Last resort: send with hostname as-is (HTTPClient will attempt DNS)
  if (code != 200) {
    code = shellySendRgb(r, g, b, host, 2000);
  }

  led_update_status = (code == 200) ? "<" : "x";
}

// -----------------------------
// OLED display
// -----------------------------
void displayOLED(float C, float H, float CO2) {
  if (!display.width()) return;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(0, 0);
  if (!isnan(C)) { display.print(C, 1); display.print(" C"); }
  else display.print("--.- C");

  display.setCursor(64, 0);
  if (!isnan(H)) { display.print(H, 1); display.print("%"); }
  else display.print("--.-%");

  display.setCursor(0, 10);
  display.print("CO2: ");
  if (!isnan(CO2)) display.print((int)lroundf(CO2));
  else display.print("---");
  display.print(" ppm");

  display.setCursor(0, 22);
  display.print("Mode: ");
  display.print(colorModeToStr(cfg.color_mode));

  display.setCursor(0, 34);
  display.print("SSID:");
  display.println(WiFi.SSID());

  display.setCursor(0, 46);
  display.print("IP: ");
  display.println(WiFi.localIP());

  display.setCursor(0, 56);
  display.print("LED:");
  display.print(cfg.led_host);
  display.print(" ");
  display.print(led_update_status);

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
// SCD30 setup (apply ONCE) + STATUS DEBUG
// -----------------------------
void applySCD30ConfigOnce(float altitude_m) {
  if (scd30_config_applied) return;

  bool ok = true;

  ok &= scd30.setMeasurementInterval(2);
  ok &= scd30.setAltitudeOffset((uint16_t)lroundf(altitude_m));
  ok &= scd30.selfCalibrationEnabled(true);

  // IMPORTANT: explicitly start measurement
  ok &= scd30.startContinuousMeasurement(0);

  scd30_config_applied = true;

  if (cfg.verbose) {
    Serial.println("SCD30 config/status:");
    Serial.print("  set ok: "); Serial.println(ok ? "true" : "false");
    Serial.print("  interval(s): "); Serial.println(scd30.getMeasurementInterval());
    Serial.print("  alt offset(m): "); Serial.println(scd30.getAltitudeOffset());
    Serial.print("  ASC enabled: "); Serial.println(scd30.selfCalibrationEnabled() ? "true" : "false");
    Serial.print("  forced cal ref(ppm): "); Serial.println(scd30.getForcedCalibrationReference());
    Serial.print("  temp offset(C): "); Serial.println((float)scd30.getTemperatureOffset() / 100.0f);
    Serial.println();
  }
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
  Serial.println();
  Serial.print("0 - Set WiFi SSID ["); Serial.print(cfg.ssid); Serial.println("]");
  Serial.print("1 - Set WiFi Password ["); Serial.print(cfg.pass); Serial.println("]");
  Serial.print("2 - Set LED host/IP ["); Serial.print(cfg.led_host); Serial.println("]");
  Serial.print("3 - Set Hostname ["); Serial.print(cfg.hostname); Serial.println("]");
  Serial.print("4 - Set Altitude (m) ["); Serial.print(cfg.altitude_m); Serial.println("]");
  Serial.print("5 - Report (A/T/H/C) ["); Serial.print(cfg.report); Serial.println("]");
  Serial.print("6 - Report headers+units (Y/N) ["); Serial.print(cfg.report_txt); Serial.println("]");
  Serial.print("7 - Color mode (toggle) ["); Serial.print(colorModeToStr(cfg.color_mode)); Serial.println("]");
  Serial.print("8 - Toggle verbose ["); Serial.print(cfg.verbose ? "true" : "false"); Serial.println("]");
  Serial.println("9 - Exit menu");
  Serial.println();
  Serial.print("Select (no Enter needed): ");
}

void runMenuWindow(uint32_t windowMs) {
  Serial.println();
  Serial.println("Serial Setup Menu available for 10 seconds.");
  Serial.println("Press any key to enter...");

  oledMessage("CO2 Sensor", String("Version: ") + MY_VERSION, "Press key for menu");

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
    if (choice == 0) return;     // timeout
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
      if (v.length()) cfg.pass = v;

    } else if (choice == '2') {
      flushSerialInput();
      Serial.print("Enter LED host/IP (then Enter): ");
      String v = readLineBlocking(60000);
      v.trim();
      if (v.length()) {
        // FIX: replace previous value, do not append
        cfg.led_host = sanitizeHost(v);
        led_cached_ip_valid = false;
        led_cached_ip = IPAddress(0,0,0,0);
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
// Setup
// -----------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(21, 22);

  pinMode(OLED_RESET_PIN, OUTPUT);
  digitalWrite(OLED_RESET_PIN, HIGH);
  delay(10);
  digitalWrite(OLED_RESET_PIN, LOW);
  delay(10);
  digitalWrite(OLED_RESET_PIN, HIGH);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("OLED SSD1306 not found. Continuing without display.");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.display();
  }

  Serial.println();
  Serial.println("CO2 Ventilation Sensor");
  Serial.print("Version: ");
  Serial.println(MY_VERSION);

  // Boot combos before reading config
  bootWindow();

  loadConfig();
  runMenuWindow(10000);

  wifiConnectBlocking(15000);

#if ENABLE_MDNS
  mdnsEnsureRunning();
#endif

  // Verify DNS/mDNS resolves at startup
  resolveLedAtStartupBlocking();

  server.begin();

  // Sensor init
  if (!scd30.begin()) {
    Serial.println("FATAL: Failed to find SCD30.");
    oledMessage("FATAL", "No SCD30 sensor", "Stopping");
    while (true) delay(50);
  }
  Serial.println("SCD30 Found!");

  if (!bme.begin()) {
    Serial.println("BME280 not found. Altitude disabled.");
    bme_sensor_found = false;
  } else {
    Serial.println("BME280 Found!");
    bme_sensor_found = true;
    readBMEandUpdateAltitude();
  }

  const float usedAlt = (!isnan(alt_s)) ? alt_s : (float)cfg.altitude_m;
  applySCD30ConfigOnce(usedAlt);

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

  // 1) Read SCD30
  if (now - tCO2 >= CO2_PERIOD_MS) {
    tCO2 = now;

    if (scd30.dataReady()) {
      if (!scd30.read()) {
        Serial.println("Error reading SCD30");
      } else {
        const float co2 = scd30.CO2;
		lastCO2raw = co2;

        // Ignore invalid 0 ppm samples (do not overwrite last valid)
        if (co2 <= 0.0f) {
          if (cfg.verbose) {
            Serial.println("SCD30 CO2 invalid (0 ppm) -> sample ignored (keeping last valid)");
          }
        } else {
          scd30_T_s = scd30.temperature;
          scd30_H_s = scd30.relative_humidity;
          scd30_CO2_s = co2;
          printSensorData();
        }
      }
    } else if (cfg.verbose) {
      Serial.println("SCD30: dataReady() = false");
    }
  }

  // 2) OLED update
  if (now - tOLED >= OLED_PERIOD_MS) {
    tOLED = now;
    displayOLED(scd30_T_s, scd30_H_s, scd30_CO2_s);
  }

  // 3) Shelly update (uses smoothing + hysteresis)
  setLightsFromCO2(scd30_CO2_s);

  // 4) Simple web server
  WiFiClient client = server.available();
  if (client) {
    client.setTimeout(200);

    bool currentLineIsBlank = true;
    while (client.connected()) {
      if (!client.available()) break;
      char c = client.read();

      if (c == '\n' && currentLineIsBlank) {
        client.println("HTTP/1.1 200 OK");
client.println("Content-Type: text/html; charset=utf-8");
client.println("Connection: close");
client.println("Refresh: 5");
client.println();
client.println("<!DOCTYPE HTML><html><head>");
client.println("<meta name='viewport' content='width=device-width, initial-scale=1'>");
client.println("<style>body{font-family:Arial;margin:14px;} .box{padding:10px;border:1px solid #ccc;border-radius:8px;margin:10px 0;} .k{color:#555;} code{word-break:break-all;}</style>");
client.println("</head><body>");

client.print("<h2>CO2 Sensor v"); client.print(MY_VERSION); client.println("</h2>");

/* ESP status */
client.println("<div class='box'>");
client.println("<b>ESP32</b><br/>");
client.print("<span class='k'>SSID:</span> "); client.print(WiFi.SSID()); client.println("<br/>");
client.print("<span class='k'>IP:</span> "); client.print(WiFi.localIP()); client.println("<br/>");
client.print("<span class='k'>RSSI:</span> "); client.print(WiFi.RSSI()); client.println(" dBm<br/>");
client.println("</div>");

/* Sensor readings */
client.println("<div class='box'>");
client.println("<b>SCD30</b><br/>");
client.print("<span class='k'>Temperature:</span> "); client.print(scd30_T_s, 1); client.println(" &deg;C<br/>");
client.print("<span class='k'>Humidity:</span> "); client.print(scd30_H_s, 1); client.println(" %<br/>");
client.print("<span class='k'>CO2 raw:</span> "); client.print(lastCO2raw, 0); client.println(" ppm<br/>");
client.print("<span class='k'>CO2 EMA:</span> "); client.print(lastCO2ema, 1); client.println(" ppm<br/>");
client.print("<span class='k'>CO2 used-for-color:</span> "); client.print(lastCO2forColor, 1); client.println(" ppm<br/>");
client.print("<span class='k'>Hysteresis holding?:</span> "); client.print(lastCO2Adjusted ? "YES" : "NO"); client.println("<br/>");
client.println("</div>");

/* LED config + resolve */
client.println("<div class='box'>");
client.println("<b>LED / Shelly</b><br/>");
client.print("<span class='k'>Configured LED host:</span> "); client.print(cfg.led_host); client.println("<br/>");
client.print("<span class='k'>Host source:</span> "); client.print(ledHostFromNVS ? "NVS override" : "Secrets default"); client.println("<br/>");
client.print("<span class='k'>Resolve state:</span> "); client.print(ledResolveState); client.println("<br/>");
client.print("<span class='k'>Cached IP:</span> ");
if (led_cached_ip_valid) client.print(led_cached_ip);
else client.print("none");
client.println("<br/>");
client.print("<span class='k'>Last Shelly status:</span> "); client.print(led_update_status); client.println("<br/>");
client.print("<span class='k'>Last HTTP code:</span> "); client.print(lastShellyHttpCode); client.println("<br/>");
client.print("<span class='k'>Last host used:</span> "); client.print(lastShellyHostUsed); client.println("<br/>");
client.print("<span class='k'>Last URL sent:</span> <code>"); client.print(lastShellyUrlSent); client.println("</code><br/>");
client.println("</div>");

/* Color mode */
client.println("<div class='box'>");
client.println("<b>Color</b><br/>");
client.print("<span class='k'>Mode:</span> "); client.print(colorModeToStr(cfg.color_mode)); client.println("<br/>");
client.print("<span class='k'>RGB last:</span> ");
client.print(lastR); client.print(","); client.print(lastG); client.print(","); client.print(lastB);
client.println("<br/>");
client.println("</div>");

#if ENABLE_MDNS
client.println("<div class='box'>");
client.println("<b>mDNS</b><br/>");
client.print("<span class='k'>ESP URL:</span> <code>http://"); client.print(cfg.hostname); client.println(".local/</code><br/>");
client.println("</div>");
#endif

client.println("</body></html>");

        break;
      }

      if (c == '\n') currentLineIsBlank = true;
      else if (c != '\r') currentLineIsBlank = false;
    }

    client.stop();
  }
}
