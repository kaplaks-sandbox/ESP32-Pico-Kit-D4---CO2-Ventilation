// ESP32-PICO-D4 CO2 Ventilation Sensor
// Optimized Version 6.9.3
//
// Key improvements:
// - Non-blocking loop scheduling (millis)
// - Shelly RGBW2 Gen1 control (HTTP GET /color/0?... or optional POST /color/0/set JSON)
// - Forces mode=color to avoid “white mode” confusion
// - Rate limiting + only-send-if-changed for Shelly
// - SCD30 config applied once (avoid repeated NVM writes)
// - Preferences managed via config struct
// - Optional mDNS initialization (ESP reachable at http://<hostname>.local/)
// - Robust DNS/mDNS LED resolution:
//     1) Warmup delay after WiFi connect (DHCP/DNS settle)
//     2) Delayed mDNS start after warmup (ESP32: no MDNS.update())
//     3) Sanitize host inputs (strip http(s):// and paths)
//     4) Resolve order:
//         - If IP: use directly
//         - If *.local: mDNS queryHost(base)
//         - Else: DNS resolve “as-is”, then DNS resolve short hostname (left of first '.') via DHCP DNS
//     5) Startup “ensure resolve” bounded retry + cache IP
//     6) Runtime retry if still unresolved
// - Factory reset combo at boot to wipe NVS (Preferences)
// - Boot key combo "++" enables debug for this boot (temporary)
// - DEBUG from secrets supported (optional macro DEBUG in arduino_secrets.h)
// - If debug enabled: prints full Shelly HTTP request + response body + error string
// - FIXED: Menu editing now replaces values reliably (no appending) by flushing serial RX before line prompts
//
// TO DO (as requested):
// - AsyncWebServer
// - OTA updates
// - MQTT instead of HTTP
// - CO2 hysteresis (to stop color flicker)

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

#include "arduino_secrets.h"  // MUST provide:
                              // SECRET_SSID, SECRET_PASS, SECRET_LED_IP, SECRET_HOSTNAME,
                              // SECRET_BAROMETERAltitude_m, SECRET_REPORT, SECRET_REPORT_TXT
                              // Optional: DEBUG

// -------- Optional mDNS --------
#ifndef ENABLE_MDNS
#define ENABLE_MDNS 1
#endif

#if ENABLE_MDNS
  #include <ESPmDNS.h>
#endif

// -------- Shelly transport mode --------
// 0 = GET query string (recommended)
// 1 = POST JSON to /color/0/set
#ifndef SHELLY_USE_POST_JSON
#define SHELLY_USE_POST_JSON 0
#endif

static const char* MY_VERSION = "6.9.3";

// -----------------------------
// Environmental assumptions
// -----------------------------
float referencePressure = 1013.25f;
float outdoorTemp = 4.7f;
float barometerAltitude = SECRET_BAROMETERAltitude_m;

// -----------------------------
// OLED
// -----------------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET_PIN 33
#define SCREEN_ADDRESS 0x3D
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET_PIN);

// -----------------------------
// BME280
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

struct Config {
  String ssid;
  String pass;
  String led_host;     // IP, FQDN, or *.local. Also tolerates http:// and paths (sanitized).
  String hostname;     // ESP32 hostname (and mDNS name)
  int    altitude_m;
  String report;       // A/T/H/C
  String report_txt;   // Y/N
  bool   verbose;
};

Config cfg;
static const char* PREF_NS = "my-app";

bool ledHostFromNVS = false;
bool enterMenuRequested = false;
bool bootDebugRequested = false;

// -----------------------------
// Web server
// -----------------------------
WiFiServer server(80);

// -----------------------------
// Runtime state
// -----------------------------
float scd30_T_s = NAN;
float scd30_H_s = NAN;
float scd30_CO2_s = NAN;

String led_update_status = "";
bool   scd30_config_applied = false;

float red = 0, green = 0, blue = 0;
int lastR = -1, lastG = -1, lastB = -1;

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
static constexpr uint32_t BOOT_WINDOW_MS   = 5000;

// Resolver warmup (after WiFi connect)
static uint32_t ledResolveNotBefore = 0;
static constexpr uint32_t RESOLVER_WARMUP_MS = 2000;

// Startup resolve window (bounded)
static constexpr uint32_t STARTUP_RESOLVE_MAX_MS = 15000;
static constexpr uint32_t STARTUP_RESOLVE_RETRY_MS = 500;

// Later resolve retry period if startup failed
static constexpr uint32_t RUNTIME_RESOLVE_RETRY_MS = 10000;

// -----------------------------
// Utility
// -----------------------------
static inline bool strYes(const String& s) {
  return (s.length() > 0 && (s[0] == 'Y' || s[0] == 'y'));
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

      enterMenuRequested = true;

      if (c == '\n') {
        String line = buf;
        buf = "";
        line.trim();
        if (line == "RESET") wipePreferencesNVS();
      } else {
        buf += c;

        if (buf.indexOf("!!") >= 0) wipePreferencesNVS();

        if (buf.indexOf("++") >= 0) {
          bootDebugRequested = true;
          buf = "++";
        }

        if (buf.length() > 32) buf.remove(0, buf.length() - 32);
      }
    }
    delay(10);
  }

  // Important: don't let boot keypress leak into menu prompt
  flushSerialInput();
}

// -----------------------------
// Config load/save
// -----------------------------
void loadConfig() {
  preferences.begin(PREF_NS, true);

  ledHostFromNVS = preferences.isKey("led");
  const bool verboseFromNVS = preferences.isKey("verbose");

  cfg.ssid       = preferences.getString("ssid", SECRET_SSID);
  cfg.pass       = preferences.getString("ssid_pwd", SECRET_PASS);
  cfg.led_host   = preferences.getString("led", SECRET_LED_IP);
  cfg.hostname   = preferences.getString("hostname", SECRET_HOSTNAME);
  cfg.altitude_m = preferences.getInt("altitude", SECRET_BAROMETERAltitude_m);
  cfg.report     = preferences.getString("report", SECRET_REPORT);
  cfg.report_txt = preferences.getString("report_txt", SECRET_REPORT_TXT);

  cfg.verbose    = preferences.getBool("verbose", debugFromSecretsDefault());
  preferences.end();

  if (bootDebugRequested) cfg.verbose = true;

  cfg.led_host = sanitizeHost(cfg.led_host);

  Serial.print("LED host in use: "); Serial.println(cfg.led_host);
  Serial.print("LED host source: "); Serial.println(ledHostFromNVS ? "NVS" : "SECRETS (SECRET_LED_IP)");

  Serial.print("Verbose source : ");
  if (bootDebugRequested) Serial.println("BOOT ++ (temporary)");
  else Serial.println(verboseFromNVS ? "NVS" : "SECRETS (DEBUG)");
  Serial.print("Verbose: "); Serial.println(cfg.verbose ? "true" : "false");
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

  preferences.end();
}

// -----------------------------
// WiFi + optional mDNS
// -----------------------------
#if ENABLE_MDNS
bool mdnsStart() {
  if (!MDNS.begin(cfg.hostname.c_str())) {
    Serial.println("mDNS: FAILED to start.");
    return false;
  }
  MDNS.addService("http", "tcp", 80);
  Serial.println("mDNS: started.");
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

  uint32_t now = millis();
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

  if (!started && millis() >= ledResolveNotBefore) {
    started = mdnsStart();
    if (cfg.verbose) Serial.println(started ? "mDNS running." : "mDNS failed.");
  }
}
#endif

// -----------------------------
// Robust resolver: IP / DNS(FQDN+short fallback) / mDNS (*.local)
// -----------------------------
static bool dnsResolveWithFallback(const String& host, IPAddress& out) {
  // 1) Try as-is first
  if (WiFi.hostByName(host.c_str(), out)) {
    if (cfg.verbose) {
      Serial.print("DNS resolved (as-is) "); Serial.print(host);
      Serial.print(" -> "); Serial.println(out.toString());
    }
    return true;
  }

  // 2) If it contains a dot, try the short hostname (left-most label)
  int dot = host.indexOf('.');
  if (dot > 0) {
    String shortHost = host.substring(0, dot);
    if (WiFi.hostByName(shortHost.c_str(), out)) {
      if (cfg.verbose) {
        Serial.print("DNS resolved (short) "); Serial.print(shortHost);
        Serial.print(" (from "); Serial.print(host); Serial.print(")");
        Serial.print(" -> "); Serial.println(out.toString());
      }
      return true;
    }
    if (cfg.verbose) {
      Serial.print("DNS short-host fallback failed for ");
      Serial.print(shortHost);
      Serial.print(" (from ");
      Serial.print(host);
      Serial.println(")");
    }
  }
  return false;
}

bool resolveLedHostToIP(const String& hostIn, IPAddress& out) {
  String host = sanitizeHost(hostIn);

  if (WiFi.status() != WL_CONNECTED) return false;

  if (millis() < ledResolveNotBefore) {
    if (cfg.verbose) Serial.println("Resolver warmup: skipping resolve this cycle.");
    return false;
  }

  if (out.fromString(host)) return true;

  if (!host.endsWith(".local")) {
    for (int i = 0; i < 4; i++) {
      if (dnsResolveWithFallback(host, out)) return true;
      if (cfg.verbose) {
        Serial.print("DNS retry "); Serial.print(i + 1);
        Serial.print(" failed for "); Serial.println(host);
      }
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
    if ((uint32_t)ip != 0) {
      out = ip;
      if (cfg.verbose) {
        Serial.print("mDNS resolved "); Serial.print(host);
        Serial.print(" -> "); Serial.println(out.toString());
      }
      return true;
    }
    if (cfg.verbose) {
      Serial.print("mDNS retry "); Serial.print(i + 1);
      Serial.print(" failed for "); Serial.println(host);
    }
    delay(250);
    yield();
  }
#endif

  return false;
}

// -----------------------------
// Startup ensures DNS/mDNS resolve + caches it
// -----------------------------
void resolveLedAtStartupBlocking() {
  if (WiFi.status() != WL_CONNECTED) return;

  const String host = sanitizeHost(cfg.led_host);
  IPAddress ip;

  Serial.print("Startup resolve LED host: ");
  Serial.println(host);

  const uint32_t start = millis();
  while ((millis() - start) < STARTUP_RESOLVE_MAX_MS) {

    if (millis() < ledResolveNotBefore) {
      delay(50);
      continue;
    }

#if ENABLE_MDNS
    mdnsEnsureRunning();
#endif

    if (resolveLedHostToIP(host, ip)) {
      led_cached_ip = ip;
      led_cached_ip_valid = true;

      Serial.print("Startup resolve OK: ");
      Serial.print(host);
      Serial.print(" -> ");
      Serial.println(ip.toString());

      oledMessage("LED resolved", host, ip.toString());
      delay(300);
      return;
    }

    delay(STARTUP_RESOLVE_RETRY_MS);
    yield();
  }

  led_cached_ip_valid = false;
  led_cached_ip = IPAddress(0,0,0,0);

  Serial.println("Startup resolve FAILED (bounded timeout). Will keep retrying in runtime.");
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
    if (cfg.verbose) {
      Serial.print("Runtime resolve OK: ");
      Serial.print(host);
      Serial.print(" -> ");
      Serial.println(ip.toString());
    }
  } else if (cfg.verbose) {
    Serial.print("Runtime resolve still failing for: ");
    Serial.println(host);
  }
}

// -----------------------------
// Shelly helpers
// -----------------------------
static void debugPrintResponseIfAny(HTTPClient& http, int httpCode) {
  if (!cfg.verbose) return;

  if (httpCode <= 0) {
    Serial.print("HTTP error: ");
    Serial.println(HTTPClient::errorToString(httpCode));
    return;
  }

  String payload = http.getString();
  if (payload.length() > 800) payload = payload.substring(0, 800) + "...";
  Serial.print("Shelly response body: ");
  Serial.println(payload);
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
  char url[300];
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

  if (cfg.verbose) {
    Serial.print("HTTP code: ");
    Serial.println(httpCode);
    debugPrintResponseIfAny(http, httpCode);
  }

  http.end();
  client.stop();
  return httpCode;
}

int shellySendRgb_POST(int r, int g, int b, const String& hostOrIp, uint32_t timeoutMs) {
  String url = String("http://") + hostOrIp + "/color/0/set";
  String json =
    String("{") +
    "\"mode\":\"color\"," +
    "\"red\":" + String(r) + "," +
    "\"green\":" + String(g) + "," +
    "\"blue\":" + String(b) + "," +
    "\"white\":0," +
    "\"gain\":100," +
    "\"effect\":0," +
    "\"turn\":\"on\"," +
    "\"transition\":500" +
    "}";

  if (cfg.verbose) {
    Serial.print("HTTP POST -> ");
    Serial.println(url);
    Serial.print("POST body  -> ");
    Serial.println(json);
  }

  WiFiClient client;
  HTTPClient http;
  http.setTimeout(timeoutMs);
  http.begin(client, url);
  http.addHeader("Content-Type", "application/json");

  int httpCode = http.POST((uint8_t*)json.c_str(), json.length());

  if (cfg.verbose) {
    Serial.print("HTTP code: ");
    Serial.println(httpCode);
    debugPrintResponseIfAny(http, httpCode);
  }

  http.end();
  client.stop();
  return httpCode;
}

int shellySendRgb(int r, int g, int b, const String& hostOrIp, uint32_t timeoutMs) {
#if SHELLY_USE_POST_JSON
  return shellySendRgb_POST(r, g, b, hostOrIp, timeoutMs);
#else
  return shellySendRgb_GET(r, g, b, hostOrIp, timeoutMs);
#endif
}

// -----------------------------
// CO2 -> RGB mapping
// -----------------------------
void wave2RGB(float co2ppm) {
  float wl = 380.0f + (co2ppm - 300.0f) * (780.0f - 380.0f) / (1000.0f - 300.0f);
  if (wl < 380.0f) wl = 380.0f;
  if (wl > 780.0f) wl = 780.0f;

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

  if (cfg.verbose) {
    Serial.print("CO2 "); Serial.print(co2ppm);
    Serial.print(" -> RGB ");
    Serial.print((int)lroundf(red)); Serial.print(":");
    Serial.print((int)lroundf(green)); Serial.print(":");
    Serial.println((int)lroundf(blue));
  }
}

// -----------------------------
// Shelly update with robust DNS/mDNS + caching
// -----------------------------
void setLightsFromCO2(float co2ppm) {
  const uint32_t now = millis();
  if (now - tShelly < SHELLY_PERIOD_MS) return;
  tShelly = now;

  if (isnan(co2ppm)) return;

  wave2RGB(co2ppm);

  const int r = (int)lroundf(red);
  const int g = (int)lroundf(green);
  const int b = (int)lroundf(blue);

  if (r == lastR && g == lastG && b == lastB) return;
  lastR = r; lastG = g; lastB = b;

  if (WiFi.status() != WL_CONNECTED) {
    led_update_status = "x";
    return;
  }

  resolveLedAtRuntimeNonBlocking();

  String host = sanitizeHost(cfg.led_host);
  int code = -1;

  // 1) cached IP first
  if (led_cached_ip_valid) {
    code = shellySendRgb(r, g, b, led_cached_ip.toString(), 2000);
  }

  // 2) resolve + retry
  if (code != 200) {
    IPAddress resolved;
    if (resolveLedHostToIP(host, resolved)) {
      led_cached_ip = resolved;
      led_cached_ip_valid = true;
      code = shellySendRgb(r, g, b, resolved.toString(), 2000);
    }
  }

  // 3) last resort raw host
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

  display.setCursor(120, 0);
  display.print(cfg.report);

  display.setCursor(0, 8);
  display.print("CO2:");

  display.setTextSize(3);
  display.setCursor(32, 8);
  if (!isnan(CO2)) display.print((int)lroundf(CO2));
  else display.print("---");

  display.setTextSize(1);
  display.setCursor(104, 24);
  display.print("ppm");

  if (bme_sensor_found && !isnan(alt_s)) {
    display.setCursor(0, 32);
    display.print(" Alt: ");
    display.print(alt_s, 0);
    display.print("m");
  }

  display.setCursor(0, 40);
  display.print("SSID:");
  display.println(WiFi.SSID());

  display.setCursor(0, 48);
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
// SCD30 setup (apply ONCE)
// -----------------------------
void applySCD30ConfigOnce(float altitude_m) {
  if (scd30_config_applied) return;
  scd30.setMeasurementInterval(2);
  scd30.setAltitudeOffset((uint16_t)lroundf(altitude_m));
  scd30.selfCalibrationEnabled(true);
  scd30_config_applied = true;
}

// -----------------------------
// Menu
// -----------------------------
void showMenu() {
  Serial.println();
  Serial.println("|***************************************|");
  Serial.println("|**|           CO2 Sensor            |**|");
  Serial.println("|***************************************|");
  Serial.println();
  Serial.print("0 - Set WiFi SSID ["); Serial.print(cfg.ssid); Serial.println("]");
  Serial.print("1 - Set WiFi Password ["); Serial.print(cfg.pass); Serial.println("]");
  Serial.print("2 - Set LED host/IP ["); Serial.print(cfg.led_host); Serial.println("]");
  Serial.print("3 - Set Hostname ["); Serial.print(cfg.hostname); Serial.println("]");
  Serial.print("4 - Set Altitude (m) ["); Serial.print(cfg.altitude_m); Serial.println("]");
  Serial.print("5 - Report (A/T/H/C) ["); Serial.print(cfg.report); Serial.println("]");
  Serial.print("6 - Report headers+units (Y/N) ["); Serial.print(cfg.report_txt); Serial.println("]");
  Serial.print("8 - Toggle verbose ["); Serial.print(cfg.verbose ? "true" : "false"); Serial.println("]");
  Serial.println("9 - Exit menu");
  Serial.println();
  Serial.print("Select (no Enter needed): ");
}

void runMenuWindow(uint32_t windowMs) {
  Serial.println();
  Serial.println("Serial Setup Menu available for 10 seconds.");
  Serial.println("Press any key to enter...");

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
      if (v.length()) cfg.pass = v;
    } else if (choice == '2') {
      // FIXED: flush before readLine so it replaces (not appends/garbles)
      flushSerialInput();
      Serial.print("Enter LED host/IP (then Enter): ");
      String v = readLineBlocking(60000);
      v.trim();
      if (v.length()) {
        cfg.led_host = sanitizeHost(v); // REPLACE
        led_cached_ip_valid = false;
        led_cached_ip = IPAddress(0,0,0,0);
        Serial.print("LED host set to: ");
        Serial.println(cfg.led_host);
      }
    } else if (choice == '3') {
      flushSerialInput();
      Serial.print("Enter Hostname (then Enter): ");
      String v = readLineBlocking(60000);
      v.trim();
      if (v.length()) cfg.hostname = v; // REPLACE
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

  bootWindow();
  loadConfig();
  runMenuWindow(10000);

  wifiConnectBlocking(15000);
  server.begin();

#if ENABLE_MDNS
  mdnsEnsureRunning();
#endif

  resolveLedAtStartupBlocking();

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
        scd30_T_s = scd30.temperature;
        scd30_H_s = scd30.relative_humidity;
        scd30_CO2_s = scd30.CO2;
        printSensorData();
      }
    }
  }

  // 2) OLED update
  if (now - tOLED >= OLED_PERIOD_MS) {
    tOLED = now;
    displayOLED(scd30_T_s, scd30_H_s, scd30_CO2_s);
  }

  // 3) Shelly update
  setLightsFromCO2(scd30_CO2_s);

  // 4) Web server
  WiFiClient client = server.available();
  if (client) {
    client.setTimeout(200);
    bool currentLineIsBlank = true;

    while (client.connected()) {
      if (!client.available()) break;
      char c = client.read();

      if (c == '\n' && currentLineIsBlank) {
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/html");
        client.println("Connection: close");
        client.println("Refresh: 5");
        client.println();
        client.println("<!DOCTYPE HTML><html>");

        client.print("<p>Temperature: "); client.print(scd30_T_s, 1); client.println("</p>");
        client.print("<p>Rel Humidity: "); client.print(scd30_H_s, 1); client.println("</p>");
        client.print("<p>CO2: "); client.print(scd30_CO2_s, 0); client.println("</p>");

        if (bme_sensor_found && !isnan(alt_s)) {
          client.print("<p>Altitude: "); client.print(alt_s, 0); client.println(" m</p>");
        }

#if ENABLE_MDNS
        client.print("<p>ESP mDNS: http://"); client.print(cfg.hostname); client.println(".local/</p>");
#endif

        client.println("</html>");
        break;
      }

      if (c == '\n') currentLineIsBlank = true;
      else if (c != '\r') currentLineIsBlank = false;
    }

    client.stop();
  }
}
