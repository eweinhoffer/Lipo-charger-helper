#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_MAX1704X.h>

#if __has_include("secrets.h")
#include "secrets.h"
#else
#error "Missing secrets.h. Copy secrets.example.h to secrets.h and set credentials."
#endif

#ifndef WIFI_STA_SSID
#error "WIFI_STA_SSID must be defined in secrets.h"
#endif
#ifndef WIFI_STA_PASS
#error "WIFI_STA_PASS must be defined in secrets.h"
#endif
#ifndef OTA_HTTP_PASSWORD
#error "OTA_HTTP_PASSWORD must be defined in secrets.h"
#endif
#ifndef OTA_HTTP_USER
#define OTA_HTTP_USER "ota"
#endif

#if __has_include(<USB.h>)
#include <USB.h>
#define HAS_ESP_USB_API 1
#else
#define HAS_ESP_USB_API 0
#endif

#ifndef TFT_CS
#define TFT_CS -1
#endif
#ifndef TFT_DC
#define TFT_DC 40
#endif
#ifndef TFT_RST
#define TFT_RST -1
#endif
#ifndef TFT_BACKLITE
#define TFT_BACKLITE -1
#endif
#ifndef TFT_I2C_POWER
#define TFT_I2C_POWER -1
#endif

#if defined(PIN_CHARGE)
static const int PIN_CHG_STATUS = PIN_CHARGE;
#elif defined(CHARGE_STATUS)
static const int PIN_CHG_STATUS = CHARGE_STATUS;
#else
static const int PIN_CHG_STATUS = -1;
#endif

#if defined(PIN_VBUS)
static const int PIN_USB_SENSE = PIN_VBUS;
#elif defined(VBUS_SENSE)
static const int PIN_USB_SENSE = VBUS_SENSE;
#else
static const int PIN_USB_SENSE = -1;
#endif

#ifndef BUTTON_BUILTIN
static const int PIN_PAGE_BUTTON = 0;
#else
static const int PIN_PAGE_BUTTON = BUTTON_BUILTIN;
#endif

static const int PIN_OTA_ARM_BUTTON = 1;  // D1 button (active HIGH on this board)
static const int PIN_EXTRA_BUTTON = 2;    // D2 button (active HIGH on this board)

static const uint16_t OTA_HTTP_PORT = 80;
static const uint32_t WIFI_RETRY_INTERVAL_MS = 180000;
static const uint32_t OTA_ARM_WINDOW_MS = 600000;
static const uint32_t BUTTON_LONG_PRESS_MS = 2200;
static const float MIN_EST_RATE_PCT_PER_HR = 0.2f;
static const float MAX_EST_RATE_PCT_PER_HR = 40.0f;

Adafruit_ST7789 tft(TFT_CS, TFT_DC, TFT_RST);
Adafruit_MAX17048 maxlipo;
WebServer otaHttp(OTA_HTTP_PORT);

enum ChargeState {
  CHARGE_UNKNOWN = 0,
  CHARGE_NONE,
  CHARGE_CHARGING,
  CHARGE_FULL,
  CHARGE_IDLE,
  CHARGE_DISCHARGING
};

enum PowerTopology {
  POWER_UNKNOWN = 0,
  POWER_USB_COMPUTER_ONLY,
  POWER_USB_POWER_ONLY,
  POWER_USB_AND_BATTERY,
  POWER_BATTERY_ONLY
};

enum OtaState {
  OTA_READY = 0,
  OTA_UPDATING,
  OTA_ERROR
};

struct Telemetry {
  bool gaugeOnline;
  bool batteryPresent;
  bool batterySampleValid;
  bool usbByPin;
  bool usbByHostMount;
  bool usbByInference;
  bool usbPresent;
  bool chargerPinAvailable;
  bool chargerPinCharging;
  float vbat;
  float vbatMed;
  float soc;
  float cRate;
  float cRateMed;
  float cRateAvg;
  float socTrendPerHour;
  ChargeState chargeState;
  PowerTopology topology;
};

static Telemetry m = {};
static uint8_t currentPage = 0;
static const uint8_t PAGE_COUNT = 3;
static bool layoutDirty = true;

static uint32_t lastSensorMs = 0;
static uint32_t lastUiMs = 0;
static uint32_t usbSeenUntilMs = 0;
static uint32_t lastWiFiRetryMs = 0;

static bool lastBtnRaw = HIGH;
static bool btnStable = HIGH;
static bool btnPressedEdge = false;
static uint32_t lastDebounceMs = 0;

static bool lastOtaBtnRaw = HIGH;
static bool otaBtnStable = HIGH;
static bool otaBtnLongPressEdge = false;
static bool otaBtnLongFired = false;
static uint32_t lastOtaDebounceMs = 0;
static uint32_t otaBtnPressedSinceMs = 0;

static PowerTopology prevTopology = POWER_UNKNOWN;
static ChargeState prevChargeState = CHARGE_UNKNOWN;
static float prevSoc = NAN;
static float prevVbat = NAN;
static float prevRate = NAN;
static bool prevGaugeOnline = false;
static bool prevUsbPresent = false;
static bool prevWiFiConnected = false;
static String prevIP = "";
static OtaState prevOtaState = OTA_READY;

static bool wifiConnected = false;
static IPAddress wifiIP(0, 0, 0, 0);
static OtaState otaState = OTA_READY;
static String otaLastError = "";
static bool otaWindowArmed = false;
static uint32_t otaWindowUntilMs = 0;
static bool d0Pressed = false;
static bool d1Pressed = false;
static bool d2Pressed = false;
static bool d2ShortPressEdge = false;
static bool d2LongPressEdge = false;
static bool d2LongFired = false;
static bool d2Stable = LOW;
static bool lastD2Raw = LOW;
static uint32_t lastD2DebounceMs = 0;
static uint32_t d2PressedSinceMs = 0;
enum UserBatteryMark {
  USER_MARK_UNKNOWN = 0,
  USER_MARK_NO_BATTERY,
  USER_MARK_BATTERY_PRESENT
};
static UserBatteryMark userBatteryMark = USER_MARK_UNKNOWN;
static bool userBatteryMarkLocked = false;
static String debugLogBuffer = "";
static uint32_t debugSeq = 0;

static const uint8_t TELEMETRY_HISTORY_LEN = 12;
static float histSoc[TELEMETRY_HISTORY_LEN] = {};
static float histVbat[TELEMETRY_HISTORY_LEN] = {};
static float histRate[TELEMETRY_HISTORY_LEN] = {};
static uint32_t histMs[TELEMETRY_HISTORY_LEN] = {};
static uint8_t histCount = 0;
static uint8_t histHead = 0;
static uint8_t lastGaugeAlerts = 0;
static uint8_t batteryValidStreak = 0;
static uint8_t batteryInvalidStreak = 0;
static uint8_t usbPresentStreak = 0;
static uint8_t usbMissingStreak = 0;
static const uint8_t RAW_MEDIAN_LEN = 3;
static float rawRate[RAW_MEDIAN_LEN] = {};
static float rawVbat[RAW_MEDIAN_LEN] = {};
static uint8_t rawCount = 0;
static uint8_t rawHead = 0;

bool almostEqual(float a, float b, float eps) {
  if (isnan(a) && isnan(b)) return true;
  if (isnan(a) || isnan(b)) return false;
  return fabsf(a - b) < eps;
}

bool readPinHigh(int pin) {
  if (pin < 0) return false;
  pinMode(pin, INPUT);
  return digitalRead(pin) == HIGH;
}

const char *userMarkText(UserBatteryMark mark) {
  switch (mark) {
    case USER_MARK_NO_BATTERY: return "NO_BATTERY";
    case USER_MARK_BATTERY_PRESENT: return "BATTERY_PRESENT";
    default: return "UNKNOWN";
  }
}

void clearTelemetryHistory() {
  histCount = 0;
  histHead = 0;
}

void pushTelemetrySample(float soc, float vbat, float cRate) {
  histSoc[histHead] = soc;
  histVbat[histHead] = vbat;
  histRate[histHead] = cRate;
  histMs[histHead] = millis();
  histHead = (uint8_t)((histHead + 1) % TELEMETRY_HISTORY_LEN);
  if (histCount < TELEMETRY_HISTORY_LEN) histCount++;
}

float averageHistory(const float *arr) {
  if (histCount == 0) return NAN;
  float sum = 0.0f;
  uint8_t idx = (uint8_t)((histHead + TELEMETRY_HISTORY_LEN - histCount) % TELEMETRY_HISTORY_LEN);
  for (uint8_t i = 0; i < histCount; i++) {
    sum += arr[idx];
    idx = (uint8_t)((idx + 1) % TELEMETRY_HISTORY_LEN);
  }
  return sum / histCount;
}

float socTrendPerHour() {
  if (histCount < 3) return NAN;
  uint8_t firstIdx = (uint8_t)((histHead + TELEMETRY_HISTORY_LEN - histCount) % TELEMETRY_HISTORY_LEN);
  uint8_t lastIdx = (uint8_t)((histHead + TELEMETRY_HISTORY_LEN - 1) % TELEMETRY_HISTORY_LEN);
  uint32_t dtMs = histMs[lastIdx] - histMs[firstIdx];
  if (dtMs < 1000) return NAN;
  float dtHours = dtMs / 3600000.0f;
  if (dtHours <= 0.0f) return NAN;
  return (histSoc[lastIdx] - histSoc[firstIdx]) / dtHours;
}

void clearRawSamples() {
  rawCount = 0;
  rawHead = 0;
}

void pushRawSample(float vbat, float cRate) {
  rawVbat[rawHead] = vbat;
  rawRate[rawHead] = cRate;
  rawHead = (uint8_t)((rawHead + 1) % RAW_MEDIAN_LEN);
  if (rawCount < RAW_MEDIAN_LEN) rawCount++;
}

float medianRecent(const float *arr) {
  float vals[RAW_MEDIAN_LEN];
  uint8_t n = 0;
  uint8_t idx = (uint8_t)((rawHead + RAW_MEDIAN_LEN - rawCount) % RAW_MEDIAN_LEN);
  for (uint8_t i = 0; i < rawCount; i++) {
    float v = arr[idx];
    if (!isnan(v)) vals[n++] = v;
    idx = (uint8_t)((idx + 1) % RAW_MEDIAN_LEN);
  }
  if (n == 0) return NAN;
  if (n == 1) return vals[0];
  if (n == 2) return 0.5f * (vals[0] + vals[1]);
  if (vals[1] < vals[0]) {
    float t = vals[0];
    vals[0] = vals[1];
    vals[1] = t;
  }
  if (vals[2] < vals[1]) {
    float t = vals[1];
    vals[1] = vals[2];
    vals[2] = t;
    if (vals[1] < vals[0]) {
      t = vals[0];
      vals[0] = vals[1];
      vals[1] = t;
    }
  }
  return vals[1];
}

bool batterySampleValid(float vbat, float soc, float cRate) {
  (void)vbat;
  (void)cRate;
  // Requested simple rule: SOC drives battery connected/disconnected.
  // >100% => disconnected. 0..100% => connected.
  return !isnan(soc) && soc >= 0.0f && soc <= 100.0f;
}

bool readUsbHostMounted() {
#if HAS_ESP_USB_API
  return (bool)USB;
#else
  return false;
#endif
}

ChargeState computeChargeState() {
  if (!m.batteryPresent) return CHARGE_NONE;

  float rate = m.cRateMed;
  if (isnan(rate)) rate = isnan(m.cRateAvg) ? m.cRate : m.cRateAvg;
  if (isnan(rate)) return CHARGE_UNKNOWN;
  return (rate >= 0.0f) ? CHARGE_CHARGING : CHARGE_DISCHARGING;
}

PowerTopology computeTopology() {
  if (!m.batteryPresent && m.usbPresent) {
    if (m.usbByHostMount) return POWER_USB_COMPUTER_ONLY;
    return POWER_USB_POWER_ONLY;
  }
  if (m.batteryPresent && m.usbPresent) return POWER_USB_AND_BATTERY;
  if (m.batteryPresent && !m.usbPresent) return POWER_BATTERY_ONLY;
  return POWER_UNKNOWN;
}

const __FlashStringHelper *topologyText(PowerTopology t) {
  switch (t) {
    case POWER_USB_COMPUTER_ONLY: return F("USB COMPUTER");
    case POWER_USB_POWER_ONLY: return F("USB POWER");
    case POWER_USB_AND_BATTERY: return F("USB + BATTERY");
    case POWER_BATTERY_ONLY: return F("BATTERY ONLY");
    default: return F("UNKNOWN");
  }
}

const __FlashStringHelper *chargeText(ChargeState s) {
  switch (s) {
    case CHARGE_NONE: return F("NO BATTERY");
    case CHARGE_CHARGING: return F("CHARGING");
    case CHARGE_FULL: return F("FULL");
    case CHARGE_IDLE: return F("USB IDLE");
    case CHARGE_DISCHARGING: return F("DISCHARGING");
    default: return F("UNKNOWN");
  }
}

const __FlashStringHelper *otaText(OtaState s) {
  switch (s) {
    case OTA_READY: return F("READY");
    case OTA_UPDATING: return F("UPDATING");
    case OTA_ERROR: return F("ERROR");
    default: return F("UNKNOWN");
  }
}

String otaLabel() {
  if (!wifiConnected) return String("OTA WAIT WIFI");
  if (otaState == OTA_ERROR) {
    if (otaLastError.length() > 0) return String("OTA ERROR: ") + otaLastError;
    return String("OTA ERROR");
  }
  if (otaState == OTA_UPDATING) return String("OTA UPDATING");
  if (!otaWindowArmed) return String("OTA LOCKED");
  int32_t remainMs = (int32_t)(otaWindowUntilMs - millis());
  if (remainMs <= 0) return String("OTA LOCKED");
  uint32_t remainMin = (uint32_t)((remainMs + 59999) / 60000);
  return String("OTA READY ") + String(remainMin) + String("m");
}

String batteryTimeRemainingLabel() {
  if (!m.batteryPresent || isnan(m.soc) || isnan(m.cRate)) {
    return String("Est left: --");
  }
  if (m.soc >= 99.8f) {
    return String("Est full: soon");
  }

  // cRate is %/hr. Positive = charging, negative = discharging.
  // This is read-only telemetry; actual charge safety/current is controlled by board charger hardware.
  float rate = isnan(m.cRateAvg) ? m.cRate : m.cRateAvg;

  if (m.chargeState == CHARGE_CHARGING || rate > 0.6f) {
    float chargeRate = rate;
    if (chargeRate < MIN_EST_RATE_PCT_PER_HR || chargeRate > MAX_EST_RATE_PCT_PER_HR) {
      return String("Est full: --");
    }
    float pctToFull = 100.0f - m.soc;
    if (pctToFull < 0.0f) pctToFull = 0.0f;
    float hoursToFull = pctToFull / chargeRate;
    int totalMinutes = (int)roundf(hoursToFull * 60.0f);
    int hh = totalMinutes / 60;
    int mm = totalMinutes % 60;
    return String("Est full ") + String(hh) + String("h ") + String(mm) + String("m");
  }

  float drainRate = -rate;
  if (drainRate < MIN_EST_RATE_PCT_PER_HR || drainRate > MAX_EST_RATE_PCT_PER_HR) {
    return String("Est left: --");
  }

  float hoursLeft = m.soc / drainRate;
  if (hoursLeft < 0.0f) return String("Est left: --");

  int totalMinutes = (int)roundf(hoursLeft * 60.0f);
  int hh = totalMinutes / 60;
  int mm = totalMinutes % 60;
  return String("Est left ") + String(hh) + String("h ") + String(mm) + String("m");
}

bool otaWindowActive() {
  if (!otaWindowArmed) return false;
  return ((int32_t)(otaWindowUntilMs - millis()) > 0);
}

void armOtaWindow() {
  otaWindowArmed = true;
  otaWindowUntilMs = millis() + OTA_ARM_WINDOW_MS;
  otaState = OTA_READY;
  otaLastError = "";
  layoutDirty = true;
}

uint16_t chargeColor(ChargeState s) {
  switch (s) {
    case CHARGE_NONE: return ST77XX_YELLOW;
    case CHARGE_CHARGING: return ST77XX_GREEN;
    case CHARGE_FULL: return ST77XX_CYAN;
    case CHARGE_IDLE: return ST77XX_WHITE;
    case CHARGE_DISCHARGING: return ST77XX_ORANGE;
    default: return ST77XX_WHITE;
  }
}

uint16_t topologyColor(PowerTopology t) {
  switch (t) {
    case POWER_USB_COMPUTER_ONLY: return ST77XX_CYAN;
    case POWER_USB_POWER_ONLY: return ST77XX_BLUE;
    case POWER_USB_AND_BATTERY: return ST77XX_GREEN;
    case POWER_BATTERY_ONLY: return ST77XX_YELLOW;
    default: return ST77XX_RED;
  }
}

uint16_t otaColor(OtaState s) {
  switch (s) {
    case OTA_READY: return ST77XX_GREEN;
    case OTA_UPDATING: return ST77XX_YELLOW;
    case OTA_ERROR: return ST77XX_RED;
    default: return ST77XX_WHITE;
  }
}

void refreshWiFiState() {
  bool wasConnected = wifiConnected;
  IPAddress oldIP = wifiIP;

  wifiConnected = (WiFi.status() == WL_CONNECTED);
  wifiIP = wifiConnected ? WiFi.localIP() : IPAddress(0, 0, 0, 0);

  if (!wifiConnected && (millis() - lastWiFiRetryMs >= WIFI_RETRY_INTERVAL_MS)) {
    WiFi.disconnect();
    WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASS);
    lastWiFiRetryMs = millis();
  }

  if (wasConnected != wifiConnected || oldIP != wifiIP) {
    layoutDirty = true;
  }
}

bool httpAuthOk() {
  if (!otaHttp.authenticate(OTA_HTTP_USER, OTA_HTTP_PASSWORD)) {
    otaHttp.requestAuthentication();
    return false;
  }
  return true;
}

void setupHttpOTA() {
  otaHttp.on("/", HTTP_GET, []() {
    if (!httpAuthOk()) return;
    String msg = String("HTTP OTA ") + (otaWindowActive() ? String("ready") : String("locked")) +
                 String("\nPOST firmware binary to /update\n");
    otaHttp.send(200, "text/plain", msg);
  });

  otaHttp.on("/debug", HTTP_GET, []() {
    if (!httpAuthOk()) return;
    String out = String("LiPo debug log\n");
    out += String("D2 short: cycle mark, D2 long: lock/unlock mark (annotation only)\n");
    out += String("Current mark=") + String(userMarkText(userBatteryMark)) +
           String(" (") + String(userBatteryMarkLocked ? "LOCKED" : "UNLOCKED") + String(")\n");
    out += debugLogBuffer;
    otaHttp.send(200, "text/plain", out);
  });

  otaHttp.on("/update", HTTP_POST,
    []() {
      if (!httpAuthOk()) return;
      if (!otaWindowActive()) {
        otaState = OTA_ERROR;
        otaLastError = "locked";
        otaHttp.send(403, "text/plain", "OTA LOCKED\nHold D1 for 2.2s.\n");
        layoutDirty = true;
        return;
      }
      if (Update.hasError()) {
        otaState = OTA_ERROR;
        if (otaLastError.length() == 0) otaLastError = "write";
        otaHttp.send(500, "text/plain", "OTA FAIL\n");
      } else {
        otaState = OTA_READY;
        otaLastError = "";
        otaHttp.send(200, "text/plain", "OTA OK, rebooting\n");
        delay(150);
        ESP.restart();
      }
      layoutDirty = true;
    },
    []() {
      HTTPUpload &up = otaHttp.upload();
      if (up.status == UPLOAD_FILE_START) {
        if (!httpAuthOk()) return;
        if (!otaWindowActive()) {
          otaState = OTA_ERROR;
          otaLastError = "locked";
          layoutDirty = true;
          return;
        }
        otaState = OTA_UPDATING;
        otaLastError = "";
        layoutDirty = true;
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
          otaState = OTA_ERROR;
          otaLastError = "begin";
          Update.printError(Serial);
        }
      } else if (up.status == UPLOAD_FILE_WRITE) {
        if (Update.write(up.buf, up.currentSize) != up.currentSize) {
          otaState = OTA_ERROR;
          otaLastError = "write";
          Update.printError(Serial);
        }
      } else if (up.status == UPLOAD_FILE_END) {
        if (!Update.end(true)) {
          otaState = OTA_ERROR;
          otaLastError = "end";
          Update.printError(Serial);
        }
      } else if (up.status == UPLOAD_FILE_ABORTED) {
        Update.abort();
        otaState = OTA_ERROR;
        otaLastError = "aborted";
      }
    }
  );

  otaHttp.begin();
}

void setupWiFiAndOTA() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASS);
  lastWiFiRetryMs = millis();

  refreshWiFiState();

  setupHttpOTA();
}

void readButton() {
  bool raw = digitalRead(PIN_PAGE_BUTTON);
  d0Pressed = (raw == LOW);
  static bool prevD0Pressed = false;
  if (d0Pressed != prevD0Pressed) {
    prevD0Pressed = d0Pressed;
    if (currentPage == 1) layoutDirty = true;
  }
  if (raw != lastBtnRaw) {
    lastDebounceMs = millis();
    lastBtnRaw = raw;
  }

  btnPressedEdge = false;
  if (millis() - lastDebounceMs > 25) {
    if (btnStable != raw) {
      btnStable = raw;
      if (btnStable == LOW) {
        btnPressedEdge = true;
      }
    }
  }
}

void readOtaArmButton() {
  bool raw = digitalRead(PIN_OTA_ARM_BUTTON);
  d1Pressed = (raw == HIGH);
  static bool prevD1Pressed = false;
  if (d1Pressed != prevD1Pressed) {
    prevD1Pressed = d1Pressed;
    if (currentPage == 1) layoutDirty = true;
  }
  if (raw != lastOtaBtnRaw) {
    lastOtaDebounceMs = millis();
    lastOtaBtnRaw = raw;
  }

  otaBtnLongPressEdge = false;
  if (millis() - lastOtaDebounceMs > 25) {
    if (otaBtnStable != raw) {
      otaBtnStable = raw;
      if (otaBtnStable == HIGH) {
        otaBtnPressedSinceMs = millis();
        otaBtnLongFired = false;
      } else {
        otaBtnPressedSinceMs = 0;
        otaBtnLongFired = false;
      }
    }

    if (otaBtnStable == HIGH && !otaBtnLongFired && otaBtnPressedSinceMs > 0 &&
        (millis() - otaBtnPressedSinceMs >= BUTTON_LONG_PRESS_MS)) {
      otaBtnLongPressEdge = true;
      otaBtnLongFired = true;
    }
  }
}

void readDebugButtonD2() {
  bool raw = digitalRead(PIN_EXTRA_BUTTON);
  d2Pressed = (raw == HIGH);
  if (raw != lastD2Raw) {
    lastD2DebounceMs = millis();
    lastD2Raw = raw;
  }

  d2ShortPressEdge = false;
  d2LongPressEdge = false;
  if (millis() - lastD2DebounceMs > 25) {
    if (d2Stable != raw) {
      d2Stable = raw;
      if (d2Stable == HIGH) {
        d2PressedSinceMs = millis();
        d2LongFired = false;
      } else {
        if (d2PressedSinceMs > 0 && !d2LongFired) d2ShortPressEdge = true;
        d2PressedSinceMs = 0;
        d2LongFired = false;
      }
    }

    if (d2Stable == HIGH && !d2LongFired && d2PressedSinceMs > 0 &&
        (millis() - d2PressedSinceMs >= BUTTON_LONG_PRESS_MS)) {
      d2LongPressEdge = true;
      d2LongFired = true;
    }
  }
}

void logSnapshot(const char *tag) {
  String line = String("#") + String(++debugSeq) + String(" t=") + String(millis()) +
                String(" tag=") + String(tag) +
                String(" usb=") + String(m.usbByPin) + String("/") + String(m.usbByHostMount) +
                String("/") + String(m.usbByInference) + String("/") + String(m.usbPresent) +
                String(" batt=") + String(m.batteryPresent) +
                String(" valid=") + String(m.batterySampleValid) +
                String(" streak(bv/bi)=") + String(batteryValidStreak) + String("/") + String(batteryInvalidStreak) +
                String(" streak(up/um)=") + String(usbPresentStreak) + String("/") + String(usbMissingStreak) +
                String(" chg=") + String(m.chargerPinCharging) +
                String(" soc=") + String(m.soc, 1) +
                String(" v=") + String(m.vbat, 3) +
                String(" rate=") + String(m.cRate, 2) +
                String(" med=") + String(m.cRateMed, 2) +
                String(" avg=") + String(m.cRateAvg, 2) +
                String(" trend=") + String(m.socTrendPerHour, 2) +
                String(" alerts=0x") + String(lastGaugeAlerts, HEX) +
                String(" mark=") + String(userMarkText(userBatteryMark)) +
                String(" lock=") + String(userBatteryMarkLocked ? "LOCKED" : "UNLOCKED");

  Serial.println(line);
  debugLogBuffer += line;
  debugLogBuffer += '\n';
  const int maxLogChars = 6000;
  if (debugLogBuffer.length() > maxLogChars) {
    int trimTo = debugLogBuffer.length() - maxLogChars;
    int nl = debugLogBuffer.indexOf('\n', trimTo);
    if (nl > 0) debugLogBuffer.remove(0, nl + 1);
    else debugLogBuffer.remove(0, trimTo);
  }
}

void sampleTelemetry() {
  if (PIN_CHG_STATUS >= 0) {
    m.chargerPinAvailable = true;
    pinMode(PIN_CHG_STATUS, INPUT_PULLUP);
    m.chargerPinCharging = (digitalRead(PIN_CHG_STATUS) == LOW);
  } else {
    m.chargerPinAvailable = false;
    m.chargerPinCharging = false;
  }

  // Read direct USB indications early so we can use them in battery presence logic.
  m.usbByPin = readPinHigh(PIN_USB_SENSE);
  m.usbByHostMount = readUsbHostMounted();
  bool usbDirectNow = m.usbByPin || m.usbByHostMount;

  if (m.gaugeOnline) {
    m.vbat = maxlipo.cellVoltage();
    m.soc = maxlipo.cellPercent();
    m.cRate = maxlipo.chargeRate();
    lastGaugeAlerts = maxlipo.getAlertStatus();
    pushRawSample(m.vbat, m.cRate);
    m.vbatMed = medianRecent(rawVbat);
    m.cRateMed = medianRecent(rawRate);

    bool sampleValid = batterySampleValid(m.vbat, m.soc, m.cRate);
    m.batterySampleValid = sampleValid;
    if (sampleValid) {
      batteryValidStreak = (uint8_t)min<int>(batteryValidStreak + 1, 10);
      batteryInvalidStreak = 0;
    } else {
      batteryInvalidStreak = (uint8_t)min<int>(batteryInvalidStreak + 1, 10);
      batteryValidStreak = 0;
    }

    bool wasBatteryPresent = m.batteryPresent;
    if (batteryValidStreak >= 2) m.batteryPresent = true;
    else if (batteryInvalidStreak >= 2) m.batteryPresent = false;

    if (m.batteryPresent && !wasBatteryPresent) {
      clearTelemetryHistory();
      clearRawSamples();
      logSnapshot("BATTERY_PRESENT");
    } else if (!m.batteryPresent && wasBatteryPresent) {
      clearTelemetryHistory();
      clearRawSamples();
      m.cRateAvg = NAN;
      m.socTrendPerHour = NAN;
      logSnapshot("BATTERY_MISSING");
    }

    if (m.batteryPresent && sampleValid) {
      float smoothedVbat = isnan(m.vbatMed) ? m.vbat : m.vbatMed;
      float smoothedRate = isnan(m.cRateMed) ? m.cRate : m.cRateMed;
      pushTelemetrySample(m.soc, smoothedVbat, smoothedRate);
      m.cRateAvg = averageHistory(histRate);
      m.socTrendPerHour = socTrendPerHour();
    } else {
      clearTelemetryHistory();
      m.cRateAvg = NAN;
      m.socTrendPerHour = NAN;
    }

    if (lastGaugeAlerts & MAX1704X_ALERTFLAG_VOLTAGE_RESET) {
      maxlipo.clearAlertFlag(MAX1704X_ALERTFLAG_VOLTAGE_RESET);
    }
    if (lastGaugeAlerts & MAX1704X_ALERTFLAG_RESET_INDICATOR) {
      maxlipo.clearAlertFlag(MAX1704X_ALERTFLAG_RESET_INDICATOR);
    }
  } else {
    m.vbat = NAN;
    m.vbatMed = NAN;
    m.soc = NAN;
    m.cRate = NAN;
    m.cRateMed = NAN;
    m.cRateAvg = NAN;
    m.socTrendPerHour = NAN;
    m.batterySampleValid = false;
    m.batteryPresent = false;
    batteryValidStreak = 0;
    batteryInvalidStreak = 10;
    clearTelemetryHistory();
    clearRawSamples();
    lastGaugeAlerts = 0;
  }

  m.usbByInference = false;
  float rateForUsb = m.cRateMed;
  if (isnan(rateForUsb)) rateForUsb = isnan(m.cRateAvg) ? m.cRate : m.cRateAvg;
  bool batteryStrongMissing = (!m.batteryPresent) && (batteryInvalidStreak >= 2);
  bool gaugeProfileInvalidNow = (m.soc > 100.5f) || (m.soc < 0.0f) || isnan(m.soc);
  bool usbStrongEvidenceNow = usbDirectNow;
  if (!usbDirectNow) {
    // Board still running with battery strongly missing => likely powered from USB.
    if (batteryStrongMissing) {
      m.usbByInference = true;
      usbStrongEvidenceNow = true;
    } else if (m.chargerPinAvailable && m.chargerPinCharging) {
      m.usbByInference = true;
      usbStrongEvidenceNow = true;
    } else if (!isnan(rateForUsb) && rateForUsb > 0.8f && batteryValidStreak >= 2) {
      m.usbByInference = true;
      usbStrongEvidenceNow = true;
    } else if (gaugeProfileInvalidNow &&
               (int32_t)(usbSeenUntilMs - millis()) > 0) {
      // Keep USB latched briefly through MAX17048 transitional invalid profile.
      m.usbByInference = true;
    }
    else if (!isnan(rateForUsb) && rateForUsb < -0.8f) m.usbByInference = false;
  }

  if (usbStrongEvidenceNow) {
    usbSeenUntilMs = millis() + 120000;  // keep 2-minute transition window
  }

  bool usbNowCandidate = usbDirectNow || m.usbByInference;
  if (usbNowCandidate) {
    usbPresentStreak = (uint8_t)min<int>(usbPresentStreak + 1, 10);
    usbMissingStreak = 0;
  } else {
    usbMissingStreak = (uint8_t)min<int>(usbMissingStreak + 1, 10);
    usbPresentStreak = 0;
  }

  // Hysteresis to prevent state flapping.
  if (usbPresentStreak >= 2) m.usbPresent = true;
  else if (usbMissingStreak >= 3) m.usbPresent = false;

  m.chargeState = computeChargeState();
  m.topology = computeTopology();

  static ChargeState lastLoggedCharge = CHARGE_UNKNOWN;
  static PowerTopology lastLoggedTopo = POWER_UNKNOWN;
  if (m.chargeState != lastLoggedCharge || m.topology != lastLoggedTopo) {
    lastLoggedCharge = m.chargeState;
    lastLoggedTopo = m.topology;
    logSnapshot("STATE");
  }
}

void drawHeader() {
  tft.fillRect(0, 0, 240, 26, ST77XX_BLUE);
  tft.setTextWrap(false);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLUE);
  tft.setCursor(8, 6);
  tft.print(F("LiPo Status"));
  tft.setTextSize(1);
  tft.setCursor(180, 9);
  if (currentPage == 0) tft.print(F("MAIN"));
  else if (currentPage == 1) tft.print(F("DEBUG"));
  else tft.print(F("RAW"));
}

void drawLineIfChanged(int16_t y, int16_t h, uint8_t size, const String &text, uint16_t color,
                       String &prevText, uint16_t &prevColor, bool force) {
  if (!force && text == prevText && color == prevColor) {
    return;
  }
  tft.fillRect(8, y, 224, h, ST77XX_BLACK);
  tft.setTextSize(size);
  tft.setTextColor(color, ST77XX_BLACK);
  tft.setCursor(8, y);
  tft.print(text);
  prevText = text;
  prevColor = color;
}

void drawMainLayout() {
  tft.fillScreen(ST77XX_BLACK);
  drawHeader();
}

void drawMainValues(bool force) {
  String ipNow = wifiIP.toString();
  static String prevL1 = "";
  static String prevL2 = "";
  static String prevL3 = "";
  static String prevL4 = "";
  static String prevL5 = "";
  static String prevL6 = "";
  static uint16_t prevC1 = ST77XX_WHITE;
  static uint16_t prevC2 = ST77XX_WHITE;
  static uint16_t prevC3 = ST77XX_WHITE;
  static uint16_t prevC4 = ST77XX_WHITE;
  static uint16_t prevC5 = ST77XX_WHITE;
  static uint16_t prevC6 = ST77XX_WHITE;

  String l1 = m.batteryPresent ? String("BATTERY CONNECTED") : String("NO BATTERY");
  String l2;
  if (!m.batteryPresent) l2 = String("");
  else if (m.chargeState == CHARGE_CHARGING) l2 = String("STATE: CHARGING");
  else if (m.chargeState == CHARGE_DISCHARGING) l2 = String("STATE: DISCHARGING");
  else l2 = String("STATE: UNKNOWN");
  String l3;
  String l4;
  if (m.batteryPresent) {
    l3 = String("SOC ") + String(m.soc, 1) + String("%   VBAT ") + String(m.vbat, 3) + String("V");
    l4 = batteryTimeRemainingLabel();
  } else {
    l3 = String("Battery: not connected");
    l4 = String("");
  }
  String l5 = String("WiFi: ") + (wifiConnected ? String("CONNECTED ") : String("RETRYING ")) + String(WIFI_STA_SSID);
  String l6 = otaLabel();

  drawLineIfChanged(34, 18, 2, l1, m.batteryPresent ? ST77XX_GREEN : ST77XX_YELLOW, prevL1, prevC1, force);
  drawLineIfChanged(58, 18, 2, l2, chargeColor(m.chargeState), prevL2, prevC2, force);
  drawLineIfChanged(84, 10, 1, l3, ST77XX_WHITE, prevL3, prevC3, force);
  drawLineIfChanged(96, 10, 1, l4, ST77XX_WHITE, prevL4, prevC4, force);
  drawLineIfChanged(108, 10, 1, l5, ST77XX_WHITE, prevL5, prevC5, force);
  drawLineIfChanged(118, 10, 1, l6, otaColor(otaState), prevL6, prevC6, force);

  prevTopology = m.topology;
  prevChargeState = m.chargeState;
  prevSoc = m.soc;
  prevVbat = m.vbat;
  prevRate = m.cRate;
  prevGaugeOnline = m.gaugeOnline;
  prevWiFiConnected = wifiConnected;
  prevIP = ipNow;
  prevOtaState = otaState;
}

void drawDebugLayout() {
  tft.fillScreen(ST77XX_BLACK);
  drawHeader();
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setCursor(8, 128);
  tft.print(F("D0 page | D1 hold OTA | D2 tap/hold mark"));
}

void drawDebugValues(bool force) {
  static String prevD1 = "";
  static String prevD2 = "";
  static String prevD3 = "";
  static String prevD4 = "";
  static String prevD5 = "";
  static String prevD6 = "";
  static String prevD7 = "";
  static String prevD8 = "";
  static uint16_t prevDC1 = ST77XX_WHITE;
  static uint16_t prevDC2 = ST77XX_WHITE;
  static uint16_t prevDC3 = ST77XX_WHITE;
  static uint16_t prevDC4 = ST77XX_WHITE;
  static uint16_t prevDC5 = ST77XX_WHITE;
  static uint16_t prevDC6 = ST77XX_WHITE;
  static uint16_t prevDC7 = ST77XX_WHITE;
  static uint16_t prevDC8 = ST77XX_WHITE;

  String d1 = String("Gauge: ") + (m.gaugeOnline ? String("ONLINE") : String("OFFLINE"));
  String d2 = String("Battery: ") + (m.batteryPresent ? String("PRESENT") : String("MISSING"));
  String d3 = String("Btns D0/D1/D2: ") +
              (d0Pressed ? String("P") : String("-")) + String("/") +
              (d1Pressed ? String("P") : String("-")) + String("/") +
              (d2Pressed ? String("P") : String("-"));
  String d4 = String("Mark: ") + String(userMarkText(userBatteryMark)) +
              String(" / ") + String(userBatteryMarkLocked ? "LOCK" : "FREE");
  String d5 = String("WiFi: ") + (wifiConnected ? String("OK ") : String("RETRY ")) + String(WIFI_STA_SSID);
  String d6 = String("IP: ") + wifiIP.toString();
  String d7 = String("Alerts: 0x") + String(lastGaugeAlerts, HEX);
  String d8 = String("OTA ") + String(otaText(otaState));
  if (wifiConnected) {
    int32_t remainMs = (int32_t)(otaWindowUntilMs - millis());
    if (otaWindowActive() && remainMs > 0) {
      d8 += String(" ARMED ") + String((remainMs + 999) / 1000) + String("s");
    } else {
      d8 += String(" LOCKED");
    }
  }

  drawLineIfChanged(32, 10, 1, d1, ST77XX_WHITE, prevD1, prevDC1, force);
  drawLineIfChanged(44, 10, 1, d2, ST77XX_WHITE, prevD2, prevDC2, force);
  drawLineIfChanged(56, 10, 1, d3, ST77XX_WHITE, prevD3, prevDC3, force);
  drawLineIfChanged(68, 10, 1, d4, ST77XX_WHITE, prevD4, prevDC4, force);
  drawLineIfChanged(80, 10, 1, d5, ST77XX_WHITE, prevD5, prevDC5, force);
  drawLineIfChanged(92, 10, 1, d6, ST77XX_WHITE, prevD6, prevDC6, force);
  drawLineIfChanged(104, 10, 1, d7, otaColor(otaState), prevD7, prevDC7, force);
  drawLineIfChanged(116, 10, 1, d8, ST77XX_WHITE, prevD8, prevDC8, force);

  prevUsbPresent = m.usbPresent;
  prevGaugeOnline = m.gaugeOnline;
  prevTopology = m.topology;
  prevChargeState = m.chargeState;
  prevWiFiConnected = wifiConnected;
  prevIP = wifiIP.toString();
  prevOtaState = otaState;
}

void drawRawLayout() {
  tft.fillScreen(ST77XX_BLACK);
  drawHeader();
}

void drawRawValues(bool force) {
  static String prevR1 = "";
  static String prevR2 = "";
  static String prevR3 = "";
  static uint16_t prevRC1 = ST77XX_WHITE;
  static uint16_t prevRC2 = ST77XX_WHITE;
  static uint16_t prevRC3 = ST77XX_WHITE;

  String r1 = String("SOC: ") + (isnan(m.soc) ? String("--") : String(m.soc, 1)) + String("%");
  String r2 = String("VBAT: ") + (isnan(m.vbat) ? String("--") : String(m.vbat, 3)) + String(" V");
  String r3 = String("cRate: ") + (isnan(m.cRate) ? String("--") : String(m.cRate, 2)) + String(" %/hr");

  drawLineIfChanged(40, 16, 2, r1, ST77XX_WHITE, prevR1, prevRC1, force);
  drawLineIfChanged(64, 16, 2, r2, ST77XX_WHITE, prevR2, prevRC2, force);
  drawLineIfChanged(88, 16, 2, r3, ST77XX_WHITE, prevR3, prevRC3, force);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_PAGE_BUTTON, INPUT_PULLUP);
  pinMode(PIN_OTA_ARM_BUTTON, INPUT_PULLDOWN);
  pinMode(PIN_EXTRA_BUTTON, INPUT_PULLDOWN);
  Wire.begin();

  if (TFT_I2C_POWER >= 0) {
    pinMode(TFT_I2C_POWER, OUTPUT);
    digitalWrite(TFT_I2C_POWER, HIGH);
  }
  if (TFT_BACKLITE >= 0) {
    pinMode(TFT_BACKLITE, OUTPUT);
    digitalWrite(TFT_BACKLITE, HIGH);
  }

  tft.init(135, 240);
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);

  m.gaugeOnline = maxlipo.begin();
  if (m.gaugeOnline) {
    // Avoid quickStart on every boot; it can temporarily skew SOC estimation.
    maxlipo.clearAlertFlag(MAX1704X_ALERTFLAG_VOLTAGE_RESET |
                           MAX1704X_ALERTFLAG_RESET_INDICATOR);
  }

  setupWiFiAndOTA();
  sampleTelemetry();
  layoutDirty = true;
}

void loop() {
  otaHttp.handleClient();
  refreshWiFiState();

  if (otaWindowArmed && !otaWindowActive()) {
    otaWindowArmed = false;
    layoutDirty = true;
  }

  readButton();
  readOtaArmButton();
  readDebugButtonD2();
  if (d2LongPressEdge) {
    userBatteryMarkLocked = !userBatteryMarkLocked;
    logSnapshot(userBatteryMarkLocked ? "D2_LOCK" : "D2_UNLOCK");
    if (currentPage == 1) layoutDirty = true;
  } else if (d2ShortPressEdge) {
    if (!userBatteryMarkLocked) {
      if (userBatteryMark == USER_MARK_UNKNOWN) userBatteryMark = USER_MARK_NO_BATTERY;
      else if (userBatteryMark == USER_MARK_NO_BATTERY) userBatteryMark = USER_MARK_BATTERY_PRESENT;
      else userBatteryMark = USER_MARK_UNKNOWN;
      logSnapshot("D2_MARK_CYCLE");
    } else {
      logSnapshot("D2_MARK_LOCKED");
    }
    if (currentPage == 1) layoutDirty = true;
  }
  if (otaBtnLongPressEdge) {
    armOtaWindow();
  }
  if (btnPressedEdge) {
    currentPage = (uint8_t)((currentPage + 1) % PAGE_COUNT);
    layoutDirty = true;
  }

  if (millis() - lastSensorMs >= 1000) {
    lastSensorMs = millis();
    sampleTelemetry();
  }

  uint32_t uiIntervalMs = (currentPage == 1) ? 250 : 100;
  if (layoutDirty || (millis() - lastUiMs >= uiIntervalMs)) {
    lastUiMs = millis();
    if (layoutDirty) {
      if (currentPage == 0) {
        drawMainLayout();
        drawMainValues(true);
      } else if (currentPage == 1) {
        drawDebugLayout();
        drawDebugValues(true);
      } else {
        drawRawLayout();
        drawRawValues(true);
      }
      layoutDirty = false;
    } else {
      if (currentPage == 0) drawMainValues(false);
      else if (currentPage == 1) drawDebugValues(false);
      else drawRawValues(false);
    }
  }
}
