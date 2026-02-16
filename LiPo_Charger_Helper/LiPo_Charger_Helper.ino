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
#define HAVE_SECRETS 1
#else
#define HAVE_SECRETS 0
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

#ifndef WIFI_STA_SSID
#define WIFI_STA_SSID ""
#endif
#ifndef WIFI_STA_PASS
#define WIFI_STA_PASS ""
#endif
#ifndef OTA_HTTP_USER
#define OTA_HTTP_USER "ota"
#endif
#ifndef OTA_HTTP_PASSWORD
#define OTA_HTTP_PASSWORD ""
#endif

#ifndef D0
#define D0 RX
#endif
#ifndef D1
#define D1 TX
#endif

Adafruit_ST7789 tft(TFT_CS, TFT_DC, TFT_RST);
Adafruit_MAX17048 maxlipo;
WebServer webServer(80);

static constexpr uint8_t BACKLIGHT_BITS = 8;
static constexpr uint8_t BACKLIGHT_DEFAULT = 96;  // 0-255

static constexpr int16_t CARD_X = 6;
static constexpr int16_t CARD_Y = 6;
static constexpr int16_t CARD_W = 228;
static constexpr int16_t CARD_H = 123;
static constexpr int16_t HEADER_H = 24;

static constexpr uint16_t COLOR_ICON = 0x9E7C;  // #99D3EE
static constexpr uint16_t COLOR_HEADER = 0x9D75;

static constexpr int BTN_CYCLE = D0;
static constexpr int BTN_WIFI = D1;
static constexpr uint16_t BUTTON_DEBOUNCE_MS = 30;
static constexpr uint32_t SAMPLE_INTERVAL_MS = 2000;
static constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;
static constexpr uint32_t WIFI_IDLE_TIMEOUT_MS = 120000;
static constexpr uint32_t OTA_UNLOCK_HOLD_MS = 2200;
static constexpr uint32_t OTA_UNLOCK_WINDOW_MS = 10UL * 60UL * 1000UL;

enum ScreenId : uint8_t {
  SCREEN_BATTERY = 0,
  SCREEN_WIFI_OTA = 1,
  SCREEN_COUNT = 2,
};

struct ButtonState {
  uint8_t pin;
  bool stableLevel;
  bool lastRead;
  uint32_t lastChangeMs;
  bool pressEvent;
  bool releaseEvent;
  bool holdFired;
  uint32_t pressStartMs;
};

static ButtonState btnCycle = {BTN_CYCLE, HIGH, HIGH, 0, false, false, false, 0};
static ButtonState btnWifi = {BTN_WIFI, HIGH, HIGH, 0, false, false, false, 0};

static bool gaugeOnline = false;
static uint32_t lastSampleMs = 0;
static uint32_t wifiStateChangeMs = 0;
static uint32_t otaUnlockUntilMs = 0;
static bool otaServerStarted = false;
static bool wifiConnectAttempted = false;

static ScreenId currentScreen = SCREEN_BATTERY;

static float soc = NAN;
static float vbat = NAN;
static float cRate = NAN;

bool isButtonPressed(const ButtonState &btn) { return btn.stableLevel == LOW; }

void updateButton(ButtonState &btn, uint32_t nowMs) {
  btn.pressEvent = false;
  btn.releaseEvent = false;

  bool raw = digitalRead(btn.pin);
  if (raw != btn.lastRead) {
    btn.lastRead = raw;
    btn.lastChangeMs = nowMs;
  }

  if ((nowMs - btn.lastChangeMs) >= BUTTON_DEBOUNCE_MS && raw != btn.stableLevel) {
    btn.stableLevel = raw;
    if (btn.stableLevel == LOW) {
      btn.pressEvent = true;
      btn.holdFired = false;
      btn.pressStartMs = nowMs;
    } else {
      btn.releaseEvent = true;
      btn.holdFired = false;
    }
  }
}

void drawLineIfChanged(int16_t y, int16_t h, uint8_t size,
                       const String &text, uint16_t color,
                       String &prevText, uint16_t &prevColor, bool force) {
  if (!force && text == prevText && color == prevColor) return;
  tft.fillRect(CARD_X + 27, y, CARD_W - 33, h, ST77XX_BLACK);
  tft.setTextSize(size);
  tft.setTextColor(color, ST77XX_BLACK);
  tft.setCursor(CARD_X + 27, y);
  tft.print(text);
  prevText = text;
  prevColor = color;
}

String valueOrPlaceholder(float value, uint8_t decimals) {
  if (isnan(value)) return String("<>");
  return String(value, static_cast<unsigned int>(decimals));
}

void drawCardBase(const __FlashStringHelper *title) {
  tft.fillScreen(ST77XX_BLACK);
  tft.fillRect(CARD_X, CARD_Y, CARD_W, CARD_H, ST77XX_BLACK);
  tft.fillRect(CARD_X, CARD_Y, CARD_W, HEADER_H, COLOR_HEADER);
  tft.setTextColor(ST77XX_BLACK, COLOR_HEADER);
  tft.setTextSize(2);
  tft.setCursor(CARD_X + 5, CARD_Y + 4);
  tft.print(title);

  tft.drawCircle(CARD_X - 2, CARD_Y + 40, 12, COLOR_ICON);
  tft.drawCircle(CARD_X - 2, CARD_Y + 40, 11, COLOR_ICON);
  tft.drawCircle(CARD_X - 2, CARD_Y + 72, 12, COLOR_ICON);
  tft.drawCircle(CARD_X - 2, CARD_Y + 72, 11, COLOR_ICON);
  tft.drawCircle(CARD_X - 2, CARD_Y + 104, 12, COLOR_ICON);
  tft.drawCircle(CARD_X - 2, CARD_Y + 104, 11, COLOR_ICON);
}

void drawBatteryLayout() { drawCardBase(F("BATTERY MONITOR")); }

void drawWifiLayout() { drawCardBase(F("WIFI / OTA")); }

void drawBatteryValues(bool force) {
  static String prevL1 = "";
  static String prevL2 = "";
  static String prevL3 = "";
  static uint16_t prevC1 = ST77XX_WHITE;
  static uint16_t prevC2 = ST77XX_WHITE;
  static uint16_t prevC3 = ST77XX_WHITE;

  String l1 = String("SOC: ") + valueOrPlaceholder(soc, 0) + String(" %");
  String l2 = String("vBat: ") + valueOrPlaceholder(vbat, 2) + String(" V");
  String l3 = String("cRate: ") + valueOrPlaceholder(cRate, 1) + String(" %/hr");

  drawLineIfChanged(CARD_Y + 34, 16, 2, l1, ST77XX_WHITE, prevL1, prevC1, force);
  drawLineIfChanged(CARD_Y + 58, 16, 2, l2, ST77XX_WHITE, prevL2, prevC2, force);
  drawLineIfChanged(CARD_Y + 82, 16, 2, l3, ST77XX_WHITE, prevL3, prevC3, force);
}

const char *wifiStatusText() {
  wl_status_t st = WiFi.status();
  if (WiFi.getMode() == WIFI_OFF) return "OFF";
  if (st == WL_CONNECTED) return "CONNECTED";
  if (st == WL_CONNECT_FAILED) return "AUTH FAIL";
  if (st == WL_NO_SSID_AVAIL) return "NO SSID";
  if (st == WL_IDLE_STATUS) return "IDLE";
  return "CONNECTING";
}

bool otaUnlocked() {
  if (otaUnlockUntilMs == 0) return false;
  return static_cast<int32_t>(otaUnlockUntilMs - millis()) > 0;
}

String otaRemainingText() {
  if (!otaUnlocked()) return String("LOCKED");
  uint32_t remainMs = otaUnlockUntilMs - millis();
  uint32_t remainSec = (remainMs + 999) / 1000;
  return String(remainSec) + String("s");
}

void drawWifiValues(bool force) {
  static String prevL1 = "";
  static String prevL2 = "";
  static String prevL3 = "";
  static uint16_t prevC1 = ST77XX_WHITE;
  static uint16_t prevC2 = ST77XX_WHITE;
  static uint16_t prevC3 = ST77XX_WHITE;

  String ip = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : String("<>");
  String l1 = String("WiFi: ") + String(wifiStatusText());
  String l2 = String("IP: ") + ip;
  String l3 = String("OTA: ") + otaRemainingText();

  uint16_t c3 = otaUnlocked() ? ST77XX_GREEN : ST77XX_YELLOW;

  drawLineIfChanged(CARD_Y + 34, 16, 2, l1, ST77XX_WHITE, prevL1, prevC1, force);
  drawLineIfChanged(CARD_Y + 58, 16, 2, l2, ST77XX_WHITE, prevL2, prevC2, force);
  drawLineIfChanged(CARD_Y + 82, 16, 2, l3, c3, prevL3, prevC3, force);
}

void renderCurrentScreen(bool force) {
  static ScreenId drawnScreen = SCREEN_COUNT;
  if (force || drawnScreen != currentScreen) {
    if (currentScreen == SCREEN_BATTERY) {
      drawBatteryLayout();
    } else {
      drawWifiLayout();
    }
    drawnScreen = currentScreen;
    force = true;
  }

  if (currentScreen == SCREEN_BATTERY) {
    drawBatteryValues(force);
  } else {
    drawWifiValues(force);
  }
}

void sampleGauge() {
  if (!gaugeOnline) {
    soc = NAN;
    vbat = NAN;
    cRate = NAN;
    return;
  }

  vbat = maxlipo.cellVoltage();
  soc = maxlipo.cellPercent();
  cRate = maxlipo.chargeRate();
}

void disableRadios() {
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
  btStop();
}

void handleOtaRoot() {
  String body;
  body.reserve(220);
  body += F("LiPo Charger Helper\n");
  body += F("WiFi: ");
  body += wifiStatusText();
  body += '\n';
  body += F("IP: ");
  body += (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : String("<>");
  body += '\n';
  body += F("OTA: ");
  body += otaUnlocked() ? F("UNLOCKED") : F("LOCKED");
  body += '\n';
  webServer.send(200, "text/plain", body);
}

bool requireAuth() {
  if (!webServer.authenticate(OTA_HTTP_USER, OTA_HTTP_PASSWORD)) {
    webServer.requestAuthentication();
    return false;
  }
  return true;
}

void handleUpdatePost() {
  if (!requireAuth()) return;
  if (!otaUnlocked()) {
    webServer.send(403, "text/plain", "OTA LOCKED");
    return;
  }
  webServer.sendHeader("Connection", "close");
  webServer.send(Update.hasError() ? 500 : 200, "text/plain",
                 Update.hasError() ? "OTA FAIL" : "OTA OK");
  delay(150);
  ESP.restart();
}

void handleUpdateUpload() {
  HTTPUpload &upload = webServer.upload();

  if (upload.status == UPLOAD_FILE_START) {
    if (!requireAuth()) {
      return;
    }
    if (!otaUnlocked()) {
      webServer.send(403, "text/plain", "OTA LOCKED");
      return;
    }
    Update.begin(UPDATE_SIZE_UNKNOWN);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (!Update.end(true)) {
      Update.printError(Serial);
    }
  }
}

void startOtaServer() {
  if (otaServerStarted) return;

  webServer.on("/", HTTP_GET, handleOtaRoot);
  webServer.on("/update", HTTP_POST, handleUpdatePost, handleUpdateUpload);
  webServer.begin();
  otaServerStarted = true;
}

void stopOtaServer() {
  if (!otaServerStarted) return;
  webServer.stop();
  otaServerStarted = false;
}

void enableWifiIfNeeded() {
  if (WiFi.getMode() == WIFI_OFF) {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(true);
    WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASS);
    wifiConnectAttempted = true;
    wifiStateChangeMs = millis();
  }
  startOtaServer();
}

void disableWifiAndOta() {
  stopOtaServer();
  otaUnlockUntilMs = 0;
  wifiConnectAttempted = false;
  disableRadios();
}

void handleButtons(uint32_t nowMs) {
  updateButton(btnCycle, nowMs);
  updateButton(btnWifi, nowMs);

  if (btnCycle.pressEvent) {
    currentScreen = static_cast<ScreenId>((static_cast<uint8_t>(currentScreen) + 1) % SCREEN_COUNT);
    renderCurrentScreen(true);
  }

  if (btnWifi.pressEvent) {
    enableWifiIfNeeded();
    wifiStateChangeMs = nowMs;
  }

  if (isButtonPressed(btnWifi) && !btnWifi.holdFired &&
      (nowMs - btnWifi.pressStartMs) >= OTA_UNLOCK_HOLD_MS) {
    btnWifi.holdFired = true;
    enableWifiIfNeeded();
    otaUnlockUntilMs = nowMs + OTA_UNLOCK_WINDOW_MS;
    wifiStateChangeMs = nowMs;
    renderCurrentScreen(true);
  }
}

void maintainWifiAndOta(uint32_t nowMs) {
  if (WiFi.getMode() == WIFI_OFF) return;

  if (otaServerStarted) {
    webServer.handleClient();
  }

  if (wifiConnectAttempted && WiFi.status() != WL_CONNECTED &&
      (nowMs - wifiStateChangeMs) > WIFI_CONNECT_TIMEOUT_MS) {
    WiFi.disconnect();
    WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASS);
    wifiStateChangeMs = nowMs;
  }

  bool wifiBtnHeld = isButtonPressed(btnWifi);
  bool keepAwake = wifiBtnHeld || otaUnlocked();
  if (!keepAwake && (nowMs - wifiStateChangeMs) >= WIFI_IDLE_TIMEOUT_MS) {
    disableWifiAndOta();
    if (currentScreen == SCREEN_WIFI_OTA) {
      renderCurrentScreen(true);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(BTN_CYCLE, INPUT_PULLUP);
  pinMode(BTN_WIFI, INPUT_PULLUP);

  disableRadios();

  if (TFT_I2C_POWER >= 0) {
    pinMode(TFT_I2C_POWER, OUTPUT);
    digitalWrite(TFT_I2C_POWER, HIGH);
  }
  if (TFT_BACKLITE >= 0) {
    ledcAttach(TFT_BACKLITE, 5000, BACKLIGHT_BITS);
    ledcWrite(TFT_BACKLITE, BACKLIGHT_DEFAULT);
  }

  tft.init(135, 240);
  tft.setRotation(3);

  gaugeOnline = maxlipo.begin();
  if (gaugeOnline) {
    maxlipo.clearAlertFlag(MAX1704X_ALERTFLAG_VOLTAGE_RESET |
                           MAX1704X_ALERTFLAG_RESET_INDICATOR);
  }

  sampleGauge();
  renderCurrentScreen(true);
}

void loop() {
  uint32_t nowMs = millis();

  handleButtons(nowMs);
  maintainWifiAndOta(nowMs);

  if (nowMs - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    lastSampleMs = nowMs;
    sampleGauge();
    renderCurrentScreen(false);
  }

  if (currentScreen == SCREEN_WIFI_OTA) {
    static uint32_t lastWifiScreenRefreshMs = 0;
    if ((nowMs - lastWifiScreenRefreshMs) >= 500) {
      lastWifiScreenRefreshMs = nowMs;
      drawWifiValues(false);
    }
  }
}
