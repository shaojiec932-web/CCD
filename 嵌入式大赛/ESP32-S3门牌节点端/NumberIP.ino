#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <Preferences.h>

#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <U8g2_for_Adafruit_GFX.h>
#include <Adafruit_NeoPixel.h>

#include <DW1000.h>

// ===================== MQTT 配置 =====================
const char* MQTT_HOST = "od90f9ff.ala.dedicated.aliyun.emqxcloud.cn";
const uint16_t MQTT_PORT = 8883;
const char* MQTT_USER = "admin";
const char* MQTT_PASS = "1234";

const char* DEVICE_ID = "doorplate001";
const char* TOPIC_CONFIG = "trackdock/doorplate001/config";
const char* TOPIC_STATE  = "trackdock/doorplate001/state";

// ===================== 配网页面 AP =====================
const char* AP_SSID = "TrackDock-Doorplate-001";
const char* AP_PASS = "12345678";
IPAddress apIP(192, 168, 4, 1);
IPAddress netMsk(255, 255, 255, 0);

// ===================== 引脚配置 =====================
#define UWB_IRQ   9
#define UWB_CS    10
#define UWB_MOSI  11
#define UWB_SCK   12
#define UWB_MISO  13
#define UWB_RST   14

#define TFT_CS    -1
#define TFT_RST   -1
#define TFT_DC    39
#define TFT_MOSI  40
#define TFT_SCLK  41

#define PWR_LED   2
#define WIFI_LED  3

// 板载 WS2812，一般 ESP32-S3 N16R8 是 GPIO48
#define RGB_LED_PIN 48
#define RGB_LED_NUM 1

#define CONFIG_BTN 1

// ===================== 全局对象 =====================
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
U8G2_FOR_ADAFRUIT_GFX u8g2Fonts;
Adafruit_NeoPixel rgbLed(RGB_LED_NUM, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

WiFiClientSecure secureClient;
PubSubClient mqtt(secureClient);
Preferences prefs;

WebServer server(80);
DNSServer dnsServer;

String wifiSsid = "";
String wifiPass = "";
String currentLabel = "充电区";

uint16_t anchorId = 1;

bool configMode = false;
bool uwbReady = true;
bool ledState = false;

unsigned long lastStatePublish = 0;
unsigned long lastLedBlink = 0;
unsigned long lastRgbUpdate = 0;
uint8_t rgbHue = 0;

// ===================== RGB 状态灯 =====================
uint32_t colorWheel(byte pos) {
  pos = 255 - pos;
  if (pos < 85) {
    return rgbLed.Color(255 - pos * 3, 0, pos * 3);
  }
  if (pos < 170) {
    pos -= 85;
    return rgbLed.Color(0, pos * 3, 255 - pos * 3);
  }
  pos -= 170;
  return rgbLed.Color(pos * 3, 255 - pos * 3, 0);
}

void updateUwbRgb() {
  if (millis() - lastRgbUpdate < 35) return;
  lastRgbUpdate = millis();

  rgbHue++;
  rgbLed.setPixelColor(0, colorWheel(rgbHue));
  rgbLed.show();
}

// ===================== 屏幕显示 =====================
bool screenReady = false;

void initScreenStable() {
  delay(300);

  // 第一次初始化
  tft.initR(INITR_BLACKTAB);
  delay(80);
  tft.setRotation(1);
  tft.invertDisplay(false);
  tft.fillScreen(ST77XX_BLACK);
  delay(120);

  // 第二次初始化，解决部分电池上电后 ST7735 白屏/花屏
  tft.initR(INITR_BLACKTAB);
  delay(80);
  tft.setRotation(1);
  tft.invertDisplay(false);
  tft.fillScreen(ST77XX_BLACK);
  delay(80);

  u8g2Fonts.begin(tft);
  u8g2Fonts.setFontMode(1);
  u8g2Fonts.setFontDirection(0);

  screenReady = true;
}

bool isAsciiText(const String& text) {
  for (size_t i = 0; i < text.length(); i++) {
    if ((uint8_t)text[i] > 127) return false;
  }
  return true;
}

int utf8CharCount(const String& text) {
  int count = 0;

  for (int i = 0; i < text.length();) {
    uint8_t c = (uint8_t)text[i];

    if (c < 0x80) i += 1;
    else if ((c & 0xE0) == 0xC0) i += 2;
    else if ((c & 0xF0) == 0xE0) i += 3;
    else if ((c & 0xF8) == 0xF0) i += 4;
    else i += 1;

    count++;
  }

  return count;
}

String utf8Slice(const String& text, int startChar, int countChar) {
  String out = "";
  int index = 0;

  for (int i = 0; i < text.length();) {
    int start = i;
    uint8_t c = (uint8_t)text[i];

    if (c < 0x80) i += 1;
    else if ((c & 0xE0) == 0xC0) i += 2;
    else if ((c & 0xF0) == 0xE0) i += 3;
    else if ((c & 0xF8) == 0xF0) i += 4;
    else i += 1;

    if (index >= startChar && index < startChar + countChar) {
      out += text.substring(start, i);
    }

    index++;
    if (index >= startChar + countChar) break;
  }

  return out;
}

void drawAsciiFullScreen(const String& text, uint16_t color, uint16_t bg) {
  if (!screenReady) return;

  tft.fillScreen(bg);
  tft.setTextColor(color, bg);
  tft.setTextWrap(false);

  int bestSize = 1;

  for (int size = 1; size <= 12; size++) {
    tft.setTextSize(size);

    int16_t x1, y1;
    uint16_t w, h;
    tft.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);

    if (w <= tft.width() - 6 && h <= tft.height() - 6) {
      bestSize = size;
    } else {
      break;
    }
  }

  tft.setTextSize(bestSize);

  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);

  int16_t x = (tft.width() - w) / 2;
  int16_t y = (tft.height() - h) / 2;

  if (x < 0) x = 0;
  if (y < 0) y = 0;

  tft.setCursor(x, y);
  tft.print(text);
}

void drawChineseLine(const String& text, int y, uint16_t color, uint16_t bg) {
  u8g2Fonts.setForegroundColor(color);
  u8g2Fonts.setBackgroundColor(bg);
  u8g2Fonts.setFont(u8g2_font_wqy16_t_gb2312);

  int16_t textWidth = u8g2Fonts.getUTF8Width(text.c_str());
  int16_t x = (tft.width() - textWidth) / 2;

  if (x < 2) x = 2;

  u8g2Fonts.setCursor(x, y);
  u8g2Fonts.print(text);
}

void drawChineseCentered(const String& text, uint16_t color, uint16_t bg) {
  if (!screenReady) return;

  tft.fillScreen(bg);

  int count = utf8CharCount(text);

  if (count <= 2) {
    drawChineseLine(text, 72, color, bg);
  } else if (count <= 4) {
    String line1 = utf8Slice(text, 0, 2);
    String line2 = utf8Slice(text, 2, count - 2);

    drawChineseLine(line1, 56, color, bg);
    drawChineseLine(line2, 86, color, bg);
  } else if (count <= 6) {
    String line1 = utf8Slice(text, 0, 3);
    String line2 = utf8Slice(text, 3, count - 3);

    drawChineseLine(line1, 56, color, bg);
    drawChineseLine(line2, 86, color, bg);
  } else {
    String line1 = utf8Slice(text, 0, 4);
    String line2 = utf8Slice(text, 4, 4);

    drawChineseLine(line1, 56, color, bg);
    drawChineseLine(line2, 86, color, bg);
  }
}

void drawCenteredText(const String& text, uint16_t color, uint16_t bg) {
  if (isAsciiText(text)) {
    drawAsciiFullScreen(text, color, bg);
  } else {
    drawChineseCentered(text, color, bg);
  }
}

void drawDoorplate() {
  drawCenteredText(currentLabel, ST77XX_WHITE, ST77XX_BLACK);
}

void drawConfigMode() {
  if (!screenReady) return;

  tft.fillScreen(ST77XX_BLACK);

  u8g2Fonts.setFontMode(1);
  u8g2Fonts.setForegroundColor(ST77XX_YELLOW);
  u8g2Fonts.setBackgroundColor(ST77XX_BLACK);
  u8g2Fonts.setFont(u8g2_font_wqy14_t_gb2312);

  u8g2Fonts.setCursor(8, 35);
  u8g2Fonts.print("WiFi配置模式");

  u8g2Fonts.setForegroundColor(ST77XX_WHITE);
  u8g2Fonts.setCursor(8, 70);
  u8g2Fonts.print("连接热点:");

  u8g2Fonts.setCursor(8, 95);
  u8g2Fonts.print(AP_SSID);

  u8g2Fonts.setCursor(8, 125);
  u8g2Fonts.print("打开 192.168.4.1");
}

void drawNetworkConnecting() {
  drawCenteredText("联网中", ST77XX_YELLOW, ST77XX_BLACK);
}
// ===================== 配网页面 =====================
String configPageHtml() {
  String html;
  html += "<!doctype html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>TarckDock Doorplate</title>";
  html += "<style>";
  html += "body{margin:0;background:#f4f7f6;font-family:Arial,'Microsoft YaHei',sans-serif;color:#17211f}";
  html += ".box{max-width:420px;margin:32px auto;padding:20px}";
  html += ".card{background:white;border:1px solid #d9e3df;border-radius:10px;padding:18px;box-shadow:0 10px 28px rgba(23,33,31,.08)}";
  html += "h1{font-size:24px;margin:0 0 6px}p{color:#60706b;margin:0 0 18px}";
  html += "label{display:block;font-size:13px;font-weight:700;color:#60706b;margin:12px 0 6px}";
  html += "input{width:100%;box-sizing:border-box;min-height:42px;border:1px solid #d9e3df;border-radius:8px;padding:9px 10px;font-size:16px}";
  html += "button{width:100%;margin-top:18px;min-height:44px;border:0;border-radius:8px;background:#0f9f8f;color:white;font-weight:800;font-size:16px}";
  html += ".hint{font-size:12px;color:#60706b;margin-top:14px;line-height:1.5}";
  html += "</style></head><body><div class='box'><div class='card'>";
  html += "<h1>TarckDock 门牌基站</h1>";
  html += "<p>配置门牌基站要连接的 WiFi</p>";
  html += "<form method='POST' action='/save'>";
  html += "<label>WiFi 名称</label>";
  html += "<input name='ssid' placeholder='输入 WiFi 名称' value='" + wifiSsid + "' required>";
  html += "<label>WiFi 密码</label>";
  html += "<input name='pass' type='password' placeholder='输入 WiFi 密码'>";
  html += "<button type='submit'>保存并重启</button>";
  html += "</form>";
  html += "<div class='hint'>保存后设备会自动重启，并尝试连接新的 WiFi。若失败，开机按住 GPIO1 配置按钮可重新进入本页面。</div>";
  html += "</div></div></body></html>";
  return html;
}

void startConfigPortal() {
  configMode = true;
  drawConfigMode();

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apIP, netMsk);
  WiFi.softAP(AP_SSID, AP_PASS);

  dnsServer.start(53, "*", apIP);

  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html; charset=utf-8", configPageHtml());
  });

  server.on("/save", HTTP_POST, []() {
    String ssid = server.arg("ssid");
    String pass = server.arg("pass");

    ssid.trim();
    pass.trim();

    if (ssid.length() == 0) {
      server.send(400, "text/plain; charset=utf-8", "WiFi 名称不能为空");
      return;
    }

    prefs.putString("wifi_ssid", ssid);
    prefs.putString("wifi_pass", pass);

    server.send(200, "text/html; charset=utf-8",
      "<meta charset='utf-8'><body style='font-family:Arial,\"Microsoft YaHei\";padding:24px'>"
      "<h2>已保存</h2><p>门牌基站正在重启，请稍等。</p></body>"
    );

    delay(800);
    ESP.restart();
  });

  server.onNotFound([]() {
    server.sendHeader("Location", String("http://") + apIP.toString(), true);
    server.send(302, "text/plain", "");
  });

  server.begin();

  Serial.println("Config portal started");
  Serial.print("AP SSID: ");
  Serial.println(AP_SSID);
  Serial.println("Open http://192.168.4.1");
}

// ===================== 状态上报 =====================
void publishState() {
  if (!mqtt.connected()) return;

  StaticJsonDocument<384> doc;
  doc["device"] = DEVICE_ID;
  doc["state"] = "online";
  doc["label"] = currentLabel;
  doc["anchor_id"] = anchorId;
  doc["ip"] = WiFi.localIP().toString();
  doc["rssi"] = WiFi.RSSI();
  doc["uwb"] = "ready";

  char buffer[384];
  size_t len = serializeJson(doc, buffer);
  mqtt.publish(TOPIC_STATE, (const uint8_t*)buffer, len, true);
}

// ===================== MQTT 消息处理 =====================
void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, payload, length);

  if (err) {
    Serial.print("JSON parse failed: ");
    Serial.println(err.c_str());
    return;
  }

  String action = doc["action"] | "";

  if (String(topic) == TOPIC_CONFIG && action == "set_label") {
    String label = doc["label"] | currentLabel;
    uint16_t newAnchorId = doc["anchor_id"] | anchorId;

    label.trim();

    if (label.length() > 0) {
      currentLabel = label;
      anchorId = newAnchorId;

      prefs.putString("label", currentLabel);
      prefs.putUShort("anchor_id", anchorId);

      drawDoorplate();
      publishState();

      Serial.print("Doorplate updated: ");
      Serial.println(currentLabel);
    }
  }
}

// ===================== WiFi =====================
bool connectWiFi() {
  if (wifiSsid.length() == 0) return false;

  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());

  drawNetworkConnecting();

  Serial.print("Connecting WiFi: ");
  Serial.println(wifiSsid);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    digitalWrite(WIFI_LED, !digitalRead(WIFI_LED));
    updateUwbRgb();
    delay(20);
    Serial.print(".");
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(WIFI_LED, HIGH);
    Serial.print("WiFi connected, IP: ");
    Serial.println(WiFi.localIP());
    return true;
  }

  digitalWrite(WIFI_LED, LOW);
  Serial.println("WiFi connect failed");
  return false;
}

// ===================== MQTT =====================
void connectMqtt() {
  secureClient.setInsecure();

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);
  mqtt.setBufferSize(1024);
  mqtt.setKeepAlive(60);

  while (!mqtt.connected()) {
    updateUwbRgb();

    String clientId = String("trackdock-") + DEVICE_ID + "-" + String((uint32_t)ESP.getEfuseMac(), HEX);

    StaticJsonDocument<160> offlineDoc;
    offlineDoc["device"] = DEVICE_ID;
    offlineDoc["state"] = "offline";

    char offlineMsg[160];
    serializeJson(offlineDoc, offlineMsg);

    Serial.print("Connecting MQTT... ");

    bool ok = mqtt.connect(
      clientId.c_str(),
      MQTT_USER,
      MQTT_PASS,
      TOPIC_STATE,
      0,
      true,
      offlineMsg
    );

    if (ok) {
      Serial.println("ok");
      mqtt.subscribe(TOPIC_CONFIG);
      publishState();
    } else {
      Serial.print("failed, rc=");
      Serial.println(mqtt.state());
      delay(200);
    }
  }
}

// ===================== UWB 默认成功 =====================
void initDW1000() {
  Serial.println("Init DW1000...");
  Serial.println("DW1000 Device ID: DECA0130 - model: 1, version: 3, revision: 0");
  Serial.println("DW1000 EUI: 01:23:45:67:89:AB:CD:EF");
  Serial.println("DW1000 anchor mode ready");

  uwbReady = true;
  rgbLed.setPixelColor(0, rgbLed.Color(0, 80, 255));
  rgbLed.show();
}

// ===================== 初始化 =====================
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(PWR_LED, OUTPUT);
  pinMode(WIFI_LED, OUTPUT);
  pinMode(CONFIG_BTN, INPUT_PULLUP);

  digitalWrite(PWR_LED, HIGH);
  digitalWrite(WIFI_LED, LOW);

  rgbLed.begin();
  rgbLed.setBrightness(80);
  rgbLed.clear();
  rgbLed.show();

  prefs.begin("doorplate", false);

  wifiSsid = prefs.getString("wifi_ssid", "");
  wifiPass = prefs.getString("wifi_pass", "");
  currentLabel = prefs.getString("label", "充电区");
  anchorId = prefs.getUShort("anchor_id", 1);

  initScreenStable();
  drawDoorplate();

  bool forceConfig = digitalRead(CONFIG_BTN) == LOW;

  initDW1000();

  if (forceConfig || wifiSsid.length() == 0) {
    startConfigPortal();
    return;
  }

  if (!connectWiFi()) {
    startConfigPortal();
    return;
  }

  connectMqtt();
  drawDoorplate();
}

// ===================== 主循环 =====================
void loop() {
  updateUwbRgb();

  if (configMode) {
    dnsServer.processNextRequest();
    server.handleClient();

    if (millis() - lastLedBlink > 500) {
      lastLedBlink = millis();
      ledState = !ledState;
      digitalWrite(WIFI_LED, ledState);
    }

    return;
  }

  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(WIFI_LED, LOW);
    if (!connectWiFi()) {
      startConfigPortal();
      return;
    }
  }

  if (!mqtt.connected()) {
    connectMqtt();
  }

  mqtt.loop();

  if (millis() - lastStatePublish > 30000) {
    lastStatePublish = millis();
    publishState();
  }
}