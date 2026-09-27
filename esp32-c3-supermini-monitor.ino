/*
 * ESPNow DMX Monitor + Relay
 * ESP32-C3 Super Mini + 128×64 OLED (I2C 0x3C)
 *
 * Top 29 rows: scrolling RSSI bar graph (newest = right)
 * Below: 1-second averaged RSSI, packets/s, last frameId, relay state
 * Press BOOT briefly while running to toggle relay (persisted in flash).
 *
 * Libraries: ESPNowDMX (local), Adafruit SSD1306, Adafruit GFX, Preferences
 */

#include <Arduino.h>
#include <Preferences.h>
#include "ESPNowDMX_Receiver.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_WIDTH  128
#define OLED_HEIGHT  64
#define OLED_ADDR   0x3C
#define OLED_RESET  -1
#define BOOT_PIN     9

// Graph occupies the top GRAPH_H rows; each column = one 1-second sample.
#define GRAPH_H  29   // pixels tall (0 .. GRAPH_H-1)
#define GRAPH_W  OLED_WIDTH  // 128 columns = 128 seconds history

// RSSI ring buffer — 0 means "no signal" (real RSSI is always negative)
int8_t  rssiHistory[GRAPH_W];
uint8_t histHead = 0;
bool    histFull = false;

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);
ESPNowDMX_Receiver receiver;
Preferences prefs;
bool relayActive = false;

// --- stats (written from WiFi task, read from loop) ---
volatile long    rssiAccum = 0;
volatile int     rssiCount = 0;
volatile int     pktCount  = 0;
volatile uint8_t lastFrame = 0;

void dmxCallback(uint8_t universe, const uint8_t* data) {
  int8_t rssi = receiver.getLastRssi();
  if (rssi != RSSI_UNKNOWN) { rssiAccum += rssi; rssiCount++; }
  pktCount++;
  lastFrame = receiver.getLastFrameId();
}

void setRelay(bool on) {
  relayActive = on;
  receiver.enableRelay(on);
  prefs.begin("monitor", false);
  prefs.putBool("relay", on);
  prefs.end();
}

void checkBootButton() {
  static bool      prevState = false;  // false = not pressed (pin HIGH)
  static unsigned long edgeMs = 0;
  bool state = (digitalRead(BOOT_PIN) == LOW);
  if (state && !prevState)       { edgeMs = millis(); }
  else if (!state && prevState)  { if (millis() - edgeMs >= 30) setRelay(!relayActive); }
  prevState = state;
}

// Push one sample and draw the full bar graph into the current frame buffer.
// rssi == 0 signals "no data this second".
void pushAndDrawGraph(int8_t rssi) {
  rssiHistory[histHead] = rssi;
  histHead = (histHead + 1) % GRAPH_W;
  if (histHead == 0) histFull = true;

  int filled = histFull ? GRAPH_W : histHead;

  for (int xi = 0; xi < GRAPH_W; xi++) {
    int stepsBack = GRAPH_W - 1 - xi;   // 0 = newest (rightmost column)
    if (stepsBack >= filled) continue;

    int bi = ((int)histHead - 1 - stepsBack + GRAPH_W * 2) % GRAPH_W;
    int8_t r = rssiHistory[bi];
    if (r == 0) continue;               // no signal at that second

    // Map -100 dBm → 0 px, -30 dBm → GRAPH_H px (clamp at both ends)
    int h = ((int)(r + 100) * GRAPH_H) / 70;
    if (h < 1) h = 1;
    if (h > GRAPH_H) h = GRAPH_H;
    display.drawFastVLine(xi, GRAPH_H - h, h, SSD1306_WHITE);
  }
}

void setup() {
  Serial.begin(115200);

  memset(rssiHistory, 0, sizeof(rssiHistory));

  // Read BOOT pin BEFORE Wire (Wire will reclaim GPIO9 as SCL)
  pinMode(BOOT_PIN, INPUT_PULLUP);
  prefs.begin("monitor", true);
  relayActive = prefs.getBool("relay", false);
  prefs.end();

  Wire.begin();
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 not found");
    while (true) delay(1000);
  }

  // Boot splash (shown while receiver initialises)
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("ESPNow DMX Monitor");
  display.printf("Relay: %s\n", relayActive ? "ON" : "OFF");
  display.println("(BOOT = toggle)");
  display.display();

  receiver.begin();
  receiver.setDMXReceiveCallback(dmxCallback);
  receiver.enableRelay(relayActive);
}

void loop() {
  receiver.relayLoop();
  checkBootButton();

  static unsigned long lastUpdate = 0;
  unsigned long now = millis();
  if (now - lastUpdate < 1000) return;
  lastUpdate = now;

  long    accum = rssiAccum;  rssiAccum = 0;
  int     cnt   = rssiCount;  rssiCount = 0;
  int     pkts  = pktCount;   pktCount  = 0;
  uint8_t frame = lastFrame;

  bool    hasSignal = (pkts > 0 && cnt > 0);
  float   avgRssi   = hasSignal ? (float)accum / cnt : 0.0f;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // --- RSSI bar graph (top GRAPH_H rows) ---
  pushAndDrawGraph(hasSignal ? (int8_t)avgRssi : 0);

  // --- separator ---
  display.drawFastHLine(0, GRAPH_H, GRAPH_W, SSD1306_WHITE);

  // --- text area ---
  display.setTextSize(1);

  // Row 1 (Y=31): RSSI value
  display.setCursor(0, 31);
  if (!hasSignal) display.print("RSSI: --");
  else            display.printf("RSSI: %+.0f dBm", avgRssi);

  // Row 2 (Y=42): packets/s  +  frameId
  display.setCursor(0, 42);
  display.printf("Pkts/s:%-3d Fr:%u", pkts, frame);

  // Row 3 (Y=54): relay state
  display.setCursor(0, 54);
  display.print(relayActive ? "Relay: ON " : "Relay: OFF");

  display.display();
}
