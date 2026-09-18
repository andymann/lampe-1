// Lampe 1 - Handsender: standalone ESPNowDMX sender
// 11 buttons + rotary encoder + SH1107 OLED (128x128, I2C)
// Libraries: "Adafruit SH110X", "Adafruit GFX Library", "ESPNowDMX" (andymann/ESPNowDMX)

// Lampe 1 - Handsender: standalone ESPNowDMX sender
// 11 buttons + rotary encoder + SH1107 OLED (128x128, I2C)
// Libraries: "Adafruit SH110X", "Adafruit GFX Library", "ESPNowDMX" (andymann/ESPNowDMX)
//
// ---------- Wiring ----------
// Buttons (11x, momentary, wired to GND, internal pull-up):
//   Button 1  -> GPIO4    (program select 1: Red Ora)
//   Button 2  -> GPIO16   (program select 2: Blue Ora)
//   Button 3  -> GPIO17   (program select 3: Bunt)
//   Button 4  -> GPIO5    (program select 4: Blu Whi)
//   Button 5  -> GPIO18   (unused)
//   Button 6  -> GPIO19   (unused)
//   Button 7  -> GPIO23   (unused)
//   Button 8  -> GPIO13   (program select 8: FIX)
//   Button 9  -> GPIO14   (turn off active program)
//   Button 10 -> GPIO27   (tap tempo)
//   Button 11 -> GPIO26   (reserved, no function yet)
//
// Rotary encoder:
//   CLK -> GPIO33
//   DT  -> GPIO32
//   SW  -> GPIO25 (push button, internal pull-up)
//   Turn while released = adjust dimmer (0-100%)
//   Turn while held down = adjust tempo (5-200 BPM)
//
// I2C OLED display (SH1107, 128x128, e.g. Waveshare 1.5" MC01506):
//   SDA -> GPIO21
//   SCL -> GPIO22
//   I2C address: 0x3C
//
// DMX fixtures (via ESPNowDMX_Sender, 6 fixtures x 8 channels each):
//   Fixture 1 -> base address 10 (channels 10-17)
//   Fixture 2 -> base address 20 (channels 20-27)
//   Fixture 3 -> base address 30 (channels 30-37)
//   Fixture 4 -> base address 40 (channels 40-47)
//   Fixture 5 -> base address 50 (channels 50-57)
//   Fixture 6 -> base address 60 (channels 60-67)
//   Per-fixture channel layout: offset 0=DIMM, 1=R, 2=G, 3=B, 4=W, 5-7=unused


#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include "ESPNowDMX_Sender.h"
#include "Color.h"

const char* VERSION = "v0.59";

// ---------- Display ----------
#define OLED_SDA 21
#define OLED_SCL 22
#define OLED_ADDR 0x3C
Adafruit_SH1107 display(128, 128, &Wire, -1);

// ---------- ESP-NOW DMX sender ----------
ESPNowDMX_Sender sender;

// ---------- Fixtures ----------
// 6 fixtures, 8 DMX channels each, base addresses 10/20/30/40/50/60
const uint16_t fixtureBase[6] = {10, 20, 30, 40, 50, 60};
const uint8_t NUM_FIXTURES = 6;
const uint8_t CHANNELS_PER_FIXTURE = 8;
uint8_t TEMPO_DIVIDER = 4; // step advances every N beats instead of every beat - set per program

enum FixtureChannel {
  CH_DIMM = 0,
  CH_R    = 1,
  CH_G    = 2,
  CH_B    = 3,
  CH_W    = 4
  // offsets 5,6,7 unused/unnamed for now
};

void setFixtureChannel(uint8_t fixtureNum, FixtureChannel channel, uint8_t value) {
  // fixtureNum: 1-6
  sender.setChannel(fixtureBase[fixtureNum - 1] + channel, value);
}

void setFixtureRGBW(uint8_t fixtureNum, uint8_t dimm, uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
  setFixtureChannel(fixtureNum, CH_DIMM, dimm);
  setFixtureChannel(fixtureNum, CH_R, r);
  setFixtureChannel(fixtureNum, CH_G, g);
  setFixtureChannel(fixtureNum, CH_B, b);
  setFixtureChannel(fixtureNum, CH_W, w);
}

void clearAllFixtures() {
  for (uint8_t f = 0; f < NUM_FIXTURES; f++) {
    for (uint8_t c = 0; c < CHANNELS_PER_FIXTURE; c++) {
      sender.setChannel(fixtureBase[f] + c, 0);
    }
  }
}

void applyColorToFixture(uint8_t fixtureNum, Color c, float dimmerScale) {
  setFixtureChannel(fixtureNum, CH_R, (uint8_t)round(c.r * dimmerScale));
  setFixtureChannel(fixtureNum, CH_G, (uint8_t)round(c.g * dimmerScale));
  setFixtureChannel(fixtureNum, CH_B, (uint8_t)round(c.b * dimmerScale));
  setFixtureChannel(fixtureNum, CH_W, (uint8_t)round(c.w * dimmerScale));
}

// ---------- Program existence / capabilities ----------
const bool programExists[9] = {false, true, true, true, true, false, false, false, true};
// index:                        0      1     2     3     4     5      6      7     8
// programs 5, 6, 7 have no function; 8 = FIX (button-advanced, no Step/Fade)

const char* programNames[9] = {
  "Off", "Red Ora", "Blue Ora", "Bunt", "Blu Whi", "-", "-", "-", "FIX"
};

uint8_t fixStepIndex = 0; // which of the 6 FIX colors is currently shown

// ---------- Buttons ----------
const uint8_t NUM_BUTTONS = 11;
const uint8_t buttonPins[NUM_BUTTONS] = {4, 16, 17, 5, 18, 19, 23, 13, 14, 27, 26};
// index:                                 0   1   2  3   4   5   6   7  8   9  10
// button #:                              1   2   3  4   5   6   7   8  9  10  11
bool buttonState[NUM_BUTTONS];
bool lastButtonState[NUM_BUTTONS];
unsigned long lastDebounceTime[NUM_BUTTONS];
const unsigned long debounceDelay = 20;

const uint8_t BTN_OFF_INDEX = 8;       // button 9 -> program off
const uint8_t BTN_TAPTEMPO_INDEX = 9;  // button 10 -> tap tempo
// index 10 (button 11) currently unused/reserved

// ---------- Rotary encoder ----------
#define ENC_CLK 33
#define ENC_DT  32
#define ENC_SW  25

volatile long encoderRaw = 0;
long lastEncoderRaw = 0;
int lastCLK;
bool swState = false;       // debounced, true = pressed
bool lastSwReading = HIGH;
unsigned long lastSwDebounce = 0;

// ---------- Dimmer / Tempo ----------
int dimmerValue = 100;      // 0-100 %
int tempoBpm = 120;         // 5-200 BPM
const int TEMPO_MIN = 5;
const int TEMPO_MAX = 200;

// ---------- Tap tempo ----------
const uint8_t TAP_BUFFER = 8;
unsigned long tapTimes[TAP_BUFFER];
uint8_t tapCount = 0;
unsigned long lastTapTime = 0;
const unsigned long TAP_TIMEOUT = 2500; // ms; resets the tap sequence if exceeded

// ---------- Program state ----------
enum ProgramMode { MODE_STEP, MODE_FADE };
uint8_t activeProgram = 0;   // 0 = off, 1-8 = program number
ProgramMode currentMode = MODE_STEP;

// ---------- Program timing ----------
const unsigned long PROGRAM_UPDATE_INTERVAL = 30; // ms -> ~33 Hz output rate
unsigned long lastProgramUpdate = 0;

void IRAM_ATTR readEncoderISR() {
  int clkState = digitalRead(ENC_CLK);
  int dtState = digitalRead(ENC_DT);
  if (clkState != lastCLK) {
    if (dtState != clkState) {
      encoderRaw++;
    } else {
      encoderRaw--;
    }
  }
  lastCLK = clkState;
}

void setup() {
  Serial.begin(115200);

  for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
    buttonState[i] = HIGH;
    lastButtonState[i] = HIGH;
  }

  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  lastCLK = digitalRead(ENC_CLK);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK), readEncoderISR, CHANGE);

  Wire.begin(OLED_SDA, OLED_SCL);
  display.begin(OLED_ADDR, true);

  showBootScreen();   // clears the display, shows the splash for 2s

  sender.begin();     // initializes ESP-NOW internally

  drawDisplay();      // switch to normal operation display
}

void loop() {
  bool needsRedraw = false;

  // --- Encoder switch (debounced) ---
  bool swReading = digitalRead(ENC_SW);
  if (swReading != lastSwReading) {
    lastSwDebounce = millis();
  }
  if ((millis() - lastSwDebounce) > debounceDelay) {
    swState = (swReading == LOW);
  }
  lastSwReading = swReading;

  // --- Encoder movement: dimmer if released, tempo if switch is held ---
  long currentRaw = encoderRaw;
  long delta = currentRaw - lastEncoderRaw;
  if (delta != 0) {
    lastEncoderRaw = currentRaw;
    if (swState) {
      tempoBpm = constrain(tempoBpm + (int)delta, TEMPO_MIN, TEMPO_MAX);
    } else {
      dimmerValue = constrain(dimmerValue + (int)delta, 0, 100);
    }
    needsRedraw = true;
  }

  // --- Buttons ---
  for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
    bool reading = digitalRead(buttonPins[i]);
    if (reading != lastButtonState[i]) {
      lastDebounceTime[i] = millis();
    }
    if ((millis() - lastDebounceTime[i]) > debounceDelay) {
      if (reading != buttonState[i]) {
        buttonState[i] = reading;
        if (buttonState[i] == LOW) { // pressed (falling edge)
          handleButtonPress(i);
          needsRedraw = true;
        }
      }
    }
    lastButtonState[i] = reading;
  }

  if (needsRedraw) {
    drawDisplay();
  }

  if (millis() - lastProgramUpdate >= PROGRAM_UPDATE_INTERVAL) {
    lastProgramUpdate = millis();
    runActiveProgram();
  }

  sender.loop();
}

void handleButtonPress(uint8_t index) {
  // Program select buttons: index 0-7 -> program 1-8
  if (index <= 7) {
    uint8_t pressedProgram = index + 1;
    if (!programExists[pressedProgram]) {
      return; // button has no function
    }

    if (activeProgram == 0) {
      clearAllFixtures();
      activeProgram = pressedProgram;
      currentMode = MODE_STEP;
      if (pressedProgram == 8) fixStepIndex = 0;
    } else if (activeProgram == pressedProgram) {
      if (pressedProgram == 8) {
        fixStepIndex = (fixStepIndex + 1) % 6; // manual advance, no auto-timing
      } else {
        currentMode = (currentMode == MODE_STEP) ? MODE_FADE : MODE_STEP;
      }
    } else {
      clearAllFixtures();
      activeProgram = pressedProgram;
      currentMode = MODE_STEP;
      if (pressedProgram == 8) fixStepIndex = 0;
    }
    return;
  }

  if (index == BTN_OFF_INDEX) {
    activeProgram = 0;
    clearAllFixtures();
    return;
  }

  if (index == BTN_TAPTEMPO_INDEX) {
    registerTap();
    return;
  }

  // index 10 (button 11): reserved, no action yet
}

void registerTap() {
  unsigned long now = millis();
  if (now - lastTapTime > TAP_TIMEOUT) {
    tapCount = 0; // sequence timed out, start fresh
  }
  lastTapTime = now;

  if (tapCount < TAP_BUFFER) {
    tapTimes[tapCount] = now;
    tapCount++;
  } else {
    for (uint8_t i = 1; i < TAP_BUFFER; i++) {
      tapTimes[i - 1] = tapTimes[i];
    }
    tapTimes[TAP_BUFFER - 1] = now;
  }

  if (tapCount >= 4) {
    unsigned long sumIntervals = 0;
    for (uint8_t i = 1; i < tapCount; i++) {
      sumIntervals += (tapTimes[i] - tapTimes[i - 1]);
    }
    float avgInterval = (float)sumIntervals / (tapCount - 1);
    int newBpm = round(60000.0 / avgInterval);
    tempoBpm = constrain(newBpm, TEMPO_MIN, TEMPO_MAX);
  }
}

// ---------- Display ----------

uint8_t fitTextSize(String text, uint16_t maxWidth, uint8_t maxSize) {
  for (uint8_t size = maxSize; size >= 1; size--) {
    display.setTextSize(size);
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
    if (w <= maxWidth) return size;
  }
  return 1;
}

void printCentered(int y, String text, uint8_t textSize) {
  display.setTextSize(textSize);
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((128 - w) / 2, y);
  display.print(text);
}

void showBootScreen() {
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);

  String lines[3] = {"Lampe 1", "Handsender", String(VERSION)};
  uint8_t sizes[3];
  uint16_t heights[3];

  for (uint8_t i = 0; i < 3; i++) {
    sizes[i] = fitTextSize(lines[i], 120, 2); // capped at size 2
    display.setTextSize(sizes[i]);
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(lines[i], 0, 0, &x1, &y1, &w, &h);
    heights[i] = h;
  }

  const uint16_t gap = 6;
  uint16_t totalHeight = heights[0] + heights[1] + heights[2] + 2 * gap;
  int16_t y = (128 - totalHeight) / 2;

  for (uint8_t i = 0; i < 3; i++) {
    printCentered(y, lines[i], sizes[i]);
    y += heights[i] + gap;
  }

  display.display();
  delay(2000);
}

void drawDisplay() {
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);

  printCentered(4,  "Dimmer", 2);
  printCentered(24, String(dimmerValue) + "%", 2);

  display.drawFastHLine(8, 44, 112, SH110X_WHITE);

  printCentered(48, "Tempo", 2);
  printCentered(68, String(tempoBpm) + "BPM", 2);

  display.drawFastHLine(8, 88, 112, SH110X_WHITE);

  if (activeProgram == 0 || activeProgram == 8) {
    printCentered(100, programNames[activeProgram], 2);
  } else {
    printCentered(92, programNames[activeProgram], 2);
    printCentered(112, currentMode == MODE_STEP ? "[Step]" : "[Fade]", 2);
  }

  display.display();
}

// ---------- Program output ----------

unsigned long beatMs() {
  return 60000UL / tempoBpm;
}

unsigned long stepDurationMs() {
  return beatMs() * TEMPO_DIVIDER;
}

void runActiveProgram() {
  switch (activeProgram) {
    case 0:
      break;
    case 1: programRedOra();  break;
    case 2: programBlueOra(); break;
    case 3: programBunt();    break;
    case 4: programBluWhi();  break;
    case 8: programFix();     break;
    // 5, 6, 7 unused
  }
}

void programRedOra() {
  TEMPO_DIVIDER = 4;
  uint8_t brightness = map(dimmerValue, 0, 100, 0, 255);
  float dimmerScale = dimmerValue / 100.0;

  for (uint8_t f = 1; f <= 6; f++) {
    setFixtureChannel(f, CH_DIMM, brightness);
    setFixtureChannel(f, CH_R, brightness);
  }

  const uint8_t groupA[] = {1, 4};
  const uint8_t groupB[] = {2, 5};
  const uint8_t groupC[] = {3, 6};

  const uint8_t stepValuesA[3] = {134, 0,   0};
  const uint8_t stepValuesB[3] = {0,   134, 0};
  const uint8_t stepValuesC[3] = {0,   0,   134};

  unsigned long duration = stepDurationMs();
  uint8_t step = (millis() / duration) % 3;

  auto applyGroup = [&](const uint8_t* fixtures, const uint8_t* values, uint8_t s) {
    uint8_t val = (uint8_t)round(values[s] * dimmerScale);
    for (uint8_t i = 0; i < 2; i++) setFixtureChannel(fixtures[i], CH_G, val);
  };

  if (currentMode == MODE_STEP) {
    applyGroup(groupA, stepValuesA, step);
    applyGroup(groupB, stepValuesB, step);
    applyGroup(groupC, stepValuesC, step);
  } else {
    uint8_t nextStep = (step + 1) % 3;
    float t = (float)(millis() % duration) / duration;

    auto applyFade = [&](const uint8_t* fixtures, const uint8_t* values) {
      float v = values[step] + t * ((float)values[nextStep] - values[step]);
      uint8_t val = (uint8_t)round(v * dimmerScale);
      for (uint8_t i = 0; i < 2; i++) setFixtureChannel(fixtures[i], CH_G, val);
    };

    applyFade(groupA, stepValuesA);
    applyFade(groupB, stepValuesB);
    applyFade(groupC, stepValuesC);
  }
}

void programBlueOra() {
  TEMPO_DIVIDER = 1;
  uint8_t brightness = map(dimmerValue, 0, 100, 0, 255);
  float dimmerScale = dimmerValue / 100.0;

  for (uint8_t f = 1; f <= 6; f++) {
    setFixtureChannel(f, CH_DIMM, brightness);
  }

  const uint8_t oddFixtures[]  = {1, 3, 5};
  const uint8_t evenFixtures[] = {2, 4, 6};

  const uint8_t stateA[3] = {255, 140, 0}; // R, G, B
  const uint8_t stateB[3] = {0,   0,   255};

  unsigned long duration = stepDurationMs();
  uint8_t step = (millis() / duration) % 2;

  auto applyState = [&](const uint8_t* fixtures, const uint8_t* state) {
    for (uint8_t i = 0; i < 3; i++) {
      setFixtureChannel(fixtures[i], CH_R, (uint8_t)round(state[0] * dimmerScale));
      setFixtureChannel(fixtures[i], CH_G, (uint8_t)round(state[1] * dimmerScale));
      setFixtureChannel(fixtures[i], CH_B, (uint8_t)round(state[2] * dimmerScale));
    }
  };

  if (currentMode == MODE_STEP) {
    applyState(oddFixtures,  (step == 0) ? stateA : stateB);
    applyState(evenFixtures, (step == 0) ? stateB : stateA);
  } else {
    uint8_t nextStep = (step + 1) % 2;
    const uint8_t* oddFrom  = (step == 0) ? stateA : stateB;
    const uint8_t* oddTo    = (nextStep == 0) ? stateA : stateB;
    const uint8_t* evenFrom = (step == 0) ? stateB : stateA;
    const uint8_t* evenTo   = (nextStep == 0) ? stateB : stateA;
    float t = (float)(millis() % duration) / duration;

    uint8_t oddVals[3], evenVals[3];
    for (uint8_t c = 0; c < 3; c++) {
      oddVals[c]  = (uint8_t)round(oddFrom[c]  + t * ((float)oddTo[c]  - oddFrom[c]));
      evenVals[c] = (uint8_t)round(evenFrom[c] + t * ((float)evenTo[c] - evenFrom[c]));
    }

    for (uint8_t i = 0; i < 3; i++) {
      setFixtureChannel(oddFixtures[i],  CH_R, (uint8_t)round(oddVals[0]  * dimmerScale));
      setFixtureChannel(oddFixtures[i],  CH_G, (uint8_t)round(oddVals[1]  * dimmerScale));
      setFixtureChannel(oddFixtures[i],  CH_B, (uint8_t)round(oddVals[2]  * dimmerScale));
      setFixtureChannel(evenFixtures[i], CH_R, (uint8_t)round(evenVals[0] * dimmerScale));
      setFixtureChannel(evenFixtures[i], CH_G, (uint8_t)round(evenVals[1] * dimmerScale));
      setFixtureChannel(evenFixtures[i], CH_B, (uint8_t)round(evenVals[2] * dimmerScale));
    }
  }
}

void programBunt() {
  TEMPO_DIVIDER = 4;
  uint8_t brightness = map(dimmerValue, 0, 100, 0, 255);
  float dimmerScale = dimmerValue / 100.0;

  for (uint8_t f = 1; f <= 6; f++) {
    setFixtureChannel(f, CH_DIMM, brightness);
  }

  // Step 1 assignment: fixture 1-6 = RED, GREEN, BLUE, ORA, YELLOW, VIO
  const Color stepColors[6] = {COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_ORA, COLOR_YELLOW, COLOR_VIO};

  unsigned long duration = stepDurationMs();
  uint8_t step = (millis() / duration) % 6;

  if (currentMode == MODE_STEP) {
    for (uint8_t f = 0; f < 6; f++) {
      Color c = stepColors[(f + 6 - step) % 6];
      applyColorToFixture(f + 1, c, dimmerScale);
    }
  } else {
    uint8_t nextStep = (step + 1) % 6;
    float t = (float)(millis() % duration) / duration;

    for (uint8_t f = 0; f < 6; f++) {
      Color from = stepColors[(f + 6 - step) % 6];
      Color to   = stepColors[(f + 6 - nextStep) % 6];

      uint8_t r = (uint8_t)round((from.r + t * ((float)to.r - from.r)) * dimmerScale);
      uint8_t g = (uint8_t)round((from.g + t * ((float)to.g - from.g)) * dimmerScale);
      uint8_t b = (uint8_t)round((from.b + t * ((float)to.b - from.b)) * dimmerScale);
      uint8_t w = (uint8_t)round((from.w + t * ((float)to.w - from.w)) * dimmerScale);

      setFixtureChannel(f + 1, CH_R, r);
      setFixtureChannel(f + 1, CH_G, g);
      setFixtureChannel(f + 1, CH_B, b);
      setFixtureChannel(f + 1, CH_W, w);
    }
  }
}

void programBluWhi() {
  TEMPO_DIVIDER = 4;
  uint8_t brightness = map(dimmerValue, 0, 100, 0, 255);
  float dimmerScale = dimmerValue / 100.0;

  for (uint8_t f = 1; f <= 6; f++) {
    setFixtureChannel(f, CH_DIMM, brightness);
  }

  const uint8_t oddFixtures[]  = {1, 3, 5};
  const uint8_t evenFixtures[] = {2, 4, 6};

  // Step 1: odd=blue, even=white. Step 2: odd=white, even=blue.
  const Color stateA[2] = {COLOR_BLUE, COLOR_WHITE};  // odd, even
  const Color stateB[2] = {COLOR_WHITE, COLOR_BLUE};

  unsigned long duration = stepDurationMs();
  uint8_t step = (millis() / duration) % 2;

  auto applyState = [&](const uint8_t* fixtures, Color c) {
    for (uint8_t i = 0; i < 3; i++) applyColorToFixture(fixtures[i], c, dimmerScale);
  };

  if (currentMode == MODE_STEP) {
    const Color* state = (step == 0) ? stateA : stateB;
    applyState(oddFixtures, state[0]);
    applyState(evenFixtures, state[1]);
  } else {
    uint8_t nextStep = (step + 1) % 2;
    const Color* from = (step == 0) ? stateA : stateB;
    const Color* to   = (nextStep == 0) ? stateA : stateB;
    float t = (float)(millis() % duration) / duration;

    auto fadeColor = [&](Color a, Color b) -> Color {
      Color c;
      c.r = (uint8_t)round(a.r + t * ((float)b.r - a.r));
      c.g = (uint8_t)round(a.g + t * ((float)b.g - a.g));
      c.b = (uint8_t)round(a.b + t * ((float)b.b - a.b));
      c.w = (uint8_t)round(a.w + t * ((float)b.w - a.w));
      return c;
    };

    applyState(oddFixtures, fadeColor(from[0], to[0]));
    applyState(evenFixtures, fadeColor(from[1], to[1]));
  }
}

const Color fixSteps[6] = {COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_WHITE, COLOR_VIO, COLOR_ORA};

void programFix() {
  uint8_t brightness = map(dimmerValue, 0, 100, 0, 255);
  float dimmerScale = dimmerValue / 100.0;

  for (uint8_t f = 1; f <= 6; f++) {
    setFixtureChannel(f, CH_DIMM, brightness);
    applyColorToFixture(f, fixSteps[fixStepIndex], dimmerScale);
  }
}
