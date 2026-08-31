/*
 * DMX receiver -> forwards channels 1-6 out via ESP-NOW using the
 * ESPNowDMX_Sender library.
 *
 * Target board: ESP32-C3 Super Mini (single-core RISC-V). Unlike the
 * ESP32-WROOM-32 version of this sketch, there's only one CPU core
 * here, so DMX reading and ESP-NOW sending can't run on separate
 * cores -- but DMX reading still runs in its own dedicated FreeRTOS
 * task rather than being polled inline from loop(), and that task's
 * priority is kept moderate (5): high enough to service the DMX
 * stream promptly, but not so high it starves loop()'s sender.loop()
 * call (the actual ESP-NOW radio transmission) of CPU time -- a
 * higher priority was tried earlier in this project and starved
 * transmission almost entirely on this chip.
 *
 * DMX source: QLC+ output via the Enttec-Open-DMX-style libftdi path.
 * Confirmed from QLC+'s own source (plugins/dmxusb/src/
 * enttecdmxusbopen.cpp, qlcftdi-libftdi.cpp):
 *   - 250000 baud, 8 data bits, 2 stop bits, no parity
 *   - A REAL hardware break (110us) via the FTDI chip's own break-
 *     control bit (ftdi_set_line_property2(..., BREAK_ON/OFF)) -- not
 *     a software baud-switch simulation.
 *   - 16us mark-after-break, ~30Hz default refresh rate.
 *
 * This is read directly as a local TTL serial link (FTDI TX -> ESP32
 * RX) -- DMX512's RS-485 differential signaling only matters on an
 * actual multi-drop cable run, not this point-to-point USB-serial tap,
 * so no RS-485 conversion is needed or relevant here.
 *
 * DMX reception uses the Dmx_ESP32 library's dmxRx class (RMT-based
 * break detection) at its default settings. Only channels 1-6 are read
 * and forwarded.
 */

#include <Dmx_ESP32.h>
#include "ESPNowDMX_Sender.h"

#define DMX_RX_PIN   20   // FTDI TTL DMX line -> GPIO20 (matches this
                           // project's original C3 wiring)
#define NUM_CHANNELS 512
#define DMX_TASK_PRIORITY 5   // see rationale in the header comment above

// Uncomment to enable the periodic serial debug dump (frame count +
// current channel values) in dmxTask.
//#define DMX_DEBUG

dmxRx dmx(&Serial1, DMX_RX_PIN);
ESPNowDMX_Sender sender;

// Whole-frame sanity filter: on the same library, on a different board
// (ESP32-WROOM-32) in this project, the flicker turned out to be the
// ENTIRE read (all 6 tracked channels at once) flipping between real
// values and all-zero for sustained stretches (dozens of frames,
// 0.5-1.5+ seconds), while QLC+'s own output monitor stayed rock-
// steady throughout -- pointing at the dmxRx library's own buffer/state
// occasionally self-clearing, not source corruption. A per-channel
// filter is the wrong tool for that failure mode. Instead: if every
// tracked channel reads 0 in the SAME update, treat that as suspect
// and hold the last known good values, UNLESS it persists far longer
// than the observed glitch ever did (ALL_ZERO_HOLD_FRAMES), in which
// case it's accepted as a real blackout.
uint8_t lastSent[NUM_CHANNELS + 1] = {0};
unsigned long allZeroStreak = 0;
#define ALL_ZERO_HOLD_FRAMES 90   // ~3s at ~30Hz -- well beyond any
                                  // observed glitch duration

void dmxTask(void *pvParameters) {
  dmx.configure();
  dmx.start();

#ifdef DMX_DEBUG
  unsigned long lastPrint = 0;
  unsigned long frameCount = 0;
#endif

  for (;;) {
    if (dmx.hasUpdated()) {
#ifdef DMX_DEBUG
      frameCount++;
#endif

      uint8_t raw[NUM_CHANNELS + 1];
      bool allZero = true;
      for (int ch = 1; ch <= NUM_CHANNELS; ch++) {
        raw[ch] = dmx.read(ch);
        if (raw[ch] != 0) allZero = false;
      }

      if (allZero) {
        allZeroStreak++;
        if (allZeroStreak > ALL_ZERO_HOLD_FRAMES) {
          // Persisted far longer than the observed glitch ever did --
          // accept it as a genuine blackout.
          for (int ch = 1; ch <= NUM_CHANNELS; ch++) {
            lastSent[ch] = 0;
            sender.setChannel(ch, 0);
          }
        }
        // else: suspected buffer-clear glitch -- hold last known good
        // values, don't touch sender at all this cycle.
      } else {
        allZeroStreak = 0;
        for (int ch = 1; ch <= NUM_CHANNELS; ch++) {
          if (raw[ch] != lastSent[ch]) {
            lastSent[ch] = raw[ch];
            sender.setChannel(ch, raw[ch]);
          }
        }
      }
    }

#ifdef DMX_DEBUG
    unsigned long now = millis();
    if (now - lastPrint >= 500) {
      lastPrint = now;
      Serial.print(now);
      Serial.print("ms frames="); Serial.print(frameCount);
      Serial.print(" ch1="); Serial.print(lastSent[1]);
      Serial.print(" ch2="); Serial.print(lastSent[2]);
      Serial.print(" ch3="); Serial.print(lastSent[3]);
      Serial.print(" ch4="); Serial.print(lastSent[4]);
      Serial.print(" ch5="); Serial.print(lastSent[5]);
      Serial.print(" ch6="); Serial.println(lastSent[6]);
    }
#endif

    vTaskDelay(1);
  }
}

void setup() {
  Serial.begin(115200);   // USB serial, separate from Serial1/DMX
#ifdef DMX_DEBUG
  Serial.println("DMX->ESPNOW bridge starting...");
#endif

  sender.begin();

  uint8_t universe[512] = {0};
  sender.setUniverse(universe);

  // No PinnedToCore variant needed -- the C3 only has one core, so a
  // plain dedicated task at a moderate priority is the right shape
  // here (see header comment).
  xTaskCreate(dmxTask, "dmxTask", 4096, NULL, DMX_TASK_PRIORITY, NULL);

#ifdef DMX_DEBUG
  Serial.println("dmxTask created, running.");
#endif
}

void loop() {
  sender.loop();  // the ESP-NOW send side
}