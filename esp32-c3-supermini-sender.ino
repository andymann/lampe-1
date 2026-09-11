/*
 * DMX receiver -> forwards the full 512-channel universe out via
 * ESP-NOW using the ESPNowDMX_Sender library.
 *
 * Target board: ESP32-C3 Super Mini (single-core RISC-V). DMX reading
 * runs in its own dedicated FreeRTOS task rather than being polled
 * inline from loop().
 *
 * The glitch filter (all-zero self-clear / torn short frame
 * detection, see DmxSanityFilter.h for the full writeup) now lives in
 * DmxSanityFilter, a small companion class wrapping dmxRx. It is
 * DISABLED by default -- explicitly enabled below via
 * enableSanityCheck(true), since this project needs it. To turn it
 * off (e.g. to test whether flicker is still filter-related), just
 * comment out that one line. setSuspectHoldFrames() lets you tune how
 * many consecutive suspect updates get held before being trusted as a
 * real event -- 90 (~3s at ~30Hz) is this project's working default,
 * shown explicitly below even though it doesn't need to be, since
 * it's likely to need tuning as the fixture grows.
 */

#include <Dmx_ESP32.h>
#include "ESPNowDMX_Sender.h" //https://github.com/andymann/ESPNowDMX
#include "DmxSanityFilter.h" //https://github.com/andymann/DmxSanityFilter

#define DMX_RX_PIN 20   // FTDI TTL DMX line -> GPIO20 (matches this
                        // project's original C3 wiring)
#define DMX_TASK_PRIORITY 3 // kept below Dmx_ESP32's internal readTask
                             // priority (4) as defense in depth.
#define NUM_CHANNELS 512

// Uncomment to enable a per-frame debug print on every filter.update()
// call that reports a change, tagged with how it was classified. Only
// lists channels that just changed -- printing all 512 values every
// frame would flood the serial port at 115200 baud.
#define RAW_FRAME_DEBUG

dmxRx dmx(&Serial1, DMX_RX_PIN);
ESPNowDMX_Sender sender;
DmxSanityFilter filter(dmx, NUM_CHANNELS);

void dmxTask(void *pvParameters) {
  dmx.configure();
  dmx.start();

  filter.enableSanityCheck(true);   // OFF by default -- this project needs it on
  filter.setSuspectHoldFrames(90);  // this project's working default; tune as needed

  for (;;) {
    if (filter.update()) {
      for (uint16_t ch = 1; ch <= NUM_CHANNELS; ch++) {
        if (filter.channelChanged(ch)) {
          sender.setChannel(ch, filter.channelValue(ch));
#ifdef RAW_FRAME_DEBUG
          Serial.print("ch"); Serial.print(ch);
          Serial.print("="); Serial.print(filter.channelValue(ch));
          Serial.print(' ');
#endif
        }
      }
#ifdef RAW_FRAME_DEBUG
      const char *statusStr;
      switch (filter.lastStatus()) {
        case DmxFrameStatus::Forwarded: statusStr = "FWD";      break;
        case DmxFrameStatus::Accepted:  statusStr = "ACCEPTED"; break;
        case DmxFrameStatus::Held:      statusStr = "HELD";     break;
        default:                        statusStr = "BYPASSED"; break;
      }
      Serial.print("[");
      Serial.print(statusStr);
      Serial.print("] @");
      Serial.println(millis());
#endif
    }
    vTaskDelay(1);
  }
}

void setup() {
  Serial.begin(115200); // USB serial, separate from Serial1/DMX
#ifdef RAW_FRAME_DEBUG
  Serial.println("Raw per-frame debug enabled (full 512-channel universe).");
#endif

  sender.begin();

  uint8_t universe[512] = {0};
  sender.setUniverse(universe);

  xTaskCreate(dmxTask, "dmxTask", 4096, NULL, DMX_TASK_PRIORITY, NULL);
}

void loop() {
  sender.loop(); // the ESP-NOW send side
}
