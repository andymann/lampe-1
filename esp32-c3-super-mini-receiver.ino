#include <Arduino.h>
#include "ESPNowDMX_Receiver.h"
#include <Dmx_ESP32.h>

#define NUM_DMX_CHANNELS 512   // note: library itself already #defines DMX_CHANNELS as 513 (start code + 512 slots)
#define DEBUG 1

// --- DMX output config ---
// ESP32-C3 has only ONE hardware UART. We use it (Serial0 / UART0) exclusively
// for DMX output. Debug logging goes over the native USB-CDC interface instead,
// which the Arduino core exposes as "Serial" as long as
// Tools > USB CDC On Boot: "Enabled" is set (required on Super Mini boards).
//
// Default UART0 pins on the ESP32-C3 Super Mini: TX0 = GPIO21, RX0 = GPIO20.
// RX isn't needed for DMX TX-only operation but the pin is still reserved.

const int dmxTransmitPin = 21;  // TX0 -> RS485 DI
const int dmxEnablePin   = 4;   // DE/RE tied together on most RS485 modules — pick any free GPIO (avoid 8/9 strapping pins)

dmxTx dmxOut(&Serial0, dmxTransmitPin, dmxEnablePin);

// received data is written from the ESP-NOW/WiFi task, read from loop() ->
// guard the shared buffer with a small critical section.
// (Still needed even though the C3 is single-core: the callback and loop()
// run in different FreeRTOS tasks, so this protects against task-switch races,
// not just multicore races.)
uint8_t dmxData[NUM_DMX_CHANNELS];
portMUX_TYPE dmxMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool newDmxData = false;

ESPNowDMX_Receiver receiver;

void dmxCallback(uint8_t universe, const uint8_t* data) {
#ifdef DEBUG
  Serial.printf("Received DMX universe %d - first 8 values: %d %d %d %d %d %d %d %d\n",
                universe, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
#endif
  portENTER_CRITICAL(&dmxMux);
  memcpy(dmxData, data, NUM_DMX_CHANNELS);
  newDmxData = true;
  portEXIT_CRITICAL(&dmxMux);
}

void onEspNowReceive(const uint8_t *mac, const uint8_t *data, int len) {
  receiver.handleReceive(mac, data, len);
}

void setup() {
  Serial.begin(115200);   // USB-CDC, for debug only — NOT the DMX UART

  // --- Dmx_ESP32 setup (uses Serial0 / UART0 internally now) ---
  dmxOut.configure();

  // --- ESP-NOW DMX receiver setup ---
  receiver.begin();  // true by default = internal ESP-NOW init
  receiver.setDMXReceiveCallback(dmxCallback);
}

void loop() {
  // Copy latest received frame into the DMX TX buffer and send it out.
  // Re-sending the last known frame even without new data keeps fixtures
  // from timing out if ESP-NOW packets are lost.
  portENTER_CRITICAL(&dmxMux);
  dmxOut.writeBytes(dmxData, NUM_DMX_CHANNELS, 1); // channel numbering starts at 1
  newDmxData = false;
  portEXIT_CRITICAL(&dmxMux);

  if (dmxOut.readyToTransmit()) {
    dmxOut.transmit();
  }

  delay(23); // ~40 Hz DMX refresh
}