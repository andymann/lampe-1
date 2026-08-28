#include <Arduino.h>
#include "ESPNowDMX_Receiver.h"
#include <Dmx_ESP32.h>

#define NUM_DMX_CHANNELS 512   // note: library itself already #defines DMX_CHANNELS as 513 (start code + 512 slots)

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
// guard the shared buffer with a small critical section. The critical
// section only ever protects the raw memcpy -- never a call into
// dmxOut.writeBytes()/transmit(), since those have an internal duration
// outside our control and holding interrupts disabled that whole time
// risks dropping incoming ESP-NOW packets.
uint8_t dmxData[NUM_DMX_CHANNELS];
portMUX_TYPE dmxMux = portMUX_INITIALIZER_UNLOCKED;

ESPNowDMX_Receiver receiver;

void dmxCallback(uint8_t universe, const uint8_t* data) {
  const uint8_t expected = 149;   // whatever QLC+ channel 2 is set to
  static int consecutiveBad = 0;

  uint8_t v = data[1];

  if (v != expected) {
    consecutiveBad++;
  } else {
    if (consecutiveBad > 0) {
      Serial.printf("[%lu] recovered after %d bad callback(s)\n", millis(), consecutiveBad);
    }
    consecutiveBad = 0;
  }

  portENTER_CRITICAL(&dmxMux);
  memcpy(dmxData, data, NUM_DMX_CHANNELS);
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
  // Take a quick local snapshot under the lock, then release the lock
  // BEFORE calling into dmxOut.writeBytes()/transmit().
  static uint8_t localData[NUM_DMX_CHANNELS];

  portENTER_CRITICAL(&dmxMux);
  memcpy(localData, dmxData, NUM_DMX_CHANNELS);
  portEXIT_CRITICAL(&dmxMux);

  dmxOut.writeBytes(localData, NUM_DMX_CHANNELS, 1); // channel numbering starts at 1

  if (dmxOut.readyToTransmit()) {
    dmxOut.transmit();
  }

  delay(23); // ~40 Hz DMX refresh
}