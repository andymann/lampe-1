#include <Arduino.h>
#include "ESPNowDMX_Receiver.h"
#include <Dmx_ESP32.h>

#define NUM_DMX_CHANNELS 512   // note: library itself already #defines DMX_CHANNELS as 513 (start code + 512 slots)
#define DEBUG 1

// --- DMX output config ---
HardwareSerial dmxSerial(2);   // use UART2

const int dmxTransmitPin = 4;   // TX -> RS485 DI
const int dmxEnablePin   = 17;  // DE/RE tied together on most RS485 modules — adjust to your wiring

dmxTx dmxOut(&dmxSerial, dmxTransmitPin, dmxEnablePin);

// received data is written from the ESP-NOW/WiFi task, read from loop() ->
// guard the shared buffer with a small critical section
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
  Serial.begin(115200);

  // --- Dmx_ESP32 setup ---
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