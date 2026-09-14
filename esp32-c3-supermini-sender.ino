/*
 * DMX receiver -> forwards the full 512-channel universe out via
 * ESP-NOW using the ESPNowDMX_Sender library.
 *
 * Target board: ESP32-C3 Super Mini (single-core RISC-V).
 */

/*
  If you are using a stock FTDI USB-to-Serial adapter, manually 
  override QLC+'s interface settings to use "Pro RX/TX" mode. This
  dramatically reduces flickering and DMX dropouts.

  If you want QLC+ to automatically detect the sender as an 
  Enttec DMX USB Pro device you need to overwrite the FTDI's EEPROM.
  More info on that can be found here:
  https://waterpigs.co.uk/articles/ftdi-configure-mac-linux/
*/

#include <HardwareSerial.h>
#include "ESPNowDMX_Sender.h" //https://github.com/andymann/ESPNowDMX

#define DMX_RX_PIN 20  // FTDI TX -> here
#define DMX_TX_PIN 4   // FTDI RX <- here

#define SOM 0x7E
#define EOM 0xE7
#define LABEL_GET_PARAMS 3
#define LABEL_SEND_DMX 6
#define NUM_CHANNELS 512

HardwareSerial ProSerial(1);
ESPNowDMX_Sender sender;

uint8_t previousDmx[NUM_CHANNELS] = {0};

void sendMessage(uint8_t label, const uint8_t* data, uint16_t len) {
  ProSerial.write(SOM);
  ProSerial.write(label);
  ProSerial.write(len & 0xFF);
  ProSerial.write((len >> 8) & 0xFF);
  if (len > 0) ProSerial.write(data, len);
  ProSerial.write(EOM);
}

void handleMessage(uint8_t label, uint8_t* data, uint16_t len) {
  if (label == LABEL_GET_PARAMS) {
    uint8_t reply[5] = { 1, 0, 9, 1, 40 };
    sendMessage(LABEL_GET_PARAMS, reply, sizeof(reply));
  }
  else if (label == LABEL_SEND_DMX) {
    // data[0] is the DMX start code (0x00), channels follow from data[1]
    uint16_t channelCount = (len > 0) ? (len - 1) : 0;
    if (channelCount > NUM_CHANNELS) channelCount = NUM_CHANNELS;

    for (uint16_t i = 0; i < channelCount; i++) {
      uint8_t value = data[1 + i];
      uint16_t ch = i + 1; // sender.setChannel is 1-indexed
      if (value != previousDmx[i]) {
        sender.setChannel(ch, value);
        previousDmx[i] = value;
      }
    }
  }
}

void setup() {
  Serial.begin(9600); // USB CDC monitor
  ProSerial.begin(250000, SERIAL_8N2, DMX_RX_PIN, DMX_TX_PIN);

  sender.begin();
  uint8_t universe[NUM_CHANNELS] = {0};
  sender.setUniverse(universe);
}

void loop() {
  static uint8_t state = 0;
  static uint8_t label = 0;
  static uint16_t dataLen = 0;
  static uint16_t dataIdx = 0;
  static uint8_t buf[600];

  while (ProSerial.available()) {
    uint8_t b = ProSerial.read();
    switch (state) {
      case 0: // wait for SOM
        if (b == SOM) state = 1;
        break;
      case 1: // label
        label = b;
        state = 2;
        break;
      case 2: // length LSB
        dataLen = b;
        state = 3;
        break;
      case 3: // length MSB
        dataLen |= (b << 8);
        dataIdx = 0;
        state = (dataLen == 0) ? 5 : 4;
        break;
      case 4: // payload
        if (dataIdx < sizeof(buf)) buf[dataIdx] = b;
        dataIdx++;
        if (dataIdx >= dataLen) state = 5;
        break;
      case 5: // expect EOM
        if (b == EOM) handleMessage(label, buf, dataLen);
        state = 0;
        break;
    }
  }

  sender.loop(); // ESP-NOW send side
}
