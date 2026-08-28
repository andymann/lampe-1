/*
    This sketch receives raw dmx data from QLC+ via FTDI Usb-serial adapter
    and forwards it via espnow. The FTDI adapter is necessary to have an unpatched version
    of QLC+ recognize the device as a DMX interface.
*/

// Single-board DMX -> ESP-NOW bridge, for ESP32-C3 Super Mini.
//
// DMX input wiring: external USB-to-serial adapter TX -> GPIO20, GND -> board GND, 3.3V logic.
//
// DMX reception via Dmx_ESP32 (RMT-based break detection).
//
// - Only scan/forward as many channels as you actually use (DMX_CHANNELS
//   below) -- scanning the full 512-channel universe every frame caused
//   intermittent dark dropouts on this single-core chip.
// - Reception occasionally glitches to a spurious 0 for a couple of
//   frames (confirmed via packet-level logging on the receiver: the
//   sender itself transmits a correct 2-byte delta packet claiming a
//   channel dropped to 0, before self-correcting a few packets later --
//   i.e. this is a real reception artifact in Dmx_ESP32 on this chip, not
//   a transmission or receiver-decode issue). A zero-confirmation filter
//   requires a drop to 0 to persist for 3 consecutive frames before it's
//   trusted, while passing every other value (including active fades)
//   through immediately with no added latency.
// - ESPNowDMX_Sender is patched to send unicast to the receiver's specific
//   MAC instead of broadcast, so lost frames get real hardware ack/retry.
// - ESPNOW_DMX_ENABLE_COMPRESSION is disabled in ESPNowDMX_Common.h --
//   the heatshrink decompression path could write partial/corrupt data
//   into the receiver's live buffer before reporting failure.
#include <Dmx_ESP32.h>
#include <WiFi.h>
#include "ESPNowDMX.h"

#define DMX_CHANNELS 64   // <-- set to the number of channels you actually use

#define RX_PIN     20
#define RX_RMT     20
#define RX_DISABLE -1
#define LED_PIN    3

#define DMX_PORT_R &Serial1

dmxRx dmxReceive = dmxRx(DMX_PORT_R, RX_PIN, RX_RMT, RX_DISABLE, LED_PIN, LOW);
ESPNowDMX dmx;

static uint8_t lastSent[DMX_CHANNELS];
static uint8_t lastGood[DMX_CHANNELS];
static uint8_t zeroStreak[DMX_CHANNELS];
static const uint8_t ZERO_CONFIRM_FRAMES = 3;   // requires 3 consecutive 0-readings before trusting a drop to 0

void setup() {
  Serial.begin(115200);
  delay(1500);   // let native USB CDC finish enumerating before first print
  Serial.println("ready...");

  if (!dmxReceive.configure()) {
    Serial.println("DMX Receive Configure failed.");
  } else {
    Serial.println("DMX Receive Configured.");
  }

  delay(10);

  if (dmxReceive.start()) {
    Serial.println("DMX reception started");
  } else {
    Serial.println("DMX reception aborted");
  }

  dmx.setUniverseId(0);
  dmx.begin(ESPNOW_DMX_MODE_SENDER);
  dmx.setFullRefreshInterval(60);   // was 200ms default
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
}

void loop() {
  if (dmxReceive.hasUpdated()) {
    for (int i = 1; i <= DMX_CHANNELS; i++) {
      uint8_t raw = dmxReceive.read(i);
      int idx = i - 1;
      uint8_t out;

      if (raw == 0 && lastGood[idx] != 0) {
        // suspicious: a sudden drop to 0 -- don't trust it yet
        zeroStreak[idx]++;
        if (zeroStreak[idx] >= ZERO_CONFIRM_FRAMES) {
          out = 0;              // confirmed real -- accept it
          lastGood[idx] = 0;
        } else {
          out = lastGood[idx];  // still unconfirmed -- hold the last known-good value
        }
      } else {
        zeroStreak[idx] = 0;
        out = raw;
        lastGood[idx] = raw;
      }

      if (out != lastSent[idx]) {
        dmx.setChannel(i, out);
        lastSent[idx] = out;
      }
    }
  }

  dmx.loop();
}