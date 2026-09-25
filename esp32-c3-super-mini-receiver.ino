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

// ---------------------------------------------------------------------------
// Relay forwarding state
//
// All variables below are accessed exclusively from the WiFi task (inside the
// ESP-NOW receive callback). No mutex is required for them.
//
// Deduplication works at two independent levels:
//
//   • Chunk level  (seqNumber, uint16_t):  forward a chunk only when its
//     sequence number is strictly newer than the last one forwarded.  This
//     prevents the same chunk from being re-broadcast by a relay that hears
//     both the original sender and another relay.
//
//   • Frame level  (frameId, uint8_t):  pass DMX data to dmxOut only when
//     the frameId differs from the most recently processed one.  Multiple
//     chunks that share the same frameId belong to the same 512-channel
//     frame; we must not re-apply dmxData after it is already current.
//
// Wrap-safe comparison for seqNumber uses modular subtraction:
//   diff = seq - lastForwardedSeq  (unsigned 16-bit, wraps naturally)
//   forward when diff != 0 && diff < 0x8000  ("newer but not impossibly far ahead")
// ---------------------------------------------------------------------------
static const uint8_t broadcastAddr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static uint8_t  lastProcessedFrameId = 0xFF; // init to "never seen"
static uint8_t  lastSessionIdRelay   = 0;
static bool     hasSessionIdRelay    = false;
static uint16_t lastForwardedSeq     = 0;
static bool     hasForwardedSeq      = false;

// Set in onEspNowReceive() before calling receiver.handleReceive();
// read in dmxCallback() which is invoked synchronously from handleReceive()
// on the same WiFi task — no mutex needed.
static bool processThisFrame = false;

// ---------------------------------------------------------------------------
// DMX callback — called by the library after a complete frame is assembled.
// Only copies data into dmxData when this is a genuinely new frame.
// ---------------------------------------------------------------------------
void dmxCallback(uint8_t universe, const uint8_t* data) {
  if (!processThisFrame) return;
  portENTER_CRITICAL(&dmxMux);
  memcpy(dmxData, data, NUM_DMX_CHANNELS);
  portEXIT_CRITICAL(&dmxMux);
}

// ---------------------------------------------------------------------------
// Common relay + dispatch logic (called from the ESP-NOW recv callback shim).
// Extracts relay-relevant header fields, re-broadcasts new chunks, gates the
// DMX output flag, then hands the raw packet to the library for full parsing.
// ---------------------------------------------------------------------------
static void onEspNowReceive(const uint8_t *mac, const uint8_t *data, int len) {
  if (len >= PACKET_HEADER_SIZE && data[0] == PACKET_TYPE_DATA_CHUNK) {
    uint8_t  sessionId = data[2];
    uint8_t  frameId   = data[3];
    uint16_t seq       = ((uint16_t)data[4] << 8) | data[5];

    // New session: reset relay state so stale dedup counters don't block the
    // first legitimate chunks (initialise to one-before so the first packet
    // is always accepted).
    if (!hasSessionIdRelay || sessionId != lastSessionIdRelay) {
      lastProcessedFrameId = (uint8_t)(frameId - 1);
      lastForwardedSeq     = (uint16_t)(seq - 1);
      hasSessionIdRelay    = true;
      hasForwardedSeq      = true;
      lastSessionIdRelay   = sessionId;
    }

    // --- Chunk-level dedup: forward only if seqNumber is strictly newer ---
    // The broadcast peer (FF:FF:FF:FF:FF:FF) was registered by receiver.begin().
    uint16_t seqDiff = seq - lastForwardedSeq;
    if (!hasForwardedSeq || (seqDiff != 0 && seqDiff < 0x8000)) {
      esp_now_send(broadcastAddr, data, len);
      lastForwardedSeq = seq;
      hasForwardedSeq  = true;
    }

    // --- Frame-level dedup: only output DMX data for a new frameId ---
    processThisFrame = (frameId != lastProcessedFrameId);
    if (processThisFrame) {
      lastProcessedFrameId = frameId;
    }
  }

  // Always hand the packet to the library — it maintains its own session /
  // sequence state and drives the dmxCallback when a frame is complete.
  receiver.handleReceive(mac, data, len);
}

// ---------------------------------------------------------------------------
// ESP-NOW receive callback shim — signature differs between IDF versions.
// Registered in setup() to replace the library's internal callback so that
// relay forwarding runs before (and alongside) normal DMX processing.
// ---------------------------------------------------------------------------
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static void espNowRecvCb(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  const uint8_t *mac = (info && info->src_addr) ? info->src_addr : nullptr;
  onEspNowReceive(mac, data, len);
}
#else
static void espNowRecvCb(const uint8_t *mac, const uint8_t *data, int len) {
  onEspNowReceive(mac, data, len);
}
#endif

void setup() {
  Serial.begin(115200);   // USB-CDC, for debug only — NOT the DMX UART

  // --- Dmx_ESP32 setup (uses Serial0 / UART0 internally now) ---
  dmxOut.configure();

  // --- ESP-NOW DMX receiver setup ---
  // begin(true) initialises WiFi + ESP-NOW, registers the broadcast peer
  // (FF:FF:FF:FF:FF:FF), and installs the library's internal recv callback.
  receiver.begin();

  // Disable sender pairing: relay-forwarded chunks arrive from the relay's
  // MAC address, not the original sender's. Pairing would lock onto the first
  // source MAC seen and reject all subsequent relayed packets.
  receiver.setPairingEnabled(false);

  receiver.setDMXReceiveCallback(dmxCallback);

  // Replace the library's internal recv callback with our relay-aware shim.
  // esp_now_register_recv_cb() replaces any previously registered callback;
  // our shim calls receiver.handleReceive() so library processing continues.
  esp_now_register_recv_cb(espNowRecvCb);
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
