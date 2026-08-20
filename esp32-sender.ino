/*
    This sketch receives raw dmx data from QLC+ via FTDI Usb-serial adapter
    and forwards it via espnow. The FTDI adapter is necessary to have an unpatched version
    of QLC+ recognize the device as a DMX interface.
*/

// Single-board DMX -> ESP-NOW bridge.
// Replaces the old Arduino(DMX in)+ESP32(ESP-NOW out) pair: this ESP32 receives raw
// DMX/QLC+ serial directly on a hardware UART and forwards it via ESPNowDMX.
//
// DMX input wiring: external USB-to-serial adapter -> GPIO16 (RX only, UART2).
// UART0 (the board's own USB port) is left free for Serial debug output.
#include "driver/uart.h"
#include <WiFi.h>
#include "ESPNowDMX.h"
#define DMX_CHANNELS   512
#define DMX_UART_NUM   UART_NUM_2
#define DMX_RX_PIN     16
#define DMX_BAUD       250000
#define LED_PIN        2   // mirrors DMX channel 1, same role as the old Arduino's heartbeat LED
#define UART_RX_BUF_SIZE      1024
#define UART_EVENT_QUEUE_LEN  20
ESPNowDMX dmx;
static QueueHandle_t uartQueue;
static portMUX_TYPE   dmxMux = portMUX_INITIALIZER_UNLOCKED;
// Touched only by dmxUartTask.
static uint8_t dmxBuffer[DMX_CHANNELS];
static int     dmxIndex = -1;
// Handoff to loop(): written by dmxUartTask under dmxMux, read/cleared by loop() under dmxMux.
static uint8_t frameBuffer[DMX_CHANNELS];
static volatile bool frameReady = false;
// last values actually pushed into ESPNowDMX, so we only call setChannel() on real changes
static uint8_t lastSent[DMX_CHANNELS];
static void handleDmxByte(uint8_t data) {
  if (dmxIndex == 0) {
    dmxIndex = (data == 0x00) ? 1 : -1;   // only the standard NULL start code is forwarded
    return;     
  }
  if (dmxIndex >= 1 && dmxIndex <= DMX_CHANNELS) {
    dmxBuffer[dmxIndex - 1] = data;
    dmxIndex++;
    if (dmxIndex > DMX_CHANNELS) {
      portENTER_CRITICAL(&dmxMux);
      memcpy(frameBuffer, dmxBuffer, DMX_CHANNELS);
      frameReady = true;
      portEXIT_CRITICAL(&dmxMux);
      dmxIndex = -1;
    }
  }
}
static void dmxUartTask(void*) {
  uint8_t rxTemp[UART_RX_BUF_SIZE];
  uart_event_t event;
  for (;;) {
    if (!xQueueReceive(uartQueue, &event, portMAX_DELAY)) continue;
    switch (event.type) {
      case UART_BREAK:
        // DMX BREAK: the hardware UART reports this itself (RXD held low past a frame time),
        // so no framing-error trick is needed here the way the AVR ISR needed one.
        dmxIndex = 0;
        break;
      case UART_DATA: {
        int len = uart_read_bytes(DMX_UART_NUM, rxTemp, event.size, 0);
        for (int i = 0; i < len; i++) handleDmxByte(rxTemp[i]);
        break;
      }
      case UART_FIFO_OVF:
      case UART_BUFFER_FULL:
        uart_flush_input(DMX_UART_NUM);
        xQueueReset(uartQueue);
        dmxIndex = -1;
        break;
      default:
        break;
    }
  }
}
void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);   // debug only; DMX input is entirely on UART2 now
  uart_config_t uartConfig = {
    .baud_rate = DMX_BAUD,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_2,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
  };
  uart_driver_install(DMX_UART_NUM, UART_RX_BUF_SIZE, 0, UART_EVENT_QUEUE_LEN, &uartQueue, 0);
  uart_param_config(DMX_UART_NUM, &uartConfig);
  uart_set_pin(DMX_UART_NUM, UART_PIN_NO_CHANGE, DMX_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  xTaskCreatePinnedToCore(dmxUartTask, "dmxUart", 4096, NULL, 3, NULL, 0);
  dmx.setUniverseId(0);
  dmx.begin(ESPNOW_DMX_MODE_SENDER);
  // ramp TX power up gradually instead of jumping straight to max
  WiFi.setTxPower(WIFI_POWER_2dBm);
  delay(200);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  //delay(200);
  //WiFi.setTxPower(WIFI_POWER_19_5dBm); // or your target power
}
void loop() {
  if (frameReady) {
    uint8_t local[DMX_CHANNELS];
    portENTER_CRITICAL(&dmxMux);
    memcpy(local, frameBuffer, DMX_CHANNELS);
    frameReady = false;
    portEXIT_CRITICAL(&dmxMux);
    for (int i = 0; i < DMX_CHANNELS; i++) {
      if (local[i] != lastSent[i]) {
        dmx.setChannel(i + 1, local[i]);   // 1-based address
        lastSent[i] = local[i];
      }
    }
    analogWrite(LED_PIN, local[0]);
  }
  dmx.loop();   // flushes any setChannel() updates as adaptive ESP-NOW sends
}