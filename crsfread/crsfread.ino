#include <HardwareSerial.h>

// --- Pins ---
// Radio single-wire pad:
constexpr int BUS_RX   = 16;  // read from radio pad
constexpr int BUS_TX   = 17;  // write to same pad (open-drain)

// Clean two-wire UART to your device:
constexpr int CLEAN_RX = 26;  // from device TX
constexpr int CLEAN_TX = 25;  // to device RX

// --- Speeds ---
constexpr uint32_t CRSF_BAUD  = 400000;  // radio bus, inverted 8N1
constexpr uint32_t CLEAN_BAUD = 400000;  // clean UART, normal 8N1

HardwareSerial Radio(1);  // UART1 for radio bus
HardwareSerial Clean(2);  // UART2 for clean two-wire pins

void setup() {
  // One-wire bus: never drive high
  pinMode(BUS_RX, INPUT_PULLUP);
  pinMode(BUS_TX, OUTPUT_OPEN_DRAIN);
  digitalWrite(BUS_TX, HIGH); // release bus (hi-Z for '1')

  // Radio side: inverted. RX=BUS_RX, TX=BUS_TX
  // begin(baud, config, rxPin, txPin, invert)
  Radio.begin(CRSF_BAUD, SERIAL_8N1, BUS_RX, BUS_TX, true);

  // Clean two-wire UART: normal polarity
  Clean.begin(CLEAN_BAUD, SERIAL_8N1, CLEAN_RX, CLEAN_TX, false);
}

void loop() {
  // Radio -> Clean (immediate, byte-by-byte)
  int b = Radio.read();
  if (b >= 0) {
    Clean.write((uint8_t)b);
  }

  // Clean -> Radio (immediate, byte-by-byte)
  b = Clean.read();
  if (b >= 0) {
    Radio.write((uint8_t)b);
  }
}
