/**
 * Basic R200 usage example.
 *
 * Board notes:
 * - ESP32: default R200 UART is Serial2 (override with setSerial if needed)
 * - Uno/Nano: default R200 UART is Serial (shared with USB serial monitor)
 * - Zero (SAMD): default R200 UART is Serial1
 */

// Uncomment to compile in debug logs
// #define R200_DEBUG 1

#include <R200RFID.h>

R200RFID r200;

void setup() {
  // Optional monitor debug stream
  Serial.begin(115200);
  delay(200);

#if R200_DEBUG
  r200.setDebugStream(Serial);
  r200.setDebug(true);
#endif

#if defined(ARDUINO_ARCH_ESP32)
  // Optional explicit pin mapping for some ESP32 boards:
  // Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  // r200.setSerial(Serial2);
#elif defined(ARDUINO_AVR_UNO)
  // Uno has one UART. If connected to USB serial monitor, avoid contention.
  // Using default Serial transport for the reader.
#elif defined(ARDUINO_ARCH_SAMD)
  // Zero: SerialUSB for monitor, Serial1 for R200 by default in library.
#endif

  if (!r200.begin()) {
    Serial.println("R200 begin failed");
    while (1) delay(100);
  }

  Serial.println("R200 ready");
}

void loop() {
  // Example placeholder command bytes.
  // Replace with real R200 protocol command from your existing implementation.
  const uint8_t cmd[] = {0xAA, 0x00, 0x01, 0x00, 0xAB};

  uint8_t response[128];
  size_t responseLen = sizeof(response);

  bool ok = r200.sendCommand(cmd, sizeof(cmd), response, responseLen, 300);

  if (ok) {
    Serial.print("Response bytes: ");
    Serial.println(responseLen);
  } else {
    Serial.println("No response / command failed");
  }

  delay(1000);
}