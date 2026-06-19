#pragma once

#include <Arduino.h>

/**
 * Debug control:
 *  - Define R200_DEBUG as 1 before including this header to enable verbose logs.
 *  - Default is 0 (disabled).
 */
#ifndef R200_DEBUG
#define R200_DEBUG 0
#endif

/**
 * Default serial and baud selection per board family.
 * You can override by calling setSerial().
 */
#if defined(ARDUINO_ARCH_ESP32)
  #ifndef R200_DEFAULT_BAUD
  #define R200_DEFAULT_BAUD 115200UL
  #endif
#elif defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO)
  #ifndef R200_DEFAULT_BAUD
  #define R200_DEFAULT_BAUD 9600UL
  #endif
#elif defined(ARDUINO_ARCH_SAMD)
  #ifndef R200_DEFAULT_BAUD
  #define R200_DEFAULT_BAUD 115200UL
  #endif
#else
  #ifndef R200_DEFAULT_BAUD
  #define R200_DEFAULT_BAUD 9600UL
  #endif
#endif

class R200RFID {
public:
  R200RFID();

  /**
   * Begin the library.
   * If no serial has been set, it selects a board-appropriate default:
   *  - ESP32: Serial2
   *  - AVR Uno/Nano: Serial (single UART shared with USB)
   *  - SAMD (Zero): Serial1
   */
  bool begin(uint32_t baud = R200_DEFAULT_BAUD);

  /**
   * Inject a serial port explicitly (recommended for full control).
   */
  void setSerial(HardwareSerial& serial);

  /**
   * Enable/disable runtime debug output (only effective when R200_DEBUG == 1).
   */
  void setDebug(bool enabled);

  /**
   * Override debug output stream (defaults to Serial if available).
   */
  void setDebugStream(Stream& stream);

  /**
   * Basic transport helpers.
   * Adjust/extend protocol behavior to match your existing R200 framing.
   */
  size_t writeFrame(const uint8_t* data, size_t len);
  int available();
  size_t readFrame(uint8_t* out, size_t maxLen, uint32_t timeoutMs = 200);

  /**
   * Convenience command API.
   */
  bool sendCommand(const uint8_t* cmd, size_t cmdLen,
                   uint8_t* response, size_t& responseLen,
                   uint32_t timeoutMs = 300);

private:
  HardwareSerial* _io;
  Stream* _debugOut;
  bool _runtimeDebug;
  uint32_t _baud;

  void initDefaultSerial(uint32_t baud);

  void dbgPrint(const __FlashStringHelper* msg);
  void dbgPrint(const char* msg);
  void dbgPrintHex(const uint8_t* data, size_t len);
};