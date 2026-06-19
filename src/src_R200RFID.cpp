#include "R200RFID.h"

R200RFID::R200RFID()
: _io(nullptr), _debugOut(nullptr), _runtimeDebug(false), _baud(R200_DEFAULT_BAUD) {}

void R200RFID::setSerial(HardwareSerial& serial) {
  _io = &serial;
}

void R200RFID::setDebug(bool enabled) {
  _runtimeDebug = enabled;
}

void R200RFID::setDebugStream(Stream& stream) {
  _debugOut = &stream;
}

void R200RFID::initDefaultSerial(uint32_t baud) {
#if defined(ARDUINO_ARCH_ESP32)
  // Default ESP32 mapping can vary by board package.
  // Users can call setSerial() and configure pins externally if needed.
  _io = &Serial2;
  _io->begin(baud);
#elif defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO)
  // Uno/Nano has one hardware UART (Serial) shared with USB.
  _io = &Serial;
  _io->begin(baud);
#elif defined(ARDUINO_ARCH_SAMD)
  // Zero-family usually uses SerialUSB for monitor and Serial1 for UART pins.
  _io = &Serial1;
  _io->begin(baud);
#else
  _io = &Serial;
  _io->begin(baud);
#endif
}

bool R200RFID::begin(uint32_t baud) {
  _baud = baud;

  if (_io == nullptr) {
    initDefaultSerial(baud);
  } else {
    _io->begin(baud);
  }

#if R200_DEBUG
  if (_debugOut == nullptr) {
    _debugOut = &Serial;
    Serial.begin(115200);
    delay(20);
  }
  if (_runtimeDebug) {
    dbgPrint(F("[R200] begin()"));
  }
#endif

  return (_io != nullptr);
}

size_t R200RFID::writeFrame(const uint8_t* data, size_t len) {
  if (_io == nullptr || data == nullptr || len == 0) return 0;

#if R200_DEBUG
  if (_runtimeDebug) {
    dbgPrint(F("[R200] TX:"));
    dbgPrintHex(data, len);
  }
#endif

  return _io->write(data, len);
}

int R200RFID::available() {
  if (_io == nullptr) return 0;
  return _io->available();
}

size_t R200RFID::readFrame(uint8_t* out, size_t maxLen, uint32_t timeoutMs) {
  if (_io == nullptr || out == nullptr || maxLen == 0) return 0;

  size_t n = 0;
  uint32_t start = millis();
  while ((millis() - start) < timeoutMs && n < maxLen) {
    while (_io->available() > 0 && n < maxLen) {
      out[n++] = static_cast<uint8_t>(_io->read());
    }
    delay(1);
  }

#if R200_DEBUG
  if (_runtimeDebug) {
    dbgPrint(F("[R200] RX:"));
    dbgPrintHex(out, n);
  }
#endif

  return n;
}

bool R200RFID::sendCommand(const uint8_t* cmd, size_t cmdLen,
                           uint8_t* response, size_t& responseLen,
                           uint32_t timeoutMs) {
  responseLen = 0;
  if (cmd == nullptr || cmdLen == 0 || response == nullptr) return false;

  size_t written = writeFrame(cmd, cmdLen);
  if (written != cmdLen) return false;

  responseLen = readFrame(response, responseLen == 0 ? 64 : responseLen, timeoutMs);
  return (responseLen > 0);
}

void R200RFID::dbgPrint(const __FlashStringHelper* msg) {
#if R200_DEBUG
  if (_runtimeDebug && _debugOut != nullptr) {
    _debugOut->println(msg);
  }
#else
  (void)msg;
#endif
}

void R200RFID::dbgPrint(const char* msg) {
#if R200_DEBUG
  if (_runtimeDebug && _debugOut != nullptr) {
    _debugOut->println(msg);
  }
#else
  (void)msg;
#endif
}

void R200RFID::dbgPrintHex(const uint8_t* data, size_t len) {
#if R200_DEBUG
  if (!_runtimeDebug || _debugOut == nullptr || data == nullptr) return;

  for (size_t i = 0; i < len; ++i) {
    if (data[i] < 0x10) _debugOut->print('0');
    _debugOut->print(data[i], HEX);
    _debugOut->print(' ');
  }
  _debugOut->println();
#else
  (void)data;
  (void)len;
#endif
}