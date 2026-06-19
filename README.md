# R200-RFID

Arduino library for the R200 RFID reader/writer.

## Installation

1. Clone or download this repository into your Arduino libraries folder as `R200-RFID`.
2. Restart Arduino IDE.
3. Open **File → Examples → R200-RFID → BasicRead**.

## Supported boards

- ESP32 (`ARDUINO_ARCH_ESP32`)
- Arduino Uno/Nano (`ARDUINO_AVR_UNO`, `ARDUINO_AVR_NANO`)
- Arduino Zero / SAMD (`ARDUINO_ARCH_SAMD`)

## Serial port behavior

By default, the library selects:

- **ESP32**: `Serial2`
- **Uno/Nano**: `Serial`
- **Zero/SAMD**: `Serial1`

You can override with:

```cpp
r200.setSerial(Serial2); // example
```

Call `begin(baud)` after setting your preferred serial.

## Debug logging

Compile-time debug toggle:

```cpp
#define R200_DEBUG 1
#include <R200RFID.h>
```

Runtime debug toggle:

```cpp
r200.setDebugStream(Serial);
r200.setDebug(true);   // or false
```

When `R200_DEBUG` is `0` (default), debug methods compile to no-op behavior.

## Notes

- On Uno/Nano, `Serial` is shared with USB serial monitor, so reader traffic may conflict with monitor output.
- On ESP32, if your board needs custom UART pins, initialize `Serial2` yourself and call `setSerial(Serial2)` before `begin()`.