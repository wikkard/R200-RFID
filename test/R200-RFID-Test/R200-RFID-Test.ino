#include <Arduino.h>
#include "RFIDR200.h"

// ---------------- CONFIG ----------------
#define RFID_SERIAL Serial1      // Change if needed
#define RFID_BAUD   115200

RFIDR200 rfid(RFID_SERIAL, RFID_BAUD);

bool pollMode = false;

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  RFID_SERIAL.begin(RFID_BAUD);

  rfid.begin();

  Serial.println("RFIDR200 Serial Controller Ready");
  Serial.println("Commands:");
  Serial.println("  /i                         -> reader info");
  Serial.println("  /r                         -> read single tag");
  Serial.println("  /p                         -> toggle poll mode");
  Serial.println("  /w bank addr len d0 d1..   -> write tag data");
    Serial.println("  /x                       -> reset comms with R200 hardware");
}

// ---------------- LOOP ----------------
void loop() {
  handleSerial();

  if (pollMode) {
    doSingleRead();
    delay(100);
  }
}

// ---------------- SERIAL HANDLER ----------------
void handleSerial() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd.startsWith("/i")) {
    doInfo();
  }
  else if (cmd.startsWith("/r")) {
    doSingleRead();
  }
  else if (cmd.startsWith("/p")) {
    pollMode = !pollMode;
    Serial.print("Poll mode: ");
    Serial.println(pollMode ? "ON" : "OFF");
  }
  else if (cmd.startsWith("/w")) {
  
    pollMode = false;          // <-- STOP POLLING
    delay(50);
    rfid.resetComms();         // <-- CLEAR UART + STOP MULTI POLLING
    delay(50);
    doWrite(cmd);
 }
  else if (cmd.startsWith("/x")) {
    Serial.println("Resetting RFID module...");
    rfid.resetComms();
  }
  else {
    Serial.println("ERR: Unknown command");
  }
}

// Simple token readers using Serial buffer after the command word

bool readNextUint8(uint8_t *out)
{
    long v = Serial.parseInt();   // reads next integer (decimal)
    if (v < 0 || v > 255) return false;
    *out = (uint8_t)v;
    return true;
}

bool readNextUint16(uint16_t *out)
{
    long v = Serial.parseInt();   // reads next integer (decimal)
    if (v < 0 || v > 65535) return false;
    *out = (uint16_t)v;
    return true;
}

bool readNextHexByte(uint8_t *out)
{
    // read next non‑space token as a String, interpret as hex
    String tok = Serial.readStringUntil(' ');
    tok.trim();
    if (tok.length() == 0) return false;

    char *endptr = nullptr;
    uint32_t v = strtoul(tok.c_str(), &endptr, 16);
    if (endptr == tok.c_str() || v > 0xFF) return false;

    *out = (uint8_t)v;
    return true;
}


// ---------------- INFO ----------------
void doInfo() {
  uint8_t resp[16];
  memset(resp, 0, sizeof(resp));

  rfid.getWorkArea(resp, sizeof(resp));
  Serial.print("Region: ");
  Serial.println(resp[0], HEX);

  rfid.getWorkingChannel(resp, sizeof(resp));
  Serial.print("Channel: ");
  Serial.println(resp[0], HEX);
}

// ---------------- SINGLE READ ----------------
void doSingleRead() {
  rfid.initiateSinglePolling();

  uint8_t resp[64];
  if (!rfid.getResponse(resp, sizeof(resp), 500)) {
    return; // no tag
  }

  if (!rfid.hasValidTag(resp)) {
    return;
  }

  uint8_t rssi;
  uint8_t epc[12];
  rfid.parseTagResponse(resp, rssi, epc);

  Serial.print("TAG EPC: ");
  for (int i = 0; i < 12; i++) {
    Serial.print(epc[i], HEX);
    Serial.print(" ");
  }
  Serial.print(" RSSI=");
  Serial.println(rssi);
}

void doWrite(String cmd) {
  Serial.println("doWrite MIN");

  // Strip leading "/w"
  cmd.trim();
  if (!cmd.startsWith("/w")) {
    Serial.println("ERR: bad cmd");
    return;
  }
  cmd.remove(0, 2);
  cmd.trim();

  // Split tokens
  int firstSpace = cmd.indexOf(' ');
  int secondSpace = cmd.indexOf(' ', firstSpace + 1);
  int thirdSpace  = cmd.indexOf(' ', secondSpace + 1);

  if (firstSpace < 0 || secondSpace < 0 || thirdSpace < 0) {
    Serial.println("ERR: Format /w bank addr len d0 d1 ...");
    return;
  }

  uint8_t  bank = (uint8_t) cmd.substring(0, firstSpace).toInt();
  uint16_t addr = (uint16_t)cmd.substring(firstSpace + 1, secondSpace).toInt();
  uint16_t len  = (uint16_t)cmd.substring(secondSpace + 1, thirdSpace).toInt();

  Serial.print("bank="); Serial.print(bank);
  Serial.print(" addr="); Serial.print(addr);
  Serial.print(" len=");  Serial.println(len);

  uint8_t data[64];
  memset(data, 0, sizeof(data));

  int index = 0;
  int pos = thirdSpace + 1;
  while (pos < cmd.length() && index < len) {
    int nextSpace = cmd.indexOf(' ', pos);
    String token = (nextSpace == -1) ? cmd.substring(pos)
                                     : cmd.substring(pos, nextSpace);
    token.trim();
    if (token.length() > 0) {
      int value = (int) strtol(token.c_str(), nullptr, 16);
      data[index++] = (uint8_t)value;
    }
    if (nextSpace == -1) break;
    pos = nextSpace + 1;
  }

  Serial.print("parsed bytes="); Serial.println(index);
  if (index < len) {
    Serial.println("ERR: Not enough data bytes");
    return;
  }

  uint32_t accessPassword = 0x00000000;
  Serial.println("calling writeTagData()");
  rfid.writeTagData(accessPassword, bank, addr, len, data);
  Serial.println("Write command sent");
}

void doWrite()
{
    Serial.println("doWrite MIN");

    // Example: /w 1 2 12  34 00 00 00 11 0B 5B 00 B5 B0 0B 5B
    // bank=1 (EPC), addr=2, len=12 (bytes)

    uint8_t bank;
    uint16_t addr;
    uint16_t len;

    if (!readNextUint8(&bank))  { Serial.println("ERR: missing bank"); return; }
    if (!readNextUint16(&addr)) { Serial.println("ERR: missing addr"); return; }
    if (!readNextUint16(&len))  { Serial.println("ERR: missing len");  return; }

    Serial.print("bank="); Serial.print(bank);
    Serial.print(" addr="); Serial.print(addr);
    Serial.print(" len=");  Serial.println(len);

    if (len != 12) {
        Serial.println("ERR: EPC must be exactly 12 bytes");
        return;
    }

    uint8_t epc[12];
    int parsed = 0;
    while (parsed < 12 && readNextHexByte(&epc[parsed])) {
        parsed++;
    }

    Serial.print("parsed bytes="); Serial.println(parsed);
    if (parsed != 12) {
        Serial.println("ERR: not enough EPC bytes");
        return;
    }

    Serial.println("calling writeTagData()");

    uint32_t accessPassword = 0x00000000;  // or your real password

    bool ok = rfid.writeTagData(accessPassword, bank, addr, len, epc);

    if (ok) {
        Serial.println("HYBRID WRITE: SUCCESS");
    } else {
        Serial.println("HYBRID WRITE: FAILED");
    }
}

