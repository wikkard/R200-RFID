#include "RFIDR200.h"

// Constructor for HardwareSerial
RFIDR200::RFIDR200(HardwareSerial &serial, uint32_t baudRate)
    : serial(serial), baudRate(baudRate), isHardwareSerial(true) {}

// Constructor for SoftwareSerial (disabled)
// RFIDR200::RFIDR200(SoftwareSerial &serial, uint32_t baudRate)
//     : serial(serial), baudRate(baudRate), isHardwareSerial(false) {}

void RFIDR200::begin() {
    if (isHardwareSerial) {
        ((HardwareSerial&)serial).begin(baudRate);
    } else {
        // ((SoftwareSerial&)serial).begin(baudRate);
    }
}

/* ---------------------------------------------------------
   INTERNAL: Corrected frame reader
   --------------------------------------------------------- */
bool RFIDR200::getResponse(uint8_t *buffer, size_t maxLen, uint32_t timeout) {
    uint32_t start = millis();
    size_t idx = 0;
    int frameLen = -1;

    while (millis() - start < timeout) {
        if (!serial.available()) continue;

        uint8_t b = serial.read();

        // Wait for header
        if (idx == 0 && b != 0xAA) continue;

        buffer[idx++] = b;

        // Determine full frame length once PL is known
        if (idx == 5 && frameLen == -1) {
            uint16_t PL = (buffer[3] << 8) | buffer[4];
            frameLen = 5 + PL + 2;   // header + payload + checksum + tail
        }

        // Stop exactly at frame boundary
        if (frameLen != -1 && idx >= frameLen) {
            return true;
        }

        // Prevent overflow
        if (idx >= maxLen) {
            return true;
        }
    }

    return false; // timeout
}

/* ---------------------------------------------------------
   INTERNAL: Correct EPC parsing
   --------------------------------------------------------- */
void RFIDR200::parseTagResponse(uint8_t* response, uint8_t& rssi, uint8_t (&epc)[12]) {
    rssi = response[5];

    uint16_t pc = (response[6] << 8) | response[7];
    uint8_t words = (pc >> 11) & 0x1F;
    uint8_t epcLen = words * 2;

    if (epcLen > 12) epcLen = 12; // safety clamp

    memcpy(epc, &response[8], epcLen);
}

/* ---------------------------------------------------------
   COMMANDS
   --------------------------------------------------------- */

void RFIDR200::setTransmitPower(uint16_t power) {
    uint8_t command[] = {
        0xAA, 0x00, 0xB6, 0x00, 0x02,
        (power >> 8) & 0xFF, power & 0xFF,
        0x00, 0xDD
    };
    command[7] = calculateChecksum(command, 7);
    sendCommand(command, sizeof(command));
}
bool RFIDR200::writeTagData(uint32_t accessPassword, uint8_t memBank,
                            uint16_t address, uint16_t lengthBytes, uint8_t *epc)
{
    Serial.println("WRITE START (HYBRID)");

    if (lengthBytes != 12) {
        Serial.println("ERR: EPC must be exactly 12 bytes");
        return false;
    }

    // -------- STEP 1: PRE‑WRITE READ WITH RETRIES --------
    uint8_t readResp[64];
    uint8_t stablePC1 = 0, stablePC2 = 0;
    uint8_t stableCRC1 = 0, stableCRC2 = 0;

    const int MAX_READ_RETRIES = 5;
    bool gotStableRead = false;

    for (int attempt = 1; attempt <= MAX_READ_RETRIES; attempt++) {

        initiateSinglePolling();
        memset(readResp, 0, sizeof(readResp));

        if (!getResponse(readResp, sizeof(readResp), 500)) {
            Serial.print("Pre‑read attempt "); Serial.print(attempt);
            Serial.println(": No response");
            continue;
        }

        if (!hasValidTag(readResp)) {
            Serial.print("Pre‑read attempt "); Serial.print(attempt);
            Serial.println(": Invalid tag");
            continue;
        }

        uint16_t PL = (readResp[3] << 8) | readResp[4];
        stablePC1 = readResp[6];
        stablePC2 = readResp[7];
        stableCRC1 = readResp[5 + PL - 2];
        stableCRC2 = readResp[5 + PL - 1];

        Serial.print("Pre‑read OK on attempt "); Serial.println(attempt);
        gotStableRead = true;
        break;
    }

    if (!gotStableRead) {
        Serial.println("ERR: Could not read tag before write");
        return false;
    }

    // -------- STEP 2: FIX PC BITS FOR 12‑BYTE EPC (6 words) --------
    uint16_t oldPC = (stablePC1 << 8) | stablePC2;
    uint16_t newPC = (oldPC & 0x07FF) | (6 << 11);  // 6 words

    uint8_t newPC1 = (newPC >> 8) & 0xFF;
    uint8_t newPC2 = newPC & 0xFF;

    // -------- STEP 3: BUILD WRITE FRAME --------
    uint8_t frame[32];

    frame[0] = 0xAA;
    frame[1] = 0x00;
    frame[2] = 0x49;
    frame[3] = 0x00;
    frame[4] = 0x19;

    frame[5] = (accessPassword >> 24) & 0xFF;
    frame[6] = (accessPassword >> 16) & 0xFF;
    frame[7] = (accessPassword >> 8)  & 0xFF;
    frame[8] =  accessPassword        & 0xFF;

    frame[9]  = memBank;
    frame[10] = (address >> 8) & 0xFF;
    frame[11] =  address       & 0xFF;

    frame[12] = 0x00;
    frame[13] = 0x06;  // 6 words = 12 bytes

    frame[14] = newPC1;
    frame[15] = newPC2;

    for (int i = 0; i < 12; i++)
        frame[16 + i] = epc[i];

    frame[28] = 0x00;
    frame[29] = 0x00;

    uint16_t sum = 0;
    for (int i = 1; i < 30; i++) sum += frame[i];
    frame[30] = sum & 0xFF;
    frame[31] = 0xDD;

    Serial.print("WRITE FRAME: ");
    for (int i = 0; i < 32; i++) {
        Serial.print(frame[i], HEX); Serial.print(" ");
    }
    Serial.println();

    // -------- STEP 4: SEND WRITE COMMAND --------
    sendCommand(frame, 32);

    uint8_t writeResp[64];
    memset(writeResp, 0, sizeof(writeResp));

    if (!getResponse(writeResp, sizeof(writeResp), 500)) {
        Serial.println("WRITE ERROR: No response");
        return false;
    }

    // -------- STEP 5: POST‑WRITE VERIFY WITH RETRIES --------
    const int MAX_VERIFY_RETRIES = 5;
    uint8_t verifyResp[64];

    for (int attempt = 1; attempt <= MAX_VERIFY_RETRIES; attempt++) {

        delay(20);  // let tag settle

        initiateSinglePolling();
        memset(verifyResp, 0, sizeof(verifyResp));

        if (!getResponse(verifyResp, sizeof(verifyResp), 500)) {
            Serial.print("Verify attempt "); Serial.print(attempt);
            Serial.println(": No response");
            continue;
        }

        if (!hasValidTag(verifyResp)) {
            Serial.print("Verify attempt "); Serial.print(attempt);
            Serial.println(": Invalid tag");
            continue;
        }

        uint8_t *epcVerify = &verifyResp[8];  // EPC starts at byte 8

        bool match = true;
        for (int i = 0; i < 12; i++) {
            if (epcVerify[i] != epc[i]) {
                match = false;
                break;
            }
        }

        if (match) {
            Serial.println("WRITE OK (verified)");
            return true;
        }

        Serial.print("Verify attempt "); Serial.print(attempt);
        Serial.println(": EPC mismatch");
    }

    Serial.println("WRITE FAILED: Could not verify EPC");
    return false;
}




void RFIDR200::resetComms()
{
    // Flush Arduino RX buffer
    while (serial.available()) serial.read();

    // Stop any stuck polling
    uint8_t stopCmd[] = {0xAA,0x00,0x28,0x00,0x00,0x28,0xDD};
    serial.write(stopCmd, sizeof(stopCmd));
    delay(50);

    // Force idle mode
    uint8_t idleCmd[] = {0xAA,0x00,0x04,0x00,0x01,0x00,0x05,0xDD};
    serial.write(idleCmd, sizeof(idleCmd));
    delay(50);

    // Flush again
    while (serial.available()) serial.read();

    Serial.println("RFID module comms reset");
}


void RFIDR200::initiateSinglePolling() {
    uint8_t command[] = {0xAA, 0x00, 0x22, 0x00, 0x00, 0x22, 0xDD};
    sendCommand(command, sizeof(command));
}

void RFIDR200::initiateMultiplePolling(uint16_t count) {
    uint8_t command[] = {
        0xAA, 0x00, 0x27, 0x00, 0x03,
        0x22, (count >> 8) & 0xFF, count & 0xFF,
        0x00, 0xDD
    };
    command[8] = calculateChecksum(command, 8);
    sendCommand(command, sizeof(command));
}

void RFIDR200::stopMultiplePolling() {
    uint8_t command[] = {0xAA, 0x00, 0x28, 0x00, 0x00, 0x28, 0xDD};
    sendCommand(command, sizeof(command));
}

void RFIDR200::setSelectParameter(uint8_t target, uint8_t action, uint8_t memBank,
                                  uint32_t pointer, uint8_t maskLength, uint8_t *mask) {

    uint8_t command[22 + maskLength];

    command[0] = 0xAA;
    command[1] = 0x00;
    command[2] = 0x0C;
    command[3] = 0x00;
    command[4] = 0x13;

    command[5] = target;
    command[6] = action;
    command[7] = memBank;

    command[8]  = (pointer >> 24) & 0xFF;
    command[9]  = (pointer >> 16) & 0xFF;
    command[10] = (pointer >> 8) & 0xFF;
    command[11] = pointer & 0xFF;

    command[12] = maskLength;
    command[13] = 0x00;

    memcpy(&command[14], mask, maskLength);

    command[14 + maskLength] = calculateChecksum(command, 14 + maskLength);
    command[15 + maskLength] = 0xDD;

    sendCommand(command, 16 + maskLength);
}

void RFIDR200::getSelectParameter(uint8_t *response, size_t length) {
    uint8_t command[] = {0xAA, 0x00, 0x0B, 0x00, 0x00, 0x0B, 0xDD};
    sendCommand(command, sizeof(command));
    getResponse(response, length);
}

void RFIDR200::killTag(uint32_t killPassword) {
    uint8_t command[] = {
        0xAA, 0x00, 0x65, 0x00, 0x04,
        (killPassword >> 24) & 0xFF,
        (killPassword >> 16) & 0xFF,
        (killPassword >> 8) & 0xFF,
        killPassword & 0xFF,
        0x00, 0xDD
    };
    command[9] = calculateChecksum(command, 9);
    sendCommand(command, sizeof(command));
}

void RFIDR200::lockTag(uint32_t accessPassword, uint16_t lockPayload) {
    uint8_t command[] = {
        0xAA, 0x00, 0x82, 0x00, 0x07,
        (accessPassword >> 24) & 0xFF,
        (accessPassword >> 16) & 0xFF,
        (accessPassword >> 8) & 0xFF,
        accessPassword & 0xFF,
        (lockPayload >> 8) & 0xFF,
        lockPayload & 0xFF,
        0x00, 0xDD
    };
    command[11] = calculateChecksum(command, 11);
    sendCommand(command, sizeof(command));
}

void RFIDR200::setWorkArea(uint8_t region) {
    uint8_t command[] = {0xAA, 0x00, 0x07, 0x00, 0x01, region, 0x00, 0xDD};
    command[6] = calculateChecksum(command, 6);
    sendCommand(command, sizeof(command));
}

void RFIDR200::getWorkArea(uint8_t *response, size_t length) {
    uint8_t command[] = {0xAA, 0x00, 0x08, 0x00, 0x00, 0x08, 0xDD};
    sendCommand(command, sizeof(command));
    getResponse(response, length);
}

void RFIDR200::setWorkingChannel(uint8_t channelIndex) {
    uint8_t command[] = {0xAA, 0x00, 0xAB, 0x00, 0x01, channelIndex, 0x00, 0xDD};
    command[6] = calculateChecksum(command, 6);
    sendCommand(command, sizeof(command));
}

void RFIDR200::getWorkingChannel(uint8_t *response, size_t length) {
    uint8_t command[] = {0xAA, 0x00, 0xAA, 0x00, 0x00, 0xAA, 0xDD};
    sendCommand(command, sizeof(command));
    getResponse(response, length);
}

void RFIDR200::setAutomaticFrequencyHopping(bool enable) {
    uint8_t command[] = {0xAA, 0x00, 0xAD, 0x00, 0x01, enable ? 0xFF : 0x00, 0x00, 0xDD};
    command[6] = calculateChecksum(command, 6);
    sendCommand(command, sizeof(command));
}

void RFIDR200::insertWorkingChannel(uint8_t count, uint8_t *channelList) {
    uint8_t command[9 + count] = {
        0xAA, 0x00, 0xA9, 0x00, static_cast<uint8_t>(count + 1), count
    };

    memcpy(&command[6], channelList, count);

    command[6 + count] = calculateChecksum(command, 6 + count);
    command[7 + count] = 0xDD;

    sendCommand(command, 8 + count);
}

void RFIDR200::moduleSleep() {
    uint8_t command[] = {0xAA, 0x00, 0x17, 0x00, 0x00, 0x17, 0xDD};
    sendCommand(command, sizeof(command));
}

void RFIDR200::moduleIdleMode(bool enable) {
    uint8_t command[] = {0xAA, 0x00, 0x04, 0x00, 0x01, enable ? 0x01 : 0x00, 0x00, 0xDD};
    command[6] = calculateChecksum(command, 6);
    sendCommand(command, sizeof(command));
}

/* ---------------------------------------------------------
   ERROR HANDLING
   --------------------------------------------------------- */

bool RFIDR200::hasValidTag(uint8_t *response) {
    if (response[2] == 0xFF) {
        checkErrorCode(response[5]);
        return false;
    }
    return true;
}

int RFIDR200::checkErrorCode(uint8_t code) {
    switch (code) {
        case 0x15: return 1;
        case 0x16: Serial.println("Access password incorrect"); return 2;
        case 0x09: Serial.println("Read fail"); return 2;
        case 0x10: Serial.println("Write fail"); return 2;
        case 0xA0: Serial.println("Read error"); return 2;
        case 0xB0: Serial.println("Write error"); return 2;
        default:
            Serial.print("Unknown error: ");
            Serial.println(code, HEX);
            return 2;
    }
}

/* ---------------------------------------------------------
   INTERNAL UTILITIES
   --------------------------------------------------------- */

void RFIDR200::sendCommand(uint8_t *command, size_t length) {
    serial.write(command, length);
}

uint8_t RFIDR200::calculateChecksum(uint8_t *command, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = 1; i < length; i++) {
        checksum += command[i];
    }
    return checksum & 0xFF;
}
