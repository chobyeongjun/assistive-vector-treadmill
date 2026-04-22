/*
 * ================================================================
 *  BLE Communication Module for Loadcell Monitor
 *  Implementation File
 * ================================================================
 */

#include "BleComm.h"

// ================================================================
// [1] 전역 변수 정의
// ================================================================

volatile bool bleStreamEnabled = false;

char bleRxBuffer[128];
uint8_t bleRxLen = 0;

// ================================================================
// [2] BLE Serial 초기화
// ================================================================

void setupBleComm() {
    BLE_SERIAL.begin(BLE_BAUD_RATE);

    bleRxLen = 0;
    memset(bleRxBuffer, 0, sizeof(bleRxBuffer));

    Serial.println("BLE Serial initialized (Serial8 @ 115200 baud)");
    Serial.println("  Teensy Pin 35 (TX8) -> Nano RX (D0)");
    Serial.println("  Teensy Pin 34 (RX8) <- Nano TX (D1)");
}

// ================================================================
// [3] Loadcell 데이터 전송
// ================================================================

static char bleTxBuffer[64];

void sendLoadcellToBLE(float l_force, float r_force) {
    if (!bleStreamEnabled) return;

    // 패킷: "SL2c<L>n<R>n"
    int len = snprintf(bleTxBuffer, sizeof(bleTxBuffer),
        "SL%dc%dn%dn",
        LOADCELL_DATA_COUNT,
        (int)(l_force * 100.0f),   // L Force (N * 100)
        (int)(r_force * 100.0f)    // R Force (N * 100)
    );

    if (len > 0 && len < (int)sizeof(bleTxBuffer)) {
        BLE_SERIAL.write(bleTxBuffer, len);
    }
}

// ================================================================
// [3-2] 응답 전송
// ================================================================

static char bleRespBuffer[64];

void sendBleResponse(const char* msg) {
    int len = snprintf(bleRespBuffer, sizeof(bleRespBuffer), "SR:%s\n", msg);
    if (len > 0 && len < (int)sizeof(bleRespBuffer)) {
        BLE_SERIAL.write(bleRespBuffer, len);
    }
}

// ================================================================
// [4] BLE Serial 수신 처리
// ================================================================

bool processBleSerial() {
    static uint32_t lastRxMs = 0;
    bool commandProcessed = false;

    while (BLE_SERIAL.available()) {
        char ch = (char)BLE_SERIAL.read();
        lastRxMs = millis();

        if (ch == '\r' || ch == '\n') {
            if (bleRxLen > 0) {
                bleRxBuffer[bleRxLen] = '\0';
                handleBleCommand(String(bleRxBuffer));
                bleRxLen = 0;
                commandProcessed = true;
            }
        } else {
            if (bleRxLen < sizeof(bleRxBuffer) - 1) {
                bleRxBuffer[bleRxLen++] = ch;
            } else {
                bleRxLen = 0;
            }
        }
    }

    // 타임아웃: 50ms (newline 누락 시 안전망)
    if (bleRxLen > 0 && (millis() - lastRxMs) > 50) {
        bleRxBuffer[bleRxLen] = '\0';
        handleBleCommand(String(bleRxBuffer));
        bleRxLen = 0;
        commandProcessed = true;
    }

    return commandProcessed;
}
