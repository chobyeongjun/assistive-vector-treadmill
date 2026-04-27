/*
 * ================================================================
 *  BLE Communication Module for Walker
 *  Implementation File
 * ================================================================
 */

#include "BleComm.h"

// ================================================================
// [1] 전역 변수 정의
// ================================================================

volatile uint32_t currentMark = 0;
volatile bool bleStreamEnabled = false;

char bleRxBuffer[128];
uint8_t bleRxLen = 0;
uint32_t bleLastRxMs = 0;  // 마지막 BLE 수신 시각 (워치독용)

// ================================================================
// [2] BLE Serial 초기화
// ================================================================

void setupBleComm() {
    BLE_SERIAL.begin(BLE_BAUD_RATE);

    // 버퍼 초기화
    bleRxLen = 0;
    memset(bleRxBuffer, 0, sizeof(bleRxBuffer));

    Serial.println("✅ BLE Serial initialized (Serial2 @ 115200 baud)");
    Serial.println("┌─────────────────────────────────────────┐");
    Serial.println("│  BLE UART Bridge Connection             │");
    Serial.println("├─────────────────────────────────────────┤");
    Serial.print("│  Teensy Pin ");
    Serial.print(BLE_TX_PIN);
    Serial.println(" (TX2) → Nano RX (D0)     │");
    Serial.print("│  Teensy Pin ");
    Serial.print(BLE_RX_PIN);
    Serial.println(" (RX2) ← Nano TX (D1)     │");
    Serial.println("│  GND ─────────────── GND                │");
    Serial.println("└─────────────────────────────────────────┘");
}

// ================================================================
// [3] Walker 데이터 전송
// ================================================================

// ★ 최적화: 단일 버퍼에 모든 데이터를 포맷팅 후 한 번에 전송
static char bleTxBuffer[256];

void sendWalkerDataToBLE(
    float l_gcp, float r_gcp,
    float l_pitch, float r_pitch,
    float l_gyro_y, float r_gyro_y,
    float l_des_force, float r_des_force,
    float l_act_force, float r_act_force,
    uint32_t mark
) {
    if (!bleStreamEnabled) return;

    // SW11c<d0>n...<d10>n
    // [0-1] GCP, [2-3] Pitch, [4-5] GyroY, [6-7] DesForce, [8-9] ActForce, [10] Mark
    int len = snprintf(bleTxBuffer, sizeof(bleTxBuffer),
        "SW%dc%dn%dn%dn%dn%dn%dn%dn%dn%dn%dn%dn",
        WALKER_DATA_COUNT,
        (int)(l_gcp * 100.0f),        // 0: L_GCP
        (int)(r_gcp * 100.0f),        // 1: R_GCP
        (int)(l_pitch * 100.0f),      // 2: L_Pitch
        (int)(r_pitch * 100.0f),      // 3: R_Pitch
        (int)(l_gyro_y * 100.0f),     // 4: L_GyroY
        (int)(r_gyro_y * 100.0f),     // 5: R_GyroY
        (int)(l_des_force * 100.0f),  // 6: L_DesForce
        (int)(r_des_force * 100.0f),  // 7: R_DesForce
        (int)(l_act_force * 100.0f),  // 8: L_ActForce
        (int)(r_act_force * 100.0f),  // 9: R_ActForce
        (int)(mark * 100)             // 10: Mark
    );

    // TX 버퍼 여유가 충분할 때만 전송 — 버퍼 풀 시 블로킹 방지
    // 블로킹하면 loop()가 멈춰 processBleSerial() 지연 → BLE 명령 처리 늦어짐
    if (len > 0 && len < (int)sizeof(bleTxBuffer)) {
        if (BLE_SERIAL.availableForWrite() >= len) {
            BLE_SERIAL.write(bleTxBuffer, len);
        }
        // else: 이 패킷 skip — 다음 20ms에 최신 데이터로 재전송
    }
}

// ================================================================
// [3-3] 펌웨어 → GUI 응답 전송
// ================================================================

static char bleRespBuffer[64];

void sendBleResponse(const char* msg) {
    // 패킷 포맷: "SR:<message>\n"
    // GUI에서 "SR:" 프리픽스로 응답 패킷 식별
    int len = snprintf(bleRespBuffer, sizeof(bleRespBuffer), "SR:%s\n", msg);
    if (len > 0 && len < (int)sizeof(bleRespBuffer)) {
        if (BLE_SERIAL.availableForWrite() >= len) {
            BLE_SERIAL.write(bleRespBuffer, len);
        }
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
        bleLastRxMs = lastRxMs;  // 워치독 타이머 갱신

        if (ch == '\r' || ch == '\n') {
            if (bleRxLen > 0) {
                bleRxBuffer[bleRxLen] = '\0';
                // 소문자 변환 — handleBleCommand에서 String.toLowerCase() 불필요
                for (uint8_t i = 0; i < bleRxLen; i++)
                    bleRxBuffer[i] = tolower((unsigned char)bleRxBuffer[i]);
                handleBleCommand(bleRxBuffer);
                bleRxLen = 0;
                commandProcessed = true;
            }
        } else {
            if (bleRxLen < sizeof(bleRxBuffer) - 1) {
                bleRxBuffer[bleRxLen++] = ch;
            } else {
                // 버퍼 오버플로 방지
                bleRxLen = 0;
            }
        }
    }

    // ★ 타임아웃: 50ms (BLE 전송 지연 고려, newline 누락 시 안전망)
    if (bleRxLen > 0 && (millis() - lastRxMs) > 50) {
        bleRxBuffer[bleRxLen] = '\0';
        for (uint8_t i = 0; i < bleRxLen; i++)
            bleRxBuffer[i] = tolower((unsigned char)bleRxBuffer[i]);
        handleBleCommand(bleRxBuffer);
        bleRxLen = 0;
        commandProcessed = true;
    }

    return commandProcessed;
}
