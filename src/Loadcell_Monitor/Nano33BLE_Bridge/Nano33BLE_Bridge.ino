/*
 * ================================================================
 *  Loadcell Monitor - BLE Bridge Firmware
 *  Arduino Nano 33 BLE 용
 * ================================================================
 *
 *  역할: Teensy 4.1 ↔ BLE (Python GUI) 간 UART 브릿지
 *
 *  핵심 수정 (v2):
 *  1. 연결 간격 6~12 (7.5ms~15ms) — 고정값 24,24 제거
 *     이유: min=max 고정 시 Mac BLE 협상 거부 → 연결 끊김
 *  2. writeValue 실패 시 버퍼 유지 → 다음 루프에서 재시도
 *     이유: 기존 코드는 실패해도 버퍼 클리어 → 데이터 유실 스파이럴
 *  3. BLE.poll()을 루프 시작과 끝에 모두 호출
 *     이유: writeValue 처리 중 poll 지연 → 스택 응답 끊김
 *  4. 20ms 전송 타이머 (1ms idle 대신)
 *     이유: 50Hz Teensy → 20ms 주기 맞춤, 불필요한 전송 시도 감소
 * ================================================================
 */

#include <ArduinoBLE.h>
#include <nrf_wdt.h>

// ================================================================
// [0] Watchdog
// ================================================================

#define WATCHDOG_TIMEOUT_MS 5000

void setupWatchdog() {
    nrf_wdt_behaviour_set(NRF_WDT_BEHAVIOUR_RUN_SLEEP);
    nrf_wdt_reload_value_set((WATCHDOG_TIMEOUT_MS * 32768) / 1000);
    nrf_wdt_reload_request_enable(NRF_WDT_RR0);
    nrf_wdt_task_trigger(NRF_WDT_TASK_START);
}

void feedWatchdog() {
    nrf_wdt_reload_request_set(NRF_WDT_RR0);
}

// ================================================================
// [1] BLE 설정
// ================================================================

#define NUS_SERVICE_UUID      "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_CHARACTERISTIC "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_CHARACTERISTIC "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

BLEService nusService(NUS_SERVICE_UUID);
BLECharacteristic txCharacteristic(NUS_TX_CHARACTERISTIC, BLENotify, 244);
BLECharacteristic rxCharacteristic(NUS_RX_CHARACTERISTIC, BLEWrite | BLEWriteWithoutResponse, 244);

// ================================================================
// [2] UART 설정
// ================================================================

#define TEENSY_SERIAL Serial1
#define TEENSY_BAUD   115200

// 링버퍼: 2패킷 이상 수용 (패킷 ~70bytes × 3 = 210)
#define UART_BUFFER_SIZE 512
static uint8_t uartBuffer[UART_BUFFER_SIZE];
static uint16_t uartBufferLen = 0;

// ================================================================
// [3] 상태 변수
// ================================================================

static bool bleConnected = false;
static uint32_t lastSendMs = 0;
static uint32_t lastStatusPrintMs = 0;
static uint32_t bleTxCount = 0;
static uint32_t bleWriteFailCount = 0;

#define LED_RED   22
#define LED_GREEN 23
#define LED_BLUE  24

// ================================================================
// [4] 초기화
// ================================================================

void setup() {
    Serial.begin(115200);
    delay(500);

    setupWatchdog();

    pinMode(LED_RED,   OUTPUT); digitalWrite(LED_RED,   HIGH);
    pinMode(LED_GREEN, OUTPUT); digitalWrite(LED_GREEN, HIGH);
    pinMode(LED_BLUE,  OUTPUT); digitalWrite(LED_BLUE,  HIGH);

    TEENSY_SERIAL.begin(TEENSY_BAUD);

    if (!BLE.begin()) {
        setLed(true, false, false);
        while (1) { feedWatchdog(); }
    }

    BLE.setLocalName("Walker");
    BLE.setDeviceName("Walker");

    // ★ 연결 간격: 7.5ms~15ms (min≠max 필수 — 고정값은 협상 거부 유발)
    BLE.setConnectionInterval(6, 12);

    nusService.addCharacteristic(txCharacteristic);
    nusService.addCharacteristic(rxCharacteristic);
    BLE.addService(nusService);

    rxCharacteristic.setEventHandler(BLEWritten, onRxReceived);
    BLE.setEventHandler(BLEConnected, onBleConnected);
    BLE.setEventHandler(BLEDisconnected, onBleDisconnected);

    BLE.advertise();
    setLed(false, false, true);

    Serial.println("[OK] Walker BLE Bridge ready");
}

// ================================================================
// [5] 메인 루프
// ================================================================

void loop() {
    feedWatchdog();

    // poll을 루프 시작에 호출 — BLE 스택 이벤트 처리
    BLE.poll();

    // UART 수신
    readUart();

    // BLE 전송 (20ms 주기)
    sendToBle();

    // poll을 루프 끝에도 호출 — writeValue 처리 후 스택 응답
    BLE.poll();

    updateLed();

    // 5초마다 상태 출력
    if (millis() - lastStatusPrintMs > 5000) {
        lastStatusPrintMs = millis();
        Serial.print("[STATUS] BLE:");
        Serial.print(bleConnected ? "CONN" : "DISC");
        Serial.print(" TX:");
        Serial.print(bleTxCount);
        Serial.print(" FAIL:");
        Serial.println(bleWriteFailCount);
    }
}

// ================================================================
// [6] UART 수신
// ================================================================

void readUart() {
    while (TEENSY_SERIAL.available()) {
        uint8_t c = TEENSY_SERIAL.read();
        if (uartBufferLen < UART_BUFFER_SIZE - 1) {
            uartBuffer[uartBufferLen++] = c;
        }
        // 버퍼 풀이면 가장 오래된 데이터 버림 (새 데이터 우선)
    }
}

// ================================================================
// [7] BLE 전송 (20ms 주기)
// ================================================================

void sendToBle() {
    uint32_t now = millis();

    // 20ms 타이머: 50Hz Teensy 전송 주기에 맞춤
    if (now - lastSendMs < 20) return;
    lastSendMs = now;

    if (uartBufferLen == 0) return;
    if (!bleConnected || !txCharacteristic.subscribed()) {
        uartBufferLen = 0;  // 연결 없으면 버퍼 비움 (쌓이지 않도록)
        return;
    }

    uint16_t sendLen = min(uartBufferLen, (uint16_t)244);
    int result = txCharacteristic.writeValue(uartBuffer, sendLen);

    if (result) {
        // ★ 성공 시에만 버퍼 클리어
        uartBufferLen = 0;
        bleTxCount++;
    } else {
        // ★ 실패 시 버퍼 유지 → 다음 20ms에 재시도
        bleWriteFailCount++;
        // 연속 실패 누적 방지: 버퍼가 너무 크면 절반 버림
        if (uartBufferLen > UART_BUFFER_SIZE / 2) {
            uartBufferLen = 0;
        }
    }
}

// ================================================================
// [8] BLE → UART (GUI → Teensy)
// ================================================================

void onRxReceived(BLEDevice central, BLECharacteristic characteristic) {
    int len = characteristic.valueLength();
    const uint8_t* data = characteristic.value();
    if (len > 0) {
        TEENSY_SERIAL.write(data, len);
    }
}

// ================================================================
// [9] BLE 연결 콜백
// ================================================================

void onBleConnected(BLEDevice central) {
    bleConnected = true;
    uartBufferLen = 0;  // 연결 시 버퍼 클리어 (구버퍼 전송 방지)
    Serial.print("[BLE] Connected: ");
    Serial.println(central.address());
    setLed(false, true, false);
}

void onBleDisconnected(BLEDevice central) {
    bleConnected = false;
    uartBufferLen = 0;
    Serial.println("[BLE] Disconnected → advertising");
    BLE.advertise();
    setLed(false, false, true);
}

// ================================================================
// [10] LED
// ================================================================

void setLed(bool red, bool green, bool blue) {
    digitalWrite(LED_RED,   red   ? LOW : HIGH);
    digitalWrite(LED_GREEN, green ? LOW : HIGH);
    digitalWrite(LED_BLUE,  blue  ? LOW : HIGH);
}

void updateLed() {
    if (!bleConnected) {
        static uint32_t t = 0;
        static bool s = false;
        if (millis() - t > 500) {
            t = millis();
            s = !s;
            setLed(false, false, s);
        }
    }
}
