/*
 * ================================================================
 *  Loadcell Monitor - BLE Bridge Firmware
 *  Arduino Nano 33 BLE 용
 * ================================================================
 *
 *  역할: Teensy 4.1 ↔ BLE (Python GUI) 간 UART 브릿지
 *        (WalkerBLE_Nano 기반, Loadcell Monitor 전용)
 *
 *  통신 구조:
 *    [Teensy 4.1] ←─Serial1 (115200)─→ [Nano 33 BLE] ←─BLE─→ [Python GUI]
 *
 *  하드웨어 연결:
 *    Teensy Pin 35 (TX8) → Nano D0 (RX / Serial1 RX)
 *    Teensy Pin 34 (RX8) ← Nano D1 (TX / Serial1 TX)
 *    GND ─────────────── GND
 *
 *  BLE Service: Nordic UART Service (NUS)
 *    - Service UUID: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 *    - TX Char UUID: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E (Notify)
 *    - RX Char UUID: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E (Write)
 *
 * ================================================================
 */

#include <ArduinoBLE.h>
#include <nrf_wdt.h>

// ================================================================
// [0] Watchdog 설정
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

#define NUS_SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_CHARACTERISTIC   "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_CHARACTERISTIC   "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

BLEService nusService(NUS_SERVICE_UUID);
BLECharacteristic txCharacteristic(NUS_TX_CHARACTERISTIC, BLENotify, 244);
BLECharacteristic rxCharacteristic(NUS_RX_CHARACTERISTIC, BLEWrite | BLEWriteWithoutResponse, 244);

// ================================================================
// [2] UART 설정
// ================================================================

#define TEENSY_SERIAL Serial1
#define TEENSY_BAUD   115200

#define UART_BUFFER_SIZE 256
char uartBuffer[UART_BUFFER_SIZE];
uint16_t uartBufferLen = 0;

// ================================================================
// [3] 상태 변수
// ================================================================

bool bleConnected = false;
uint32_t lastActivityMs = 0;
uint32_t ledToggleMs = 0;

// 디버그 카운터
uint32_t lastStatusPrintMs = 0;
uint32_t uartRxCount = 0;
uint32_t bleTxCount = 0;
uint32_t bleWriteFailCount = 0;
uint32_t bleWriteSuccessCount = 0;
uint32_t notSubscribedCount = 0;
uint32_t lastUartRxTime = 0;

// LED 핀 (Nano 33 BLE 내장 RGB)
#define LED_BUILTIN_RED   22
#define LED_BUILTIN_GREEN 23
#define LED_BUILTIN_BLUE  24

// ================================================================
// [4] 초기화
// ================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("================================");
    Serial.println("  Loadcell BLE Bridge");
    Serial.println("================================");

    setupWatchdog();

    // LED 초기화 (Active Low)
    pinMode(LED_BUILTIN_RED, OUTPUT);
    pinMode(LED_BUILTIN_GREEN, OUTPUT);
    pinMode(LED_BUILTIN_BLUE, OUTPUT);
    digitalWrite(LED_BUILTIN_RED, HIGH);
    digitalWrite(LED_BUILTIN_GREEN, HIGH);
    digitalWrite(LED_BUILTIN_BLUE, HIGH);

    // UART 초기화
    TEENSY_SERIAL.begin(TEENSY_BAUD);
    Serial.println("[OK] UART (Serial1 @ 115200)");

    // BLE 초기화
    if (!BLE.begin()) {
        Serial.println("[ERROR] BLE init failed!");
        setLedColor(true, false, false);
        while (1);
    }

    // ★ 디바이스 이름: Python GUI에서 "Walker"로 검색
    BLE.setLocalName("Walker");
    BLE.setDeviceName("Walker");

    // 연결 간격: 30ms (안정적)
    BLE.setConnectionInterval(24, 24);

    // NUS 서비스 등록
    nusService.addCharacteristic(txCharacteristic);
    nusService.addCharacteristic(rxCharacteristic);
    BLE.addService(nusService);

    // 콜백 등록
    rxCharacteristic.setEventHandler(BLEWritten, onRxReceived);
    BLE.setEventHandler(BLEConnected, onBleConnected);
    BLE.setEventHandler(BLEDisconnected, onBleDisconnected);

    // Advertising 시작
    BLE.advertise();
    Serial.println("[OK] BLE advertising: Walker");
    Serial.println("================================");

    setLedColor(false, false, true);  // Blue = 대기
}

// ================================================================
// [5] 메인 루프
// ================================================================

void loop() {
    feedWatchdog();
    BLE.poll();
    processUartToBle();
    updateLed();

    // 2초마다 상태 출력
    if (millis() - lastStatusPrintMs > 2000) {
        lastStatusPrintMs = millis();
        Serial.print("[STATUS] BLE:");
        Serial.print(bleConnected ? "CONN" : "DISC");
        Serial.print(" Sub:");
        Serial.print(txCharacteristic.subscribed() ? "Y" : "N");
        Serial.print(" | UART_RX:");
        Serial.print(uartRxCount);
        Serial.print(" | BLE_TX:");
        Serial.print(bleWriteSuccessCount);
        Serial.print(" FAIL:");
        Serial.println(bleWriteFailCount);
    }
}

// ================================================================
// [6] UART → BLE (Teensy → GUI)
// ================================================================

void processUartToBle() {
    while (TEENSY_SERIAL.available()) {
        char c = TEENSY_SERIAL.read();
        if (uartBufferLen < UART_BUFFER_SIZE - 1) {
            uartBuffer[uartBufferLen++] = c;
        }
        lastActivityMs = millis();
        lastUartRxTime = millis();
        uartRxCount++;
    }

    // 버퍼에 데이터가 있고 1ms 이상 추가 수신 없으면 전송
    if (uartBufferLen > 0 && (millis() - lastActivityMs > 1)) {
        if (bleConnected && txCharacteristic.subscribed()) {
            uint16_t sendLen = min(uartBufferLen, (uint16_t)244);
            int result = txCharacteristic.writeValue((uint8_t*)uartBuffer, sendLen);

            if (result) {
                bleWriteSuccessCount++;
                bleTxCount++;
            } else {
                bleWriteFailCount++;
            }
        }
        uartBufferLen = 0;
    }
}

// ================================================================
// [7] BLE → UART (GUI → Teensy)
// ================================================================

void onRxReceived(BLEDevice central, BLECharacteristic characteristic) {
    int len = characteristic.valueLength();
    const uint8_t* data = characteristic.value();

    if (len > 0) {
        TEENSY_SERIAL.write(data, len);

        Serial.print("[BLE->UART] ");
        for (int i = 0; i < len; i++) {
            Serial.print((char)data[i]);
        }
        Serial.println();
    }
}

// ================================================================
// [8] BLE 연결 콜백
// ================================================================

void onBleConnected(BLEDevice central) {
    bleConnected = true;
    Serial.print("[BLE] Connected: ");
    Serial.println(central.address());
    setLedColor(false, true, false);  // Green = 연결됨
}

void onBleDisconnected(BLEDevice central) {
    bleConnected = false;
    Serial.print("[BLE] Disconnected: ");
    Serial.println(central.address());
    BLE.advertise();
    setLedColor(false, false, true);  // Blue = 대기
}

// ================================================================
// [9] LED 제어
// ================================================================

void setLedColor(bool red, bool green, bool blue) {
    digitalWrite(LED_BUILTIN_RED, red ? LOW : HIGH);
    digitalWrite(LED_BUILTIN_GREEN, green ? LOW : HIGH);
    digitalWrite(LED_BUILTIN_BLUE, blue ? LOW : HIGH);
}

void updateLed() {
    if (!bleConnected) {
        if (millis() - ledToggleMs > 500) {
            ledToggleMs = millis();
            static bool ledState = false;
            ledState = !ledState;
            setLedColor(false, false, ledState);
        }
    }
}
