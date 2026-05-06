/*
 * ================================================================
 *  Walker Treadmill - BLE Bridge Firmware
 *  Arduino Nano 33 BLE 용
 * ================================================================
 *
 *  역할: Walker Teensy 4.1 ↔ BLE (Python GUI) 간 UART 브릿지
 *  장치명: "Walker_BLE"
 *
 *  핵심 설계:
 *  1. 연결 간격 6~12 (7.5ms~15ms) — 고정값은 Mac BLE 협상 거부 유발
 *  2. writeValue 실패 시 오래된 패킷 폐기 → 최신 데이터 우선
 *  3. BLE.poll() 루프 시작+끝 이중 호출 — writeValue 중 스택 응답 유지
 *  4. 40ms 전송 타이머 + \n 경계 기반 complete packet 전송
 *     (partial packet → Python 파서 오염 → 플롯 정지 방지)
 *  5. blecon/bledis: BLE 연결/해제 시 Teensy에 알림
 *     Teensy bleStreamEnabled 플래그를 모터 상태와 독립적으로 제어
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

// 링버퍼: RF 차단 복구 시 백로그 전송 (패킷 ~90bytes × 45 = ~4KB)
// sendToBle()의 pre-trim이 244B cap을 보장하므로 실제 누적은 없음
#define UART_BUFFER_SIZE 4096
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
static uint32_t bleDropCount = 0;
static uint32_t uartOverflowCount = 0;

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

    BLE.setLocalName("Walker_BLE");
    BLE.setDeviceName("Walker_BLE");

    // 연결 간격: 7.5ms~15ms (짧을수록 body-shadowing 시 TX 버퍼 회복 빠름)
    BLE.setConnectionInterval(6, 12);

    nusService.addCharacteristic(txCharacteristic);
    nusService.addCharacteristic(rxCharacteristic);
    BLE.addService(nusService);

    rxCharacteristic.setEventHandler(BLEWritten, onRxReceived);
    BLE.setEventHandler(BLEConnected, onBleConnected);
    BLE.setEventHandler(BLEDisconnected, onBleDisconnected);

    BLE.advertise();
    setLed(false, false, true);

    Serial.println("[OK] Walker_BLE Bridge ready — waiting for BLE connection");
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

    // 5초마다 상태 출력 (디버그)
    if (millis() - lastStatusPrintMs > 5000) {
        lastStatusPrintMs = millis();
        Serial.print("[Walker_BLE STATUS] BLE:");
        Serial.print(bleConnected ? "CONN" : "DISC");
        Serial.print(" buf:");
        Serial.print(uartBufferLen);
        Serial.print("B TX:");
        Serial.print(bleTxCount);
        Serial.print(" FAIL:");
        Serial.print(bleWriteFailCount);
        Serial.print(" DROP:");
        Serial.print(bleDropCount);
        Serial.print(" OVF:");
        Serial.println(uartOverflowCount);
    }
}

// ================================================================
// [6] UART 수신
// ================================================================

void dropBytes(uint16_t count) {
    if (count >= uartBufferLen) {
        uartBufferLen = 0;
        return;
    }
    uint16_t remaining = uartBufferLen - count;
    memmove(uartBuffer, uartBuffer + count, remaining);
    uartBufferLen = remaining;
}

void dropOldestPacket() {
    for (uint16_t i = 0; i < uartBufferLen; i++) {
        if (uartBuffer[i] == '\n') {
            dropBytes(i + 1);
            bleDropCount++;
            return;
        }
    }
    uartBufferLen = 0;
    uartOverflowCount++;
}

void readUart() {
    while (TEENSY_SERIAL.available()) {
        uint8_t c = TEENSY_SERIAL.read();
        if (uartBufferLen >= UART_BUFFER_SIZE - 1) {
            dropOldestPacket();
        }
        if (uartBufferLen < UART_BUFFER_SIZE - 1) {
            uartBuffer[uartBufferLen++] = c;
        } else {
            uartBufferLen = 0;
            uartOverflowCount++;
        }
    }
}

// ================================================================
// [7] BLE 전송 (40ms 주기, \n 경계 기반)
// ================================================================

void sendToBle() {
    uint32_t now = millis();

    // 40ms 타이머: Teensy 전송 주기(40ms)에 맞춤
    if (now - lastSendMs < 40) return;
    lastSendMs = now;

    if (uartBufferLen == 0) return;

    // 연결 끊겼으면 버퍼 비움
    if (!bleConnected) {
        uartBufferLen = 0;
        return;
    }

    // subscribed() 임시 false (연결 직후 CCCD 협상 중) → 버퍼 보존, 다음 틱에 재시도
    if (!txCharacteristic.subscribed()) {
        return;
    }

    // 244B 한계 이내에서 마지막 \n까지만 전송 (파서 partial packet 방지)
    // oldest 데이터부터 전송 → 느리지만 연속적, BLE 렉 구간에도 끊기지 않음
    uint16_t searchLimit = (uartBufferLen < 244) ? uartBufferLen : 244;
    int lastNewline = -1;
    for (int i = (int)searchLimit - 1; i >= 0; i--) {
        if (uartBuffer[i] == '\n') {
            lastNewline = i;
            break;
        }
    }

    if (lastNewline < 0) {
        if (uartBufferLen >= 244) {
            uartBufferLen = 0;
            uartOverflowCount++;
        }
        return;
    }

    uint16_t sendLen = (uint16_t)(lastNewline + 1);

    // writeValue 직전 BLE 스택 드레인 — TX 큐 여유 확보
    BLE.poll();

    int result = txCharacteristic.writeValue(uartBuffer, sendLen);

    if (result) {
        dropBytes(sendLen);
        bleTxCount++;
    } else {
        // 실패 시: 버퍼 유지 → 다음 40ms에 재시도 (절대 폐기 안함)
        bleWriteFailCount++;
        BLE.poll();
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
    // Teensy에 BLE 연결 알림 → bleStreamEnabled=true (데이터 스트림 ON)
    // 모터 상태(motorEnabled)와 독립적으로 스트림을 제어
    TEENSY_SERIAL.write("blecon\n", 7);
    Serial.print("[Walker_BLE] Connected: ");
    Serial.print(central.address());
    Serial.println(" → blecon sent to Teensy");
    setLed(false, true, false);
}

void onBleDisconnected(BLEDevice central) {
    bleConnected = false;
    uartBufferLen = 0;
    // Teensy에 BLE 해제 알림 → bleStreamEnabled=false (UART 절전)
    TEENSY_SERIAL.write("bledis\n", 7);
    Serial.println("[Walker_BLE] Disconnected → bledis sent, advertising restart");
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
