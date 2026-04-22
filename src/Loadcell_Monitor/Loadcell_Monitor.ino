/*
 * ================================================================
 *  Loadcell Monitor Firmware (Teensy 4.1)
 * ================================================================
 *
 *  목적: L/R 로드셀 Compression 힘을 실시간 측정하여 BLE로 전송
 *
 *  하드웨어:
 *   - Teensy 4.1 (메인 컨트롤러)
 *   - Arduino Nano 33 BLE (BLE 브릿지)
 *   - 아날로그 로드셀 x2 (Left: A16, Right: A6)
 *
 *  변환 공식:
 *   voltage = ADC_raw * (3.3V / 4096)
 *   Force_N = (voltage * sensitive) + bias
 *   ★ Compression이 양수가 되도록 부호 조정
 *
 *  BLE 패킷: "SL2c<L_force*100>n<R_force*100>n"
 *  전송 주기: 50Hz (20ms)
 *
 * ================================================================
 */

#if defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)

#include <Arduino.h>
#include <IntervalTimer.h>
#include <SD.h>
#include <SPI.h>
#include <math.h>

#include "BleComm.h"

// ================================================================
// [1] 하드웨어 핀 설정
// ================================================================

#define LEFT_LOADCELL_PIN   A16
#define RIGHT_LOADCELL_PIN  A6

// Analog trigger (A7 = pin21 on Teensy 4.1) — 펌웨어 Treadmill_main.ino와 동일
// 외부 트리거 신호로 firmware와 동시에 로깅을 시작하고, 매 행에 a7 값을 기록해
// 사후(post-hoc) 시간 동기화에 사용한다.
const int ANALOG_PIN = 21;
const int TRIGGER_THRESHOLD = 2000;

// ================================================================
// [2] ADC 변환 상수
// ================================================================

// Teensy 4.1: 12-bit ADC, 3.3V 기준
const float AI_CNT_TO_V = 3.3f / 4096.0f;

// ================================================================
// [3] 로드셀 캘리브레이션 값
// ================================================================
// ★ 실제 캘리브레이션 후 여기 값을 업데이트할 것!
// ★ Compression이 양수가 되도록 sensitive 부호를 조정
//    - 기존: tension 양수 (당길 때 +)
//    - 변경: compression 양수 (누를 때 +) → sensitive 부호 반전 or bias 조정

// --- Left Loadcell ---
const float LEFT_BIAS       = -308.2f;   // [N] offset
const float LEFT_SENSITIVE  = 250.5f;    // [N/V] sensitivity

// --- Right Loadcell ---
const float RIGHT_BIAS      = -670.0f;   // [N] offset
const float RIGHT_SENSITIVE = 549.0f;    // [N/V] sensitivity

// ★ Compression 양수 변환 플래그
// true: Force = -((voltage * sensitive) + bias) → compression이 양수
// false: Force = (voltage * sensitive) + bias → tension이 양수 (기존 방식)
const bool INVERT_FOR_COMPRESSION = true;

// ================================================================
// [4] LowPass Filter (50Hz cutoff)
// ================================================================

class LowPassFilter {
    float alpha;
    float y_prev;
public:
    LowPassFilter(float cutoff_hz, float sample_fs) {
        float dt = 1.0f / sample_fs;
        float rc = 1.0f / (2.0f * 3.14159f * cutoff_hz);
        alpha = dt / (rc + dt);
        y_prev = 0;
    }

    float update(float x) {
        y_prev = y_prev + alpha * (x - y_prev);
        return y_prev;
    }

    void reset() { y_prev = 0; }
};

// ================================================================
// [5] 타이밍 설정
// ================================================================

// ADC 읽기: 3ms (333Hz) - 고속 샘플링으로 LPF 정확도 확보
const uint32_t ADC_PERIOD_US = 3000;       // ISR 주기: 3ms (333Hz)
const float    ADC_FS = 333.3f;            // 샘플링 주파수

// BLE 전송: 20ms (50Hz) - BLE NUS 안정 대역폭 내
const uint8_t  BLE_DIVIDER = 7;            // 333Hz / 7 ≈ 47Hz
const uint32_t SERIAL_PRINT_MS = 100;      // 시리얼 출력: 100ms (10Hz)

// SD 로깅: ISR 매 틱(3ms, 333Hz)마다 기록
const int SDCARD_CS_PIN = BUILTIN_SDCARD;
const uint8_t  LOG_DIVIDER = 3;            // 333Hz / 3 ≈ 111Hz 로깅

// ================================================================
// [6] 전역 변수
// ================================================================

// 로드셀 필터 (cutoff 20Hz, 333Hz 샘플링 → 깨끗한 필터링)
LowPassFilter loadcellFilter_L(20.0f, ADC_FS);
LowPassFilter loadcellFilter_R(20.0f, ADC_FS);

// 측정값 (ISR에서 갱신 → volatile)
volatile float forceLeft_N  = 0.0f;
volatile float forceRight_N = 0.0f;

// Tare 오프셋 (영점 조정용)
volatile float tareOffset_L = 0.0f;
volatile float tareOffset_R = 0.0f;

// ISR 타이머
IntervalTimer adcTimer;
volatile uint32_t isrTickCount = 0;

// ── SD 로깅 (링 버퍼) ──
struct LogEntry {
    uint32_t timestamp_ms;
    float l_force;
    float r_force;
    uint16_t a7;  // 펌웨어 sync용 analog trigger 값 (A7)
};

const uint32_t LOG_BUF_SIZE = 512;
// ★ firmware Treadmill_main.ino와 동일 패턴:
//   - 버퍼 자체는 volatile이 아니다 (DMAMEM → DMA-coherent RAM 배치).
//   - volatile 배열은 멤버별 쓰기가 ARM GCC 최적화에서 사라지는 버그의 원인.
DMAMEM LogEntry logBuffer[LOG_BUF_SIZE];
volatile uint32_t logHead = 0;   // ISR이 쓰는 위치
volatile uint32_t logTail = 0;   // loop()이 읽는 위치

volatile bool isLogging = false;
File dataFile;
char filename[32] = "LC_00.CSV";

// 저장 행 카운터 (시리얼 하트비트용) — 전원 끊겨도 몇 행까진 SD에 있는지 확인용
volatile uint32_t totalRowsLogged = 0;

// ★ 디버그 카운터 — 파이프라인 어디서 막히는지 추적
volatile uint32_t isrLogGateHits    = 0;  // if (isLogging && tick%3==0) 진입 횟수
volatile uint32_t isrLogWrites      = 0;  // 실제 logBuffer[] 기록 성공 횟수
volatile uint32_t isrLogBufFull     = 0;  // 링버퍼 가득참 → 드롭

// A7 analog trigger (펌웨어와 동일 패턴: loop()에서 읽어 syncA7 갱신 → ISR이 로그 엔트리에 기록)
volatile uint16_t syncA7 = 0;

// ================================================================
// [7] 로드셀 읽기 함수
// ================================================================

float readLoadcellForceN(int pin, float bias, float sensitive, LowPassFilter& lpf) {
    int raw_adc = analogRead(pin);
    float voltage = raw_adc * AI_CNT_TO_V;
    float F = (voltage * sensitive) + bias;

    // Compression 양수 변환
    if (INVERT_FOR_COMPRESSION) {
        F = F;
    }

    // LPF 적용
    F = lpf.update(F);

    // 음수 클램프 (compression 기준: 당기는 힘은 0으로)
    if (F < 0) F = 0;

    return F;
}

// ================================================================
// [8] ADC ISR (333Hz) - 고속 읽기 + LPF + 50Hz BLE 전송
// ================================================================

void adcISR() {
    // 333Hz로 ADC 읽기 + LPF 적용 - tare 오프셋 차감
    forceLeft_N  = readLoadcellForceN(LEFT_LOADCELL_PIN,  LEFT_BIAS,  LEFT_SENSITIVE,  loadcellFilter_L) - tareOffset_L;
    forceRight_N = readLoadcellForceN(RIGHT_LOADCELL_PIN, RIGHT_BIAS, RIGHT_SENSITIVE, loadcellFilter_R) - tareOffset_R;
    if (forceLeft_N < 0)  forceLeft_N = 0;
    if (forceRight_N < 0) forceRight_N = 0;

    isrTickCount++;

    // 매 7틱마다 BLE 전송 (333/7 ≈ 47Hz)
    if (isrTickCount % BLE_DIVIDER == 0) {
        sendLoadcellToBLE(forceLeft_N, forceRight_N);
    }

    // 매 3틱마다 SD 로그 버퍼에 기록 (333/3 ≈ 111Hz)
    // ★ firmware ISR_Control()와 동일 패턴: 로컬 구조체에 채운 뒤 통째 대입
    if (isLogging && (isrTickCount % LOG_DIVIDER == 0)) {
        isrLogGateHits++;

        LogEntry e;
        e.timestamp_ms = millis();
        e.l_force      = forceLeft_N;
        e.r_force      = forceRight_N;
        e.a7           = syncA7;

        uint32_t head     = logHead;
        uint32_t nextHead = (head + 1) % LOG_BUF_SIZE;
        if (nextHead != logTail) {  // 오버플로 방지
            logBuffer[head] = e;   // 구조체 통째 복사
            logHead = nextHead;
            isrLogWrites++;
        } else {
            isrLogBufFull++;
        }
    }
}

// ================================================================
// [9] SD 로깅 함수
// ================================================================

void startLogging() {
    if (isLogging) return;

    // 자동 파일명: LC_00.CSV ~ LC_99.CSV
    for (int i = 0; i < 100; i++) {
        snprintf(filename, sizeof(filename), "LC_%02d.CSV", i);
        if (!SD.exists(filename)) break;
    }

    dataFile = SD.open(filename, FILE_WRITE);
    if (dataFile) {
        // ★ 헤더 먼저 쓰고 즉시 flush → 빈 파일이어도 컬럼은 남는다
        dataFile.println("timestamp_ms,L_force_N,R_force_N,a7");
        dataFile.flush();

        noInterrupts();
        logHead = 0;
        logTail = 0;
        interrupts();
        isLogging = true;

        Serial.print("[SD] Logging started: ");
        Serial.println(filename);

        char resp[48];
        snprintf(resp, sizeof(resp), "LOG_START:%s", filename);
        sendBleResponse(resp);
    } else {
        Serial.println("[SD] Failed to open file!");
        sendBleResponse("LOG_FAIL:SD_ERROR");
    }
}

void stopLogging() {
    isLogging = false;

    // 남은 버퍼 모두 기록
    processLogBuffer();

    if (dataFile) {
        dataFile.flush();
        dataFile.close();
    }

    Serial.print("[SD] Logging stopped: ");
    Serial.println(filename);

    char resp[48];
    snprintf(resp, sizeof(resp), "LOG_STOP:%s", filename);
    sendBleResponse(resp);
}

void processLogBuffer() {
    static uint16_t flushCount = 0;
    static uint32_t lastFlushMs = 0;

    while (logTail != logHead) {
        LogEntry e = logBuffer[logTail];  // 구조체 통째 복사 (firmware 패턴)
        logTail = (logTail + 1) % LOG_BUF_SIZE;

        if (dataFile) {
            dataFile.print(e.timestamp_ms);
            dataFile.print(",");
            dataFile.print(e.l_force, 3);
            dataFile.print(",");
            dataFile.print(e.r_force, 3);
            dataFile.print(",");
            dataFile.println(e.a7);

            flushCount++;
            totalRowsLogged++;
        }
    }

    // 10행 또는 100ms마다 flush → 전원 차단 시에도 손실 행 ≤ 100ms 분량
    uint32_t now = millis();
    if (dataFile && (flushCount >= 10 || (flushCount > 0 && now - lastFlushMs >= 100))) {
        dataFile.flush();
        flushCount = 0;
        lastFlushMs = now;
    }
}

// ================================================================
// [10] BLE 명령 핸들러
// ================================================================

void handleBleCommand(String cmd) {
    cmd.trim();

    if (cmd == "start") {
        bleStreamEnabled = true;
        sendBleResponse("STREAM_ON");
        Serial.println("[BLE] Stream ON");
    }
    else if (cmd == "stop") {
        bleStreamEnabled = false;
        sendBleResponse("STREAM_OFF");
        Serial.println("[BLE] Stream OFF");
    }
    else if (cmd == "log") {
        startLogging();
    }
    else if (cmd == "logstop") {
        stopLogging();
    }
    else if (cmd == "tare") {
        // 현재 측정값 + 기존 오프셋 = 새 오프셋 (누적)
        tareOffset_L += forceLeft_N;
        tareOffset_R += forceRight_N;

        char resp[64];
        snprintf(resp, sizeof(resp), "TARE_OK:L=%.1f,R=%.1f", tareOffset_L, tareOffset_R);
        sendBleResponse(resp);
        Serial.print("[BLE] Tare set: L_offset=");
        Serial.print(tareOffset_L, 1);
        Serial.print(" R_offset=");
        Serial.println(tareOffset_R, 1);
    }
    else {
        Serial.print("[BLE] Unknown cmd: ");
        Serial.println(cmd);
    }
}

// ================================================================
// [10] Setup
// ================================================================

void setup() {
    // Serial Monitor
    Serial.begin(115200);
    delay(500);

    Serial.println("  Loadcell Monitor  (auto-log on boot)");
    Serial.println("====================================================");

    // ADC 설정: 12-bit
    analogReadResolution(12);

    // [1/4] 핀 초기화
    pinMode(LEFT_LOADCELL_PIN, INPUT);
    pinMode(RIGHT_LOADCELL_PIN, INPUT);
    pinMode(ANALOG_PIN, INPUT);  // A7 sync trigger (펌웨어와 동일)
    Serial.println("[1/4] Pins ready (L=A16, R=A6, sync=A7)");

    // [2/4] SD 카드 초기화
    bool sdOK = SD.begin(SDCARD_CS_PIN);
    if (sdOK) {
        Serial.println("[2/4] SD OK");
    } else {
        Serial.println("[2/4] SD FAIL — data will NOT be saved!");
    }

    // [3/4] BLE + ADC ISR
    setupBleComm();
    Serial.println("[3/4] BLE Serial ready");

    adcTimer.begin(adcISR, ADC_PERIOD_US);
    Serial.println("[3/4] ADC ISR @333Hz / BLE @47Hz / LOG @111Hz");

    // [4/4] AUTO-LOG: SD가 살아있으면 부팅 즉시 로깅 시작
    //   → A7 트리거나 BLE 'log' 명령을 기다리지 않는다.
    //   → 시간/L/R/A7 컬럼이 즉시 파일에 남고, 데이터는 333/3 ≈ 111Hz로 계속 append.
    if (sdOK) {
        Serial.println("[4/4] AUTO-LOG: calling startLogging()...");
        startLogging();
        Serial.print(">>> BOOT OK — logging: ");
        Serial.println(filename);
    } else {
        Serial.println("[4/4] AUTO-LOG skipped (SD not ready)");
    }
    Serial.println("====================================================");
}

// ================================================================
// [11] Main Loop
// ================================================================

void loop() {
    // BLE 명령 수신 처리
    processBleSerial();

    // ── A7 analog sync trigger (펌웨어 Treadmill_main.ino와 동일 패턴) ──
    // 외부 트리거 신호로 firmware와 동시 로깅 시작 + 매 행 a7 값 기록(사후 동기화용)
    int a7 = analogRead(ANALOG_PIN);
    syncA7 = (uint16_t)a7;

    if (!isLogging && a7 > TRIGGER_THRESHOLD) {
        Serial.print("[A7] Trigger detected (");
        Serial.print(a7);
        Serial.println(") → auto startLogging");
        startLogging();
    }

    // SD 로그 버퍼 → 파일 쓰기
    if (isLogging || dataFile) {
        processLogBuffer();
    }

    // 시리얼 모니터 출력 (10Hz, 디버그용)
    static uint32_t lastPrintMs = 0;
    uint32_t now = millis();

    if (now - lastPrintMs >= SERIAL_PRINT_MS) {
        lastPrintMs = now;

        Serial.print("L: ");
        Serial.print(forceLeft_N, 1);
        Serial.print(" N  |  R: ");
        Serial.print(forceRight_N, 1);
        Serial.print(" N  | isr_tick=");
        Serial.print(isrTickCount);
        Serial.print(" gate=");
        Serial.print(isrLogGateHits);
        Serial.print(" write=");
        Serial.print(isrLogWrites);
        Serial.print(" full=");
        Serial.print(isrLogBufFull);
        Serial.print(" head=");
        Serial.print(logHead);
        Serial.print(" tail=");
        Serial.print(logTail);
        Serial.print(" isLog=");
        Serial.print(isLogging ? 1 : 0);
        Serial.print(" rows=");
        Serial.print(totalRowsLogged);
        Serial.print("  |  BLE: ");
        Serial.println(bleStreamEnabled ? "ON" : "OFF");
    }
}

#else
#error "이 코드는 Teensy 4.1 전용입니다."
#endif
