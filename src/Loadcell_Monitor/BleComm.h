/*
 * ================================================================
 *  BLE Communication Module for Loadcell Monitor
 *  Header File
 * ================================================================
 *
 *  Teensy 4.1 <-> Arduino Nano 33 BLE 간 UART 브릿지 통신
 *  Nordic UART Service (NUS) 프로토콜 사용
 *
 *  패킷 포맷: "SL2c<L_force>n<R_force>n"
 *  - S: 시작 문자
 *  - L: Loadcell 명령
 *  - 2: 데이터 개수 (L, R)
 *  - c: 데이터 시작
 *  - n: 구분자
 *  - 값은 정수 (실제값 * 100)
 *
 * ================================================================
 */

#ifndef BLE_COMM_H
#define BLE_COMM_H

#include <Arduino.h>

// ================================================================
// [1] BLE Serial 정의 및 핀 배치
// ================================================================

/*
 * ┌─────────────────────────────────────────────────────────────┐
 * │                    HARDWARE CONNECTION                       │
 * ├─────────────────────────────────────────────────────────────┤
 * │  Teensy 4.1                    Arduino Nano 33 BLE          │
 * │  ───────────                   ───────────────────          │
 * │  Pin 35 (TX8) ──────────────►  RX (D0 / Serial1 RX)         │
 * │  Pin 34 (RX8) ◄──────────────  TX (D1 / Serial1 TX)         │
 * │  GND         ─────────────────  GND                         │
 * │                                                              │
 * │  ⚠  3.3V Logic Level (Teensy 4.1 및 Nano 33 BLE 모두 호환)  │
 * └─────────────────────────────────────────────────────────────┘
 */

#define BLE_TX_PIN 35
#define BLE_RX_PIN 34

#define BLE_SERIAL Serial8
#define BLE_BAUD_RATE 115200

// ================================================================
// [2] 데이터 전송 설정
// ================================================================

#define LOADCELL_DATA_COUNT 2    // L, R 두 채널
#define BLE_SEND_PERIOD_MS 20   // 전송 주기 (20ms = 50Hz)

// ================================================================
// [3] 전역 변수 선언 (extern)
// ================================================================

extern volatile bool bleStreamEnabled;

extern char bleRxBuffer[128];
extern uint8_t bleRxLen;

// ================================================================
// [4] 함수 선언
// ================================================================

void setupBleComm();

/**
 * @brief L/R 로드셀 힘 데이터를 BLE로 전송
 * 패킷 포맷: "SL2c<L_force>n<R_force>n"
 */
void sendLoadcellToBLE(float l_force, float r_force);

/**
 * @brief 응답 전송 (펌웨어 → GUI)
 * 패킷 포맷: "SR:<message>\n"
 */
void sendBleResponse(const char* msg);

/**
 * @brief BLE Serial 수신 처리
 */
bool processBleSerial();

/**
 * @brief 외부 구현 필요 - BLE 명령 핸들러
 */
extern void handleBleCommand(String cmd);

#endif // BLE_COMM_H
