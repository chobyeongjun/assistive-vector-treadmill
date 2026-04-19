# Teensy 4.1 — Hardware Specification
## MCU 사양
| 항목 | 값 |
|------|----|
| MCU | NXP iMXRT1062 |
| 클럭 | 600MHz |
| RAM | 1MB SRAM |
| Flash | 8MB |
| 개발환경 | PlatformIO (framework: arduino) |
## 제어 루프
- **inner loop**: 111Hz — 임피던스 제어, 케이블 장력 계산
- **outer loop 수신**: 10~30Hz — Jetson에서 목표 힘 벡터
- **BLE 처리**: 이벤트 기반 (non-blocking)
## PlatformIO 설정
```ini
[env:teensy41]
platform = teensy
board = teensy41
framework = arduino
lib_deps =
    FlexCAN_T4
    NimBLE-Arduino
