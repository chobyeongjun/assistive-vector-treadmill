# AK60 Motor — Hardware Specification

## 모터 사양 (CubeMars AK60-6)
| 항목 | 값 |
|------|----|
| 모델 | AK60-6 |
| 정격 전압 | 24V |
| 최대 전류 | 60A |
| 피크 토크 | 15 Nm |
| 연속 토크 | 9 Nm |
| 최대 RPM | 6000 |
| 통신 | CAN 2.0B (MIT 모드) |
| 인코더 | 내장 14-bit |

## 케이블 장력 제한
- **최대 장력**: 70N (초과 시 즉시 0으로 클램프)
- 실제 제어 범위: 0~65N (5N 마진)

## MIT 모드 CAN 프로토콜
CAN ID: 0x01 ~ 0x04 (모터별)
Baudrate: 1Mbps
프레임 (8 bytes):
[0-1]: position (12-bit, -4π ~ +4π rad)
[2-3]: velocity (12-bit, -30 ~ +30 rad/s)
[4]: kp (8-bit, 0500 N·m/rad)
[5]: kd (8-bit, 05 N·m·s/rad)
[6-7]: torque (12-bit, -18~+18 N·m)

## BLE 트레드밀 인터페이스
- 서비스: Nordic UART Service (NUS)
- UUID TX: `6e400003-b5a3-f393-e0a9-e50e24dcca9e`
- UUID RX: `6e400002-b5a3-f393-e0a9-e50e24dcca9e`
- 라이브러리: bleak (Python asyncio)
- 패킷: `SPEED:x.x\r\n`
