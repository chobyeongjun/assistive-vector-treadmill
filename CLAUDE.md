# assistive-vector-treadmill

## 프로젝트
케이블 드리븐 보조력 벡터 제어 (트레드밀 기반 보행 재활)

## 핵심 제약
- AK60 max cable force: 70N, 초과 시 즉시 0
- Teensy inner loop: 111Hz
- BLE 통신: BleComm

## 규칙 (전체 규칙 → ~/research-vault/20_Meta/claude-rules.md)
- 하드웨어 수치는 공식 스펙 확인 필수, 추측 금지
- 로봇: Exosuit (외골격/exoskeleton 금지)
- 작업 완료 시 커밋 + 푸시 자동 수행

## Vault 연동
- 실험 기록: ~/research-vault/assistive-vector-treadmill/experiments/
- 미팅 노트: ~/research-vault/assistive-vector-treadmill/meetings/
- 논문 자료: ~/research-vault/assistive-vector-treadmill/papers/
- Wiki: ~/research-vault/10_Wiki/

## 세션 시작 시
1. 관련 Wiki 노트 확인 (cable-driven-mechanism, ak60-motor 등)
2. 최근 실험 확인: ~/research-vault/assistive-vector-treadmill/experiments/
