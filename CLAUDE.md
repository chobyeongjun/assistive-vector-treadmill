
assistive-vector-treadmill
프로젝트 개요
AK60 모터 케이블 드리븐 워커 × 트레드밀 보행 재활. BLE 트레드밀 제어 + GUI.

구조
src/firmware/ — Teensy 4.1 PlatformIO 펌웨어
src/firmware/src/Treadmill_main.ino — 메인 루프
src/firmware/src/BleComm.cpp/.h — Nordic UART BLE 통신
src/gui/ — PyQt5 트레드밀 제어 GUI
docs/hardware/ — AK60, Teensy 하드웨어 스펙
빌드
pip install -r requirements.txt
python src/gui/main.py
cd src/firmware && pio run -t upload
핵심 제약
AK60 최대 케이블 장력: 70N (초과 시 즉시 0)
Teensy inner loop: 111Hz
Jetson outer loop: 10~30Hz
