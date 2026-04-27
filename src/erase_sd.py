#!/usr/bin/env python3
"""
erase_sd.py — SD 카드 전체 파일 삭제

Usage:
  python3 src/erase_sd.py
  python3 src/erase_sd.py --port /dev/cu.usbmodem...
"""

import sys
import time
import argparse
from pathlib import Path

BAUD    = 115200
TIMEOUT = 10


def _import_serial():
    try:
        import serial
        import serial.tools.list_ports
        return serial, serial.tools.list_ports
    except ImportError:
        sys.exit("pyserial not found. Run: pip3 install pyserial")


def auto_detect_port(list_ports):
    teensy_ids = [(0x16C0, 0x0483), (0x16C0, 0x0487)]
    usbmodem = []
    others = []
    for p in list_ports.comports():
        dev = p.device or ""
        vid = getattr(p, 'vid', None)
        pid = getattr(p, 'pid', None)
        if (vid, pid) in teensy_ids:
            return dev
        if 'usbmodem' in dev.lower() or 'ttyacm' in dev.lower():
            usbmodem.append(dev)
        else:
            others.append(dev)
    if len(usbmodem) == 1:
        return usbmodem[0]
    all_ports = usbmodem + others
    if not all_ports:
        return None
    print("[!] 포트 목록:")
    for i, d in enumerate(all_ports):
        print(f"    [{i}] {d}")
    try:
        idx = int(input("번호 선택 > ").strip())
        return all_ports[idx]
    except Exception:
        return None


def open_port(port, baud):
    serial, _ = _import_serial()
    import threading
    result = [None]

    def _open():
        try:
            result[0] = serial.Serial(port, baud, timeout=0.1, exclusive=False)
        except Exception as e:
            result[0] = e

    t = threading.Thread(target=_open, daemon=True)
    t.start()
    t.join(timeout=5.0)
    if t.is_alive():
        sys.exit(f"[ERROR] 포트 열기 타임아웃 — 다른 프로그램이 {port} 를 점유 중인지 확인")
    if isinstance(result[0], Exception):
        sys.exit(f"[ERROR] {result[0]}")
    return result[0]


def readline(ser, timeout=10):
    deadline = time.time() + timeout
    buf = b""
    while time.time() < deadline:
        c = ser.read(1)
        if not c:
            continue
        if c == b"\n":
            return buf.decode(errors="replace").rstrip("\r")
        buf += c
    return None


def list_files(ser):
    ser.reset_input_buffer()
    ser.write(b"ls\n")
    ser.flush()
    files = []
    deadline = time.time() + TIMEOUT
    in_list = False
    while time.time() < deadline:
        line = readline(ser)
        if line is None:
            break
        if "__SD_ERROR__" in line:
            print("[ERROR] SD 사용 불가")
            return []
        if "__FILE_LIST_START__" in line:
            in_list = True
            continue
        if "__FILE_LIST_END__" in line:
            break
        if in_list and "," in line:
            parts = line.strip().split(",")
            if len(parts) >= 2:
                try:
                    files.append(parts[0].strip())
                except Exception:
                    pass
    return files


def delete_file(ser, filename):
    ser.reset_input_buffer()
    ser.write(f"del {filename}\n".encode())
    ser.flush()
    deadline = time.time() + TIMEOUT
    while time.time() < deadline:
        line = readline(ser)
        if line is None:
            break
        if "__DELETED__" in line:
            return True
        if "__ERROR__" in line:
            print(f"  [FAIL] {filename}: {line}")
            return False
    print(f"  [TIMEOUT] {filename}")
    return False


def main():
    parser = argparse.ArgumentParser(description="SD 카드 전체 파일 삭제")
    parser.add_argument("--port")
    parser.add_argument("--baud", type=int, default=BAUD)
    args = parser.parse_args()

    _, list_ports = _import_serial()
    port = args.port or auto_detect_port(list_ports)
    if not port:
        sys.exit("포트를 찾을 수 없습니다. --port 로 지정하세요.")

    print(f"[+] {port} @ {args.baud} — 연결 중...")
    ser = open_port(port, args.baud)
    time.sleep(0.5)
    ser.reset_input_buffer()
    print("[+] 연결 완료\n")

    print("파일 목록 조회 중...")
    files = list_files(ser)
    if not files:
        print("SD 카드가 비어 있거나 접근 불가.")
        ser.close()
        return

    print(f"\n총 {len(files)}개 파일:")
    for f in files:
        print(f"  {f}")

    print(f"\n⚠️  위 {len(files)}개 파일을 모두 삭제합니다.")
    print("로깅이 중지된 상태인지 확인하세요 (logstop 먼저).")
    ans = input("\n계속하려면 'DELETE' 입력 > ").strip()
    if ans != "DELETE":
        print("취소됨.")
        ser.close()
        return

    print()
    ok = 0
    fail = 0
    for f in files:
        if delete_file(ser, f):
            print(f"  ✓ {f}")
            ok += 1
        else:
            fail += 1

    print(f"\n[완료] 삭제 {ok}개 / 실패 {fail}개")
    ser.close()


if __name__ == "__main__":
    main()
