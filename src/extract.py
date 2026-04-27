#!/usr/bin/env python3
"""
extract.py — SD card data extractor for Teensy firmware (Treadmill_main / Loadcell_Monitor)

SD 카드를 뽑지 않고 USB Serial로 파일 목록 조회, 다운로드, 삭제 가능.

Protocol (SDTransfer 라이브러리):
  ls             → __FILE_LIST_START__ / name,size / __FILE_LIST_END__
  get <file>     → __FILE_START__,name,size / data / __FILE_END__
  del <file>     → __DELETED__: name  or  __ERROR__: msg
  stop_log       → __LOG_STOPPED__  (Treadmill_main)
  logstop        → __LOG_STOPPED__  (Loadcell_Monitor)

Usage:
  python3 src/extract.py ls
  python3 src/extract.py get DATA_00.CSV
  python3 src/extract.py all
  python3 src/extract.py all --delete
  python3 src/extract.py del DATA_00.CSV
Options:
  --port     <port>  Serial port (default: auto-detect Teensy)
  --baud     <rate>  Baud rate (default: 115200)
  --out      <dir>   Local save directory (default: ./data)
  --delete           Delete file from SD after successful download
  --timeout  <s>     Response timeout in seconds (default: 10)
"""

import sys
import argparse
import time
from pathlib import Path


def pick_folder(title="저장 폴더 선택") -> Path:
    import platform
    if platform.system() == "Darwin":
        import subprocess
        r = subprocess.run(
            ["osascript", "-e",
             f'POSIX path of (choose folder with prompt "{title}")'],
            capture_output=True, text=True
        )
        if r.returncode != 0 or not r.stdout.strip():
            sys.exit("폴더 선택 취소됨")
        return Path(r.stdout.strip())
    # non-Mac fallback
    try:
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk()
        root.withdraw()
        folder = filedialog.askdirectory(title=title)
        root.destroy()
        if not folder:
            sys.exit("폴더 선택 취소됨")
        return Path(folder)
    except Exception:
        folder = input("저장 폴더 경로 입력: ").strip()
        return Path(folder) if folder else Path("data")

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
            return dev                          # VID/PID 확실히 일치
        if 'usbmodem' in dev.lower() or 'ttyacm' in dev.lower():
            usbmodem.append(dev)
        else:
            others.append(dev)

    if len(usbmodem) == 1:
        return usbmodem[0]                      # usbmodem 하나만 있으면 바로 사용

    # 자동 판단 불가 → 전체 목록 보여주고 선택
    all_ports = usbmodem + others
    if not all_ports:
        return None
    print("[!] 포트를 자동으로 특정하지 못했습니다. 연결된 포트 목록:")
    for i, d in enumerate(all_ports):
        print(f"    [{i}] {d}")
    try:
        idx = int(input("번호 선택 > ").strip())
        return all_ports[idx]
    except Exception:
        return None


# ──────────────────────────────────────────────────────────────────

class TeensyLink:
    def __init__(self, port: str, baud: int = BAUD, timeout: float = TIMEOUT):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser = None
        self._open_port()

    def _open_port(self):
        serial, _ = _import_serial()
        import threading
        result = [None]

        def _open():
            try:
                result[0] = serial.Serial(self.port, self.baud, timeout=0.1,
                                          exclusive=False)
            except Exception as e:
                result[0] = e

        t = threading.Thread(target=_open, daemon=True)
        t.start()
        t.join(timeout=5.0)

        if t.is_alive():
            sys.exit(f"[ERROR] 포트 열기 실패 (5초 타임아웃). "
                     f"Arduino IDE나 GUI가 {self.port} 를 점유 중인지 확인하세요.")
        if isinstance(result[0], Exception):
            sys.exit(f"[ERROR] {result[0]}")

        self.ser = result[0]
        time.sleep(0.5)
        self._flush()

    def _reconnect(self, max_wait: float = 8.0) -> bool:
        serial, _ = _import_serial()
        print(f"[!] 재연결 대기 중...", end=" ", flush=True)
        try:
            self.ser.close()
        except Exception:
            pass
        deadline = time.time() + max_wait
        while time.time() < deadline:
            time.sleep(1.0)
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=0.1,
                                         exclusive=False)
                time.sleep(0.5)
                self._flush()
                print("재연결 완료")
                return True
            except Exception:
                pass
        print("재연결 실패")
        return False

    def _flush(self):
        self.ser.reset_input_buffer()

    def _send(self, cmd: str):
        self.ser.write((cmd.strip() + "\n").encode())
        self.ser.flush()

    def _readline_timeout(self):
        import serial as _serial
        deadline = time.time() + self.timeout
        buf = b""
        while time.time() < deadline:
            try:
                c = self.ser.read(1)
            except _serial.SerialException:
                print(f"\n[DISCONNECT]", end=" ")
                self._reconnect()
                return None
            if not c:
                continue
            if c == b"\n":
                return buf.decode(errors="replace").rstrip("\r")
            buf += c
        return None

    def _readbytes_exact(self, n: int) -> bytes:
        deadline = time.time() + self.timeout + n / BAUD
        data = b""
        while len(data) < n and time.time() < deadline:
            data += self.ser.read(n - len(data))
        return data

    def stop_logging(self):
        self._flush()
        self._send("stop_log")   # Treadmill_main
        self._send("logstop")    # Loadcell_Monitor
        print("  로깅 중지 중...", end=" ", flush=True)
        # __LOG_STOPPED__ 수신 대기 (최대 15s — 대용량 파일 flush 고려)
        deadline = time.time() + 15.0
        while time.time() < deadline:
            line = self._readline_timeout()
            if line is None:
                continue
            if "__LOG_STOPPED__" in line:
                break
        else:
            print("(타임아웃) ", end="")
        time.sleep(0.3)
        self._flush()
        print("완료")

    def start_logging(self):
        self._flush()
        self._send("s")          # Treadmill_main
        self._send("log")        # Loadcell_Monitor
        time.sleep(1.0)
        self._flush()
        print("  로깅 시작됨")

    def list_files(self) -> list:
        self._flush()
        self._send("ls")
        files = []
        in_list = False
        deadline = time.time() + self.timeout
        while time.time() < deadline:
            line = self._readline_timeout()
            if line is None:
                break
            if "__SD_ERROR__" in line:
                print("[ERROR] SD not available")
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
                        files.append({"name": parts[0].strip(),
                                      "size": int(parts[1].strip())})
                    except ValueError:
                        pass
        return files

    def get_file(self, filename: str, out_dir: Path, retries: int = 2):
        for attempt in range(retries + 1):
            if attempt > 0:
                print(f"  재시도 {attempt}/{retries}...")
                time.sleep(1.0)

            self._flush()
            self._send(f"get {filename}")

            header = None
            deadline = time.time() + self.timeout
            while time.time() < deadline:
                line = self._readline_timeout()
                if line is None:
                    break
                if line.startswith("__ERROR__"):
                    print(f"[ERROR] {line}")
                    break
                if "__FILE_START__" in line:
                    header = line
                    break

            if header:
                break
        else:
            print(f"[ERROR] No response for {filename}")
            return None

        parts = header.split(",")
        if len(parts) < 3:
            print(f"[ERROR] Bad header: {header}")
            return None
        try:
            file_size = int(parts[-1].strip())
        except ValueError:
            print(f"[ERROR] Bad size in header: {header}")
            return None

        print(f"  {filename} ({file_size:,} bytes)...", end=" ", flush=True)
        data = self._readbytes_exact(file_size)

        end_deadline = time.time() + 3
        while time.time() < end_deadline:
            line = self._readline_timeout()
            if line is None or "__FILE_END__" in line:
                break

        if len(data) != file_size:
            print(f"[WARN] Got {len(data)}/{file_size} bytes")

        out_dir.mkdir(parents=True, exist_ok=True)
        out_path = out_dir / filename
        out_path.write_bytes(data)
        print(f"→ {out_path}")
        return out_path

    def delete_file(self, filename: str) -> bool:
        self._flush()
        self._send(f"del {filename}")
        deadline = time.time() + self.timeout
        while time.time() < deadline:
            line = self._readline_timeout()
            if line is None:
                break
            if "__DELETED__" in line:
                print(f"  Deleted {filename}")
                return True
            if "__ERROR__" in line:
                print(f"  Delete failed: {line}")
                return False
        return False

    def close(self):
        self.ser.close()


# ──────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Extract SD card files from Teensy over USB serial")
    parser.add_argument("action", choices=["ls", "get", "all", "del", "log"],
                        nargs="?", default="all")
    parser.add_argument("filename", nargs="?", help="For get/del")
    parser.add_argument("--port")
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--out", default=None,
                        help="저장 폴더 (생략 시 폴더 선택창)")
    parser.add_argument("--delete", action="store_true",
                        help="Delete from SD after download")
    parser.add_argument("--timeout", type=float, default=TIMEOUT)
    args = parser.parse_args()

    _, list_ports = _import_serial()
    port = args.port or auto_detect_port(list_ports)
    if not port:
        sys.exit("No serial port found. Specify --port /dev/ttyXXX")

    print(f"[+] {port} @ {args.baud} — 연결 중...")
    link = TeensyLink(port, args.baud, args.timeout)
    print("[+] 연결 완료")
    out_dir = Path(args.out) if args.out else pick_folder()
    print(f"[+] 저장 위치: {out_dir.resolve()}")

    try:
        if args.action == "ls":
            files = link.list_files()
            if not files:
                print("(empty or SD error)")
            else:
                print(f"{'Name':<20} {'Size':>10}")
                print("-" * 32)
                for f in files:
                    print(f"{f['name']:<20} {f['size']:>10,}")
                print(f"\n{len(files)} file(s)")

        elif args.action == "get":
            if not args.filename:
                sys.exit("Specify filename")
            link.stop_logging()
            path = link.get_file(args.filename, out_dir)
            if args.delete and path:
                link.delete_file(args.filename)

        elif args.action == "del":
            if not args.filename:
                sys.exit("Specify filename")
            link.delete_file(args.filename)

        elif args.action == "log":
            ans = input("로깅 [1] 중지  [2] 시작  > ").strip()
            if ans == "1":
                link.stop_logging()
            elif ans == "2":
                link.start_logging()

        elif args.action == "all":
            # 파일 목록
            files = link.list_files()
            csv_files = [f for f in files
                         if f["name"].upper().endswith(".CSV")
                         and not f["name"].startswith("._")]
            if not csv_files:
                print("No CSV files on SD.")
            else:
                print(f"\n{'#':<4} {'Name':<20} {'Size':>10}")
                print("-" * 36)
                for i, f in enumerate(csv_files):
                    print(f"[{i}] {f['name']:<20} {f['size']:>10,}")
                print("\n전체 다운로드: Enter  |  선택: 번호 입력 (예: 0 2 3)")
                sel = input("> ").strip()
                if sel == "":
                    selected = csv_files
                else:
                    try:
                        idxs = [int(x) for x in sel.split()]
                        selected = [csv_files[i] for i in idxs if 0 <= i < len(csv_files)]
                    except Exception:
                        selected = csv_files

                # 3) 다운로드
                print(f"[+] {len(selected)}개 다운로드")
                for f in selected:
                    path = link.get_file(f["name"], out_dir)
                    if args.delete and path:
                        link.delete_file(f["name"])
                print(f"\n[OK] 저장 완료 → {out_dir.resolve()}")

    finally:
        link.close()


if __name__ == "__main__":
    main()
