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
  python3 src/extract.py all --firmware loadcell

Options:
  --port      <port>  Serial port (default: auto-detect Teensy)
  --baud      <rate>  Baud rate (default: 115200)
  --out       <dir>   Local save directory (default: ./data)
  --firmware  <name>  main | loadcell (default: main)
  --delete            Delete file from SD after successful download
  --timeout   <s>     Response timeout in seconds (default: 10)
"""

import sys
import argparse
import time
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
    candidates = []
    for p in list_ports.comports():
        vid = getattr(p, 'vid', None)
        pid = getattr(p, 'pid', None)
        if (vid, pid) in teensy_ids:
            return p.device
        if p.device and ('usbmodem' in p.device.lower() or 'ttyacm' in p.device.lower()):
            candidates.append(p.device)
    if candidates:
        return candidates[0]
    all_ports = list(list_ports.comports())
    return all_ports[0].device if all_ports else None


# ──────────────────────────────────────────────────────────────────

class TeensyLink:
    def __init__(self, port: str, baud: int = BAUD, timeout: float = TIMEOUT,
                 firmware: str = "main"):
        serial, _ = _import_serial()
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.timeout = timeout
        self.firmware = firmware
        time.sleep(1.5)
        self._flush()

    def _flush(self):
        self.ser.reset_input_buffer()

    def _send(self, cmd: str):
        self.ser.write((cmd.strip() + "\n").encode())
        self.ser.flush()

    def _readline_timeout(self):
        deadline = time.time() + self.timeout
        buf = b""
        while time.time() < deadline:
            c = self.ser.read(1)
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
        cmd = "stop_log" if self.firmware == "main" else "logstop"
        self._send(cmd)
        deadline = time.time() + 3
        while time.time() < deadline:
            line = self._readline_timeout()
            if line is None:
                break
            if "__LOG_STOPPED__" in line or "Logging stopped" in line:
                print("[OK] Logging stopped")
                return

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

    def get_file(self, filename: str, out_dir: Path):
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
                return None
            if "__FILE_START__" in line:
                header = line
                break

        if not header:
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
    parser.add_argument("action", choices=["ls", "get", "all", "del"])
    parser.add_argument("filename", nargs="?", help="For get/del")
    parser.add_argument("--port")
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--out", default="data")
    parser.add_argument("--firmware", choices=["main", "loadcell"], default="main")
    parser.add_argument("--delete", action="store_true",
                        help="Delete from SD after download")
    parser.add_argument("--timeout", type=float, default=TIMEOUT)
    args = parser.parse_args()

    _, list_ports = _import_serial()
    port = args.port or auto_detect_port(list_ports)
    if not port:
        sys.exit("No serial port found. Specify --port /dev/ttyXXX")

    print(f"[+] {port} @ {args.baud} ({args.firmware})")
    link = TeensyLink(port, args.baud, args.timeout, args.firmware)
    out_dir = Path(args.out)

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

        elif args.action == "all":
            link.stop_logging()
            files = link.list_files()
            csv_files = [f for f in files if f["name"].upper().endswith(".CSV")]
            if not csv_files:
                print("No CSV files on SD.")
            else:
                print(f"[+] {len(csv_files)} CSV file(s)")
                for f in csv_files:
                    path = link.get_file(f["name"], out_dir)
                    if args.delete and path:
                        link.delete_file(f["name"])
                print(f"\n[OK] Saved to {out_dir.resolve()}")

    finally:
        link.close()


if __name__ == "__main__":
    main()
