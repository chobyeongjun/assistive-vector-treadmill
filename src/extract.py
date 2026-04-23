#!/usr/bin/env python3
"""
extract.py — SD card data extractor for Teensy firmware (Treadmill_main / Loadcell_Monitor)

SD 카드를 뽑지 않고 USB Serial로 파일 목록 조회, 다운로드, 삭제 가능.
다운로드한 파일은 로컬 폴더 AND/OR Google Drive에 저장.

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
  python3 src/extract.py all --drive
  python3 src/extract.py all --firmware loadcell

Options:
  --port    <port>    Serial port (default: auto-detect Teensy)
  --baud    <rate>    Baud rate (default: 115200)
  --out     <dir>     Local save directory (default: ./data)
  --drive             Also upload to Google Drive
  --drive-folder <n>  Google Drive folder name (default: assistive-vector-treadmill)
  --firmware <name>   main | loadcell (affects stop command; default: main)
  --delete            Delete file from SD after successful download
  --timeout <s>       Response timeout in seconds (default: 10)

Google Drive setup (one-time):
  1. Google Cloud Console → 새 프로젝트 → Drive API 활성화
  2. OAuth 2.0 사용자 인증 정보 생성 → credentials.json 다운로드
  3. credentials.json을 이 스크립트 옆(src/)에 놓거나 --creds 로 경로 지정
  4. 첫 실행 시 브라우저 인증 → token.json 자동 저장
"""

import sys
import argparse
import time
import os
import re
from pathlib import Path

# ──────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────

BAUD = 115200
TIMEOUT = 10   # seconds for response
CHUNK = 4096


def _import_serial():
    try:
        import serial
        import serial.tools.list_ports
        return serial, serial.tools.list_ports
    except ImportError:
        sys.exit("pyserial not found. Run: pip3 install pyserial")


def auto_detect_port(list_ports):
    """Return first Teensy/USB-Serial port found, else None."""
    teensy_ids = [
        (0x16C0, 0x0483),  # Teensy USB Serial
        (0x16C0, 0x0487),  # Teensy rawhid + serial
    ]
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
    # Last resort: any serial port
    all_ports = list(list_ports.comports())
    if all_ports:
        return all_ports[0].device
    return None


# ──────────────────────────────────────────────────────────────────
# TeensyLink: thin wrapper around pyserial
# ──────────────────────────────────────────────────────────────────

class TeensyLink:
    def __init__(self, port: str, baud: int = BAUD, timeout: float = TIMEOUT,
                 firmware: str = "main"):
        serial, _ = _import_serial()
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.timeout = timeout
        self.firmware = firmware
        time.sleep(1.5)   # wait for Teensy to boot after port open
        self._flush()

    def _flush(self):
        self.ser.reset_input_buffer()

    def _send(self, cmd: str):
        line = (cmd.strip() + "\n").encode()
        self.ser.write(line)
        self.ser.flush()

    def _readline_timeout(self) -> str | None:
        """Read one text line with global timeout. Returns None on timeout."""
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
        """Read exactly n bytes with timeout."""
        deadline = time.time() + self.timeout + n / BAUD
        data = b""
        while len(data) < n and time.time() < deadline:
            chunk = self.ser.read(n - len(data))
            data += chunk
        return data

    def stop_logging(self):
        """Stop any active SD logging before file operations."""
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
        # Not necessarily an error — may not have been logging

    def list_files(self) -> list[dict]:
        """Return [{'name': str, 'size': int}, ...]"""
        self._flush()
        self._send("ls")
        files = []
        deadline = time.time() + self.timeout
        in_list = False
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

    def get_file(self, filename: str, out_dir: Path) -> Path | None:
        """Download a file from SD to out_dir. Returns local path or None."""
        self._flush()
        self._send(f"get {filename}")

        # Wait for header: __FILE_START__,name,size
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

        # Parse: __FILE_START__,filename,size
        parts = header.split(",")
        if len(parts) < 3:
            print(f"[ERROR] Bad header: {header}")
            return None
        try:
            file_size = int(parts[-1].strip())
        except ValueError:
            print(f"[ERROR] Bad size in header: {header}")
            return None

        # Read binary data (size bytes)
        print(f"  Downloading {filename} ({file_size:,} bytes)...", end=" ", flush=True)
        data = self._readbytes_exact(file_size)

        # Drain __FILE_END__ marker
        end_deadline = time.time() + 3
        while time.time() < end_deadline:
            line = self._readline_timeout()
            if line is None:
                break
            if "__FILE_END__" in line:
                break

        if len(data) != file_size:
            print(f"[WARN] Got {len(data)} bytes, expected {file_size}")

        # Save
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
# Google Drive upload
# ──────────────────────────────────────────────────────────────────

def _get_drive_service(creds_path: Path):
    """Return authenticated Google Drive service. Caches token.json next to creds."""
    try:
        from google.oauth2.credentials import Credentials
        from google_auth_oauthlib.flow import InstalledAppFlow
        from google.auth.transport.requests import Request
        from googleapiclient.discovery import build
    except ImportError:
        sys.exit(
            "Google API client not found.\n"
            "Run: pip3 install google-api-python-client google-auth-httplib2 google-auth-oauthlib"
        )

    SCOPES = ["https://www.googleapis.com/auth/drive.file"]
    token_path = creds_path.parent / "token.json"
    creds = None

    if token_path.exists():
        creds = Credentials.from_authorized_user_file(str(token_path), SCOPES)

    if not creds or not creds.valid:
        if creds and creds.expired and creds.refresh_token:
            creds.refresh(Request())
        else:
            flow = InstalledAppFlow.from_client_secrets_file(str(creds_path), SCOPES)
            creds = flow.run_local_server(port=0)
        token_path.write_text(creds.to_json())

    return build("drive", "v3", credentials=creds)


def _get_or_create_folder(service, folder_name: str) -> str:
    """Return Google Drive folder ID, creating it if needed."""
    q = (f"name='{folder_name}' and mimeType='application/vnd.google-apps.folder'"
         f" and trashed=false")
    res = service.files().list(q=q, fields="files(id,name)").execute()
    items = res.get("files", [])
    if items:
        return items[0]["id"]
    meta = {"name": folder_name,
            "mimeType": "application/vnd.google-apps.folder"}
    folder = service.files().create(body=meta, fields="id").execute()
    return folder["id"]


def upload_to_drive(local_path: Path, folder_id: str, service) -> str:
    """Upload file to Drive folder. Return file URL."""
    from googleapiclient.http import MediaFileUpload
    media = MediaFileUpload(str(local_path), resumable=True)
    meta = {"name": local_path.name, "parents": [folder_id]}
    f = service.files().create(body=meta, media_body=media, fields="id,webViewLink").execute()
    return f.get("webViewLink", f["id"])


# ──────────────────────────────────────────────────────────────────
# CLI entry point
# ──────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Extract SD card files from Teensy over USB serial")
    parser.add_argument("action", choices=["ls", "get", "all", "del"],
                        help="ls=list, get=download one, all=download all CSV, del=delete")
    parser.add_argument("filename", nargs="?", help="Filename for get/del")
    parser.add_argument("--port", help="Serial port (auto-detect if omitted)")
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--out", default="data", help="Local output directory")
    parser.add_argument("--firmware", choices=["main", "loadcell"], default="main")
    parser.add_argument("--delete", action="store_true",
                        help="Delete from SD after successful download")
    parser.add_argument("--drive", action="store_true",
                        help="Upload to Google Drive after download")
    parser.add_argument("--drive-folder", default="assistive-vector-treadmill",
                        dest="drive_folder")
    parser.add_argument("--creds", default=str(Path(__file__).parent / "credentials.json"),
                        help="Path to Google OAuth credentials.json")
    parser.add_argument("--timeout", type=float, default=TIMEOUT)
    args = parser.parse_args()

    _, list_ports = _import_serial()

    # Resolve serial port
    port = args.port or auto_detect_port(list_ports)
    if not port:
        sys.exit("No serial port found. Specify --port /dev/ttyXXX")
    print(f"[+] Connecting to {port} @ {args.baud} baud (firmware={args.firmware})")

    link = TeensyLink(port, args.baud, args.timeout, args.firmware)
    out_dir = Path(args.out)

    # Google Drive service (lazy init)
    drive_service = None
    drive_folder_id = None

    def _ensure_drive():
        nonlocal drive_service, drive_folder_id
        if drive_service is None:
            creds_path = Path(args.creds)
            if not creds_path.exists():
                sys.exit(
                    f"credentials.json not found at {creds_path}\n"
                    "See --help for Google Drive setup instructions."
                )
            print("[Drive] Authenticating...")
            drive_service = _get_drive_service(creds_path)
            drive_folder_id = _get_or_create_folder(drive_service, args.drive_folder)
            print(f"[Drive] Folder: {args.drive_folder} (id={drive_folder_id})")

    def _maybe_upload(path: Path):
        if args.drive and path:
            _ensure_drive()
            url = upload_to_drive(path, drive_folder_id, drive_service)
            print(f"  [Drive] {path.name} → {url}")

    try:
        if args.action == "ls":
            files = link.list_files()
            if not files:
                print("(no files or SD error)")
            else:
                print(f"{'Name':<20} {'Size':>10}")
                print("-" * 32)
                for f in files:
                    print(f"{f['name']:<20} {f['size']:>10,}")
                print(f"\n{len(files)} file(s)")

        elif args.action == "get":
            if not args.filename:
                sys.exit("Specify filename: extract.py get <filename>")
            link.stop_logging()
            path = link.get_file(args.filename, out_dir)
            _maybe_upload(path)
            if args.delete and path:
                link.delete_file(args.filename)

        elif args.action == "del":
            if not args.filename:
                sys.exit("Specify filename: extract.py del <filename>")
            link.delete_file(args.filename)

        elif args.action == "all":
            link.stop_logging()
            files = link.list_files()
            csv_files = [f for f in files
                         if f["name"].upper().endswith(".CSV")]
            if not csv_files:
                print("No CSV files found on SD card.")
            else:
                print(f"[+] {len(csv_files)} CSV file(s) found")
                for f in csv_files:
                    path = link.get_file(f["name"], out_dir)
                    _maybe_upload(path)
                    if args.delete and path:
                        link.delete_file(f["name"])
                print(f"\n[OK] All files saved to {out_dir.resolve()}")

    finally:
        link.close()


if __name__ == "__main__":
    main()
