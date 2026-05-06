"""
ARWalker BLE Client - Robust Auto-Reconnect Version

Arduino Nano 33 BLE와 통신하는 BLE 클라이언트입니다.
Nordic UART Service (NUS)를 사용합니다.

핵심 설계 원칙:
1. GUI 블로킹 없음 - 모든 BLE 작업은 별도 스레드에서 실행
2. 절대 끊기지 않는 연결 - 무한 자동 재연결 (exponential backoff)
3. Watchdog - 조용한 연결 끊김도 감지
4. 데이터 버퍼링 - 시그널 폭주 방지 (GUI 프리징 해결)
5. Proper asyncio - busy-wait 없음 (asyncio.gather 기반)
"""

import asyncio
import logging
import threading
import time
from typing import Optional
from datetime import datetime
from bleak import BleakClient, BleakScanner, BleakError
from bleak.backends.device import BLEDevice
from PyQt5.QtCore import QThread, pyqtSignal, QObject
from collections import deque

logger = logging.getLogger(__name__)

def _ts() -> str:
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]

# Nordic UART Service UUIDs
NUS_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
NUS_TX_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # Notify (데이터 수신)
NUS_RX_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  # Write (명령 전송)


class BleClientSignals(QObject):
    """BLE 클라이언트 시그널 - Qt 이벤트 시스템과 연결"""
    connected = pyqtSignal()
    disconnected = pyqtSignal()
    reconnecting = pyqtSignal(int)   # 재연결 시도 횟수
    data_received = pyqtSignal(str)
    diagnostics = pyqtSignal(dict)
    error = pyqtSignal(str)
    devices_found = pyqtSignal(list)  # List[BLEDevice]
    command_sent = pyqtSignal(str)    # 명령 전송 확인


class BleClientThread(QThread):
    """
    BLE 통신 전용 스레드 - 자동 재연결 + Watchdog

    아키텍처:
    - asyncio.gather로 3개 코루틴 동시 실행:
      1. command_processor: 명령 큐 처리 (500ms 타임아웃, busy-wait 아님)
      2. buffer_flusher: 40ms 간격으로 데이터 버퍼 플러시
      3. watchdog: 3초 간격으로 연결 상태 확인

    자동 재연결:
    - 연결이 끊기면 1초 → 5초 exponential backoff로 무한 재시도
    - 사용자가 수동으로 끊을 때만 재연결 중지
    """

    # 재연결 설정 — nRF52840 advertise 재시작 안정화 대기 포함
    RECONNECT_DELAY_INIT = 1.0    # 1.0초 (nRF52840 disconnect→advertise 안정화)
    RECONNECT_DELAY_MAX = 5.0     # 최대 5초
    RECONNECT_BACKOFF = 2.0       # 지수 증가

    # Watchdog 설정
    WATCHDOG_INTERVAL = 2.0       # 2초마다 연결 상태 확인
    DATA_TIMEOUT = 15.0           # 15초 데이터 없으면 조용한 연결 끊김으로 판단

    # 데이터 버퍼링 설정
    DATA_BUFFER_INTERVAL_MS = 40   # 40ms = 25Hz

    def __init__(self, parent=None):
        super().__init__(parent)
        self.signals = BleClientSignals()
        self._client: Optional[BleakClient] = None
        self._last_device: Optional[BLEDevice] = None
        self._running = False
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._command_queue: Optional[asyncio.Queue] = None
        self._is_connected = False

        # 재연결 상태
        self._reconnecting = False
        self._user_disconnected = False   # 사용자가 수동으로 끊었을 때 True
        self._ever_connected = False      # ★ 한 번이라도 연결 성공해야 자동 재연결 허용
        self._disconnecting = False       # _disconnect() 진행 중 → 콜백 중복 emit 방지

        # 데이터 버퍼링 (GUI 프리징 방지)
        self._data_buffer: deque = deque(maxlen=1000)
        self._thread_lock = threading.Lock()
        self._last_data_time = 0.0
        self._notify_count = 0
        self._rx_bytes = 0

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    def run(self):
        """QThread 메인 루프"""
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._command_queue = asyncio.Queue()
        self._running = True

        try:
            self._loop.run_until_complete(self._main_loop())
        except Exception as e:
            logger.error(f"BLE thread fatal error: {e}")
        finally:
            self._loop.run_until_complete(self._cleanup())
            self._loop.close()

    async def _main_loop(self):
        """비동기 메인 루프 - 3개 코루틴 동시 실행 (heartbeat 제거)"""
        try:
            await asyncio.gather(
                self._command_processor(),
                self._buffer_flusher(),
                self._watchdog(),
            )
        except asyncio.CancelledError:
            pass
        except Exception as e:
            logger.error(f"BLE main loop crashed: {e}")
            self.signals.error.emit(f"BLE loop error: {e}")

    async def _command_processor(self):
        """명령 큐 처리 - 500ms 타임아웃으로 적절한 블로킹"""
        while self._running:
            try:
                cmd = await asyncio.wait_for(
                    self._command_queue.get(), timeout=0.5
                )
                if cmd is None:  # 종료 센티널
                    break
                await self._process_command(cmd)
            except asyncio.TimeoutError:
                continue
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Command processor error: {e}")

    async def _buffer_flusher(self):
        """데이터 버퍼 주기적 플러시 (40ms 간격)"""
        while self._running:
            await asyncio.sleep(self.DATA_BUFFER_INTERVAL_MS / 1000.0)
            if not self._running:
                break
            try:
                self._flush_data_buffer()
            except Exception as e:
                logger.error(f"Buffer flush error: {e}")

    async def _watchdog(self):
        """연결 상태 감시 - 조용한 연결 끊김 감지"""
        while self._running:
            await asyncio.sleep(self.WATCHDOG_INTERVAL)
            if not self._running:
                break
            try:
                await self._check_connection_health()
            except Exception as e:
                logger.debug(f"Watchdog check error: {e}")

    async def _check_connection_health(self):
        """BLE 연결 상태 확인 — 첫 수동 연결 이후 예기치 않은 끊김만 자동 복구"""
        if not self._client or self._user_disconnected:
            return

        # is_connected == False 감지
        if self._is_connected and not self._client.is_connected:
            print(f"[{_ts()}][WATCHDOG] ✗ is_connected=False — link lost")
            logger.warning("Watchdog: Connection lost (is_connected=False)")
            self._is_connected = False
            self.signals.disconnected.emit()
            if self._should_auto_reconnect():
                await self._attempt_reconnect()
            return

        # 데이터 타임아웃: 조용한 연결 끊김으로 처리하고 조건부 자동 복구
        if self._is_connected and self._last_data_time > 0:
            elapsed = time.monotonic() - self._last_data_time
            print(f"[{_ts()}][WATCHDOG] data_age={elapsed:.1f}s  connected={self._client.is_connected}")
            self.signals.diagnostics.emit(self._snapshot_diagnostics(elapsed))
            if elapsed > self.DATA_TIMEOUT:
                print(f"[{_ts()}][WATCHDOG] ✗ NO DATA for {elapsed:.1f}s — force disconnect")
                logger.warning(f"Watchdog: No data for {elapsed:.1f}s")
                self._is_connected = False
                try:
                    await self._force_disconnect()
                except Exception:
                    pass
                self.signals.disconnected.emit()
                if self._should_auto_reconnect():
                    await self._attempt_reconnect()

    async def _cleanup(self):
        """종료 시 연결 정리"""
        if self._client:
            try:
                if self._client.is_connected:
                    await self._client.disconnect()
            except Exception:
                pass
            self._client = None

    async def _process_command(self, cmd: tuple):
        """명령 처리"""
        action = cmd[0]
        try:
            if action == "scan":
                await self._scan_devices()
            elif action == "connect":
                self._user_disconnected = False
                await self._connect(cmd[1])
            elif action == "disconnect":
                self._user_disconnected = True
                self._reconnecting = False
                await self._disconnect()
            elif action == "send":
                await self._send_data(cmd[1])
        except Exception as e:
            self.signals.error.emit(f"{action} failed: {str(e)}")

    async def _scan_devices(self):
        """BLE 디바이스 스캔"""
        print(f"\n[{_ts()}][SCAN] ▶ start — timeout=4s")
        try:
            devices = await BleakScanner.discover(timeout=4.0)
            named = [d for d in devices if d.name]
            print(f"[{_ts()}][SCAN] total={len(devices)} named={len(named)}")
            for d in named:
                rssi = getattr(d, "rssi", "?")
                print(f"[{_ts()}][SCAN]   {d.name!r:30s}  {d.address}  RSSI={rssi}")

            filtered = [d for d in devices if d.name and any(
                keyword in d.name for keyword in
                ["Nano", "Walker", "ExoBLE", "Arduino", "BLE"]
            )]
            result = filtered if filtered else named
            print(f"[{_ts()}][SCAN] ✓ emitting {len(result)} devices (filtered={len(filtered)})")
            self.signals.devices_found.emit(result)
        except Exception as e:
            print(f"[{_ts()}][SCAN] ✗ {type(e).__name__}: {e}")
            self.signals.error.emit(f"Scan error: {str(e)}")
            self.signals.devices_found.emit([])

    async def _connect(self, device: BLEDevice):
        """BLE 디바이스에 연결"""
        print(f"\n[{_ts()}][CONNECT] ▶ target={device.name!r} addr={device.address}")
        try:
            self._last_device = device

            # ── 구간 1: 기존 클라이언트 정리 ─────────────────────────
            if self._client:
                print(f"[{_ts()}][CONNECT] old client exists — cleaning up...")
                old_client = self._client
                self._client = None
                try:
                    if old_client.is_connected:
                        await old_client.disconnect()
                        print(f"[{_ts()}][CONNECT] old client disconnected")
                except Exception as e:
                    print(f"[{_ts()}][CONNECT] old client cleanup error: {e}")
                await asyncio.sleep(0.2)

            # ── 구간 2: GATT 연결 ─────────────────────────────────────
            print(f"[{_ts()}][CONNECT] BleakClient({device.address}) created")
            self._client = BleakClient(
                device.address,
                disconnected_callback=self._on_disconnect_callback
            )

            print(f"[{_ts()}][CONNECT] wait_for connect (timeout=4s)...")
            await asyncio.wait_for(self._client.connect(), timeout=4.0)
            print(f"[{_ts()}][CONNECT] connect() done — is_connected={self._client.is_connected}")

            # ── 구간 3: Notify 구독 ───────────────────────────────────
            if self._client.is_connected:
                print(f"[{_ts()}][CONNECT] start_notify TX={NUS_TX_UUID[:8]}...")
                await self._client.start_notify(NUS_TX_UUID, self._on_notify)
                self._is_connected = True
                self._ever_connected = True
                self._last_data_time = 0.0  # 첫 패킷 수신 시 설정 — 연결만 된 상태에서 타임아웃 방지
                self._notify_count = 0
                self._rx_bytes = 0
                self._reconnecting = False
                print(f"[{_ts()}][CONNECT] ✓ READY — emitting connected")
                self.signals.connected.emit()
                logger.info(f"Connected to {device.name}")
            else:
                raise BleakError("Connection not established")

        except asyncio.TimeoutError:
            print(f"[{_ts()}][CONNECT] ✗ TIMEOUT (4s) — no response from device")
            if self._client:
                self._client = None
            raise
        except Exception as e:
            print(f"[{_ts()}][CONNECT] ✗ {type(e).__name__}: {e}")
            if self._client:
                try:
                    if self._client.is_connected:
                        await self._client.disconnect()
                except Exception:
                    pass
                self._client = None
            logger.error(f"Connection error: {e}")
            raise

    async def _disconnect(self):
        """사용자 요청에 의한 연결 해제

        _disconnecting=True 구간 동안 콜백 emit을 억제해 double emit 방지.
        """
        print(f"\n[{_ts()}][DISCONNECT] ▶ user-requested")
        self._is_connected = False
        self._ever_connected = False
        self._disconnecting = True        # ← 콜백 억제 시작
        client = self._client
        self._client = None
        try:
            if client:
                if client.is_connected:
                    # ── 구간 1: Notify 해제 ───────────────────────────
                    print(f"[{_ts()}][DISCONNECT] stop_notify...")
                    try:
                        await client.stop_notify(NUS_TX_UUID)
                        print(f"[{_ts()}][DISCONNECT] ✓ notify stopped")
                    except Exception as e:
                        print(f"[{_ts()}][DISCONNECT] stop_notify error: {e}")
                    # ── 구간 2: GATT 해제 ────────────────────────────
                    print(f"[{_ts()}][DISCONNECT] disconnect()...")
                    await client.disconnect()
                    print(f"[{_ts()}][DISCONNECT] ✓ GATT disconnected")
                else:
                    print(f"[{_ts()}][DISCONNECT] client not connected — skip GATT ops")
            else:
                print(f"[{_ts()}][DISCONNECT] client was None")
        except Exception as e:
            print(f"[{_ts()}][DISCONNECT] ✗ {type(e).__name__}: {e}")
            logger.error(f"Disconnect error: {e}")
        finally:
            self._disconnecting = False   # ← 억제 해제
            self.signals.disconnected.emit()
            print(f"[{_ts()}][DISCONNECT] ✓ done — disconnected emitted")

    async def _force_disconnect(self):
        """강제 연결 해제 (watchdog에서 사용 - 시그널 emit 안함)

        self._client를 먼저 None으로 세팅해 _on_disconnect_callback stale 처리.
        """
        client = self._client
        self._client = None   # ← 먼저 None, 이후 콜백은 stale로 무시됨
        try:
            if client:
                if client.is_connected:
                    try:
                        await client.stop_notify(NUS_TX_UUID)
                    except Exception:
                        pass
                    await client.disconnect()
        except Exception:
            pass

    async def _send_data(self, data: str):
        """BLE로 명령 전송"""
        cmd_repr = data.strip()
        if not self._client or not self._is_connected:
            print(f"[{_ts()}][SEND →] ✗ NOT CONNECTED — cmd={cmd_repr!r}")
            self.signals.error.emit("Not connected - command not sent")
            return

        print(f"[{_ts()}][SEND →] cmd={cmd_repr!r}  len={len(data)}B")
        try:
            await self._client.write_gatt_char(
                NUS_RX_UUID,
                data.encode('utf-8'),
                response=False
            )
            print(f"[{_ts()}][SEND ✓] cmd={cmd_repr!r}")
            self.signals.command_sent.emit(cmd_repr)
        except BleakError as e:
            print(f"[{_ts()}][SEND ✗] BleakError: {e}")
            self.signals.error.emit(f"Send failed: {str(e)}")
            if self._client and not self._client.is_connected:
                self._is_connected = False
                self.signals.disconnected.emit()
        except Exception as e:
            print(f"[{_ts()}][SEND ✗] {type(e).__name__}: {e}")
            self.signals.error.emit(f"Send error: {str(e)}")

    def _flush_data_buffer(self):
        """버퍼된 데이터를 한 번에 emit (GUI 프리징 방지)

        threading.Lock으로 bleak 콜백 스레드와 안전하게 동기화
        emit은 lock 밖에서 호출 — lock 중 emit 예외로 인한 스레드 crash 방지
        """
        combined_data = None
        with self._thread_lock:
            if self._data_buffer:
                combined_data = ''.join(self._data_buffer)
                self._data_buffer.clear()
        if combined_data:
            self.signals.data_received.emit(combined_data)

    def _snapshot_diagnostics(self, data_age: float = 0.0) -> dict:
        with self._thread_lock:
            buffered = len(self._data_buffer)
        return {
            "connected": self._is_connected,
            "reconnecting": self._reconnecting,
            "ever_connected": self._ever_connected,
            "data_age": data_age,
            "notify_count": self._notify_count,
            "rx_bytes": self._rx_bytes,
            "buffered_batches": buffered,
        }

    def _should_auto_reconnect(self) -> bool:
        return (
            not self._user_disconnected
            and self._ever_connected
            and self._last_device is not None
            and self._running
        )

    def _on_notify(self, sender, data: bytearray):
        """Notify 콜백 - 데이터 수신 (bleak의 콜백 스레드에서 호출됨!)"""
        try:
            self._last_data_time = time.monotonic()
            text = data.decode('utf-8', errors='ignore')
            # 100패킷마다 1회 출력 (스팸 방지)
            self._notify_count += 1
            self._rx_bytes += len(data)
            if self._notify_count % 100 == 0:
                hex_preview = data.hex()[:32] + ("…" if len(data) > 16 else "")
                print(f"[{_ts()}][RECV ←] #{self._notify_count:6d}  {len(data):3d}B | {hex_preview}")
            with self._thread_lock:
                self._data_buffer.append(text)
        except Exception as e:
            print(f"[{_ts()}][RECV ←] ✗ decode error: {e}")
            logger.error(f"Notify decode error: {e}")

    def _on_disconnect_callback(self, client):
        """연결 해제 콜백 - BLE 스택에서 호출됨

        _disconnecting=True 이면 _disconnect()가 진행 중 → 중복 emit 방지.
        """
        if self._disconnecting:
            print(f"[{_ts()}][DISCONNECT CB] _disconnect() in progress — ignored (no double emit)")
            return
        if client is not self._client:
            print(f"[{_ts()}][DISCONNECT CB] stale client callback — ignored")
            return

        print(f"[{_ts()}][DISCONNECT CB] ✗ BLE stack fired disconnect — was_connected={self._is_connected}")
        logger.warning("BLE disconnected via callback")
        self._is_connected = False
        self.signals.disconnected.emit()
        if self._should_auto_reconnect() and self._loop and self._loop.is_running():
            print(f"[{_ts()}][DISCONNECT CB] scheduling auto-reconnect")
            asyncio.run_coroutine_threadsafe(self._attempt_reconnect(), self._loop)
        else:
            print(f"[{_ts()}][DISCONNECT CB] disconnected emitted — auto-reconnect disabled")

    async def _attempt_reconnect(self):
        """자동 재연결 시도 (exponential backoff) - 무한 재시도

        1초부터 시작하여 최대 5초 간격으로 재시도합니다.
        사용자가 수동으로 끊거나 스레드가 종료될 때까지 계속합니다.
        """
        if self._reconnecting:
            return
        self._reconnecting = True

        delay = self.RECONNECT_DELAY_INIT
        attempt = 0

        while self._running and not self._user_disconnected:
            attempt += 1
            logger.info(f"Reconnect attempt {attempt}...")
            self.signals.reconnecting.emit(attempt)

            try:
                await self._connect(self._last_device)
                if self._client and self._client.is_connected:
                    logger.info("Reconnected successfully!")
                    self._reconnecting = False
                    return
            except Exception as e:
                logger.debug(f"Reconnect attempt {attempt} failed: {e}")

            # 대기 (0.1초 단위로 확인하여 빠른 종료 지원)
            for _ in range(int(delay * 10)):
                if not self._running or self._user_disconnected:
                    self._reconnecting = False
                    return
                await asyncio.sleep(0.1)

            delay = min(delay * self.RECONNECT_BACKOFF, self.RECONNECT_DELAY_MAX)

        self._reconnecting = False

    # === 외부 인터페이스 (메인 스레드에서 호출) ===

    def _enqueue_command(self, cmd: tuple):
        """명령 큐에 추가 (thread-safe)"""
        if self._loop and self._command_queue:
            asyncio.run_coroutine_threadsafe(
                self._command_queue.put(cmd),
                self._loop
            )

    def scan(self):
        """디바이스 스캔 요청"""
        self._enqueue_command(("scan",))

    def connect_device(self, device: BLEDevice):
        """디바이스 연결 요청"""
        self._enqueue_command(("connect", device))

    def disconnect_device(self):
        """연결 해제 요청"""
        self._enqueue_command(("disconnect",))

    def send_command(self, command: str):
        """명령 전송 - 줄바꿈 자동 추가"""
        if not command.endswith('\n'):
            command += '\n'
        self._enqueue_command(("send", command))

    def stop(self):
        """스레드 안전하게 종료"""
        self._running = False
        self._user_disconnected = True
        # 센티널 전송으로 command processor 종료
        if self._loop and self._loop.is_running() and self._command_queue:
            asyncio.run_coroutine_threadsafe(
                self._command_queue.put(None),
                self._loop
            )
        self.wait(3000)  # 최대 3초 대기
