"""
ARWalker Data Parser - High Performance Version

BLE로부터 수신한 패킷을 파싱하여 WalkerData 객체로 변환합니다.

패킷 포맷: "SW9c<d0>n<d1>n...<d8>n"
- 9개 데이터:
  [0-1] L/R GCP (%)
  [2-3] L/R Pitch (deg)
  [4-5] L/R DesForce (N)
  [6-7] L/R ActForce (N)
  [8]   Mark
"""

from dataclasses import dataclass
from typing import Optional, List
from collections import deque


@dataclass
class WalkerData:
    """Walker 센서 데이터 구조체"""

    # GCP (Gait Cycle Percentage) - 0~1
    l_gcp: float = 0.0
    r_gcp: float = 0.0

    # IMU
    l_pitch: float = 0.0
    r_pitch: float = 0.0

    # Force (N)
    l_des_force: float = 0.0
    r_des_force: float = 0.0
    l_act_force: float = 0.0
    r_act_force: float = 0.0

    mark: int = 0
    timestamp: int = 0


class WalkerDataParser:
    """
    Walker 패킷 파서 - 고성능 버전

    ★★ 성능 최적화:
    1. 단순 문자열 버퍼: io.StringIO 객체 생성/관리 오버헤드 제거
    2. 패킷 제한 없음: 모든 가용 패킷 즉시 처리 (burst 시 지연 방지)
    3. 스로틀링은 main_window 타이머 레벨에서 관리
    """

    EXPECTED_COUNT = 9   # GCP×2, Pitch×2, DesForce×2, ActForce×2, Mark

    # 데이터 범위 제한 (유효 범위 밖 = 즉시 기각, 패킷 깨짐 감지용)
    VALID_RANGES = {
        'gcp':   (-0.1, 1.1),   # 0~1 정상, 소폭 마진
        'pitch': (-90, 90),     # 물리적 최대 범위
        'force': (-20, 150),    # AK60 최대 70N + 넉넉한 마진
    }

    def __init__(self):
        self._buffer = ''  # ★ 단순 문자열 버퍼 (io.StringIO 대비 오버헤드 감소)
        self._sample_count = 0
        self._parse_errors = 0
        self._max_buffer_size = 4096
        self._spike_count = 0

    @property
    def sample_count(self) -> int:
        return self._sample_count

    @property
    def parse_errors(self) -> int:
        return self._parse_errors

    @property
    def spike_count(self) -> int:
        return self._spike_count

    def feed(self, data: str) -> List[WalkerData]:
        """
        데이터를 버퍼에 추가하고 완성된 패킷들을 반환합니다.

        ★★ 성능 최적화:
        - 단순 문자열 버퍼 (io.StringIO 객체 생성 오버헤드 제거)
        - 패킷 수 제한 없음 (모든 가용 패킷 즉시 처리 → 지연 방지)
        - 스로틀링은 main_window 레벨에서 관리
        """
        self._buffer += data

        if len(self._buffer) > self._max_buffer_size:
            self._compact_buffer()

        results = []
        parse_pos = 0
        buf = self._buffer

        while True:
            start_idx = buf.find('S', parse_pos)
            if start_idx == -1:
                parse_pos = len(buf)
                break

            if start_idx + 5 > len(buf):
                parse_pos = start_idx
                break

            if buf[start_idx + 1] != 'W':
                parse_pos = start_idx + 1
                continue

            c_idx = buf.find('c', start_idx + 2, start_idx + 10)
            if c_idx == -1:
                parse_pos = start_idx + 1
                continue

            try:
                count = int(buf[start_idx + 2:c_idx])
            except ValueError:
                self._parse_errors += 1
                parse_pos = start_idx + 1
                continue

            data_start = c_idx + 1

            remaining = buf[data_start:]
            n_count = remaining.count('n')
            if n_count < count:
                parse_pos = start_idx
                break

            values = []
            current_pos = data_start
            parse_success = True

            for _ in range(count):
                n_idx = buf.find('n', current_pos)
                if n_idx == -1:
                    parse_success = False
                    break
                try:
                    values.append(int(buf[current_pos:n_idx]) / 100.0)
                except ValueError:
                    parse_success = False
                    break
                current_pos = n_idx + 1

            if parse_success and len(values) == count:
                walker_data = self._create_walker_data(values, count)
                if walker_data:
                    self._sample_count += 1
                    walker_data.timestamp = self._sample_count
                    results.append(walker_data)
                parse_pos = current_pos
            else:
                self._parse_errors += 1
                parse_pos = start_idx + 1

        # ★ 처리된 부분만 제거 (단순 슬라이싱)
        if parse_pos > 0:
            self._buffer = self._buffer[parse_pos:]

        return results

    def _compact_buffer(self):
        """버퍼 정리 - 마지막 'S' 이후만 유지"""
        last_s = self._buffer.rfind('S')
        if last_s > 0:
            self._buffer = self._buffer[last_s:]
        elif len(self._buffer) > 512:
            self._buffer = self._buffer[-512:]

    def _validate_value(self, value: float, range_key: str) -> bool:
        """값이 유효 범위 내인지 확인"""
        if range_key not in self.VALID_RANGES:
            return True
        min_val, max_val = self.VALID_RANGES[range_key]
        return min_val <= value <= max_val

    def _create_walker_data(self, values: List[float], count: int) -> Optional[WalkerData]:
        """값 리스트를 WalkerData 객체로 변환

        [0] L_GCP  [1] R_GCP
        [2] L_Pitch  [3] R_Pitch
        [4] L_DesForce  [5] R_DesForce
        [6] L_ActForce  [7] R_ActForce
        [8] Mark
        """
        if count != self.EXPECTED_COUNT:
            return None

        def clamp(val: float, key: str) -> float:
            lo, hi = self.VALID_RANGES[key]
            if val < lo or val > hi:
                self._spike_count += 1
                return 0.0
            return val

        data = WalkerData(
            l_gcp=clamp(values[0], 'gcp'),
            r_gcp=clamp(values[1], 'gcp'),
            l_pitch=clamp(values[2], 'pitch'),
            r_pitch=clamp(values[3], 'pitch'),
            l_des_force=clamp(values[4], 'force'),
            r_des_force=clamp(values[5], 'force'),
            l_act_force=clamp(values[6], 'force'),
            r_act_force=clamp(values[7], 'force'),
            mark=int(values[8]),
        )
        return data

    def reset(self):
        """파서 상태 초기화"""
        self._buffer = ''
        self._sample_count = 0
        self._parse_errors = 0
        self._spike_count = 0


class DataBuffer:
    """효율적인 실시간 데이터 버퍼 (deque 기반)"""

    def __init__(self, max_size: int = 500):
        self.max_size = max_size
        self._data: deque = deque(maxlen=max_size)

    def append(self, value: float):
        self._data.append(value)

    def get_array(self):
        """numpy 배열로 반환 (플로팅용)"""
        import numpy as np
        return np.asarray(self._data, dtype=np.float32)

    def __len__(self):
        return len(self._data)

    def clear(self):
        self._data.clear()

    @property
    def last(self) -> Optional[float]:
        return self._data[-1] if self._data else None
