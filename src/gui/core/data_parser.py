"""
ARWalker Data Parser - High Performance Version

BLE로부터 수신한 패킷을 파싱하여 WalkerData 객체로 변환합니다.

패킷 포맷: "SW11c<d0>n<d1>n...<d10>n"
- 11개 데이터 (펌웨어 BleComm.cpp 순서와 일치):
  [0-1]  L/R GCP (%)
  [2-3]  L/R Pitch (deg)
  [4-5]  L/R GyroY (deg/s)
  [6-7]  L/R DesForce (N)
  [8-9]  L/R ActForce (N)
  [10]   Mark
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
    l_gyro_y: float = 0.0
    r_gyro_y: float = 0.0

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

    EXPECTED_COUNT = 11  # 펌웨어가 11개 데이터 전송 (mark 포함)

    # 데이터 범위 제한 (유효 범위 밖 = 스파이크)
    VALID_RANGES = {
        'gcp': (-0.1, 1.5),
        'pitch': (-90, 90),
        'force': (-50, 350),
        'gyro': (-500, 500),
    }

    # 스파이크 감지
    MAX_CHANGE = {
        'gcp': 0.3,
        'pitch': 20,
        'force': 50,
        'gyro': 100,
    }

    def __init__(self):
        self._buffer = ''  # ★ 단순 문자열 버퍼 (io.StringIO 대비 오버헤드 감소)
        self._sample_count = 0
        self._parse_errors = 0
        self._max_buffer_size = 4096
        self._prev_data: WalkerData = None
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

    def _check_spike(self, new_val: float, old_val: float, change_key: str) -> bool:
        """스파이크 여부 확인 (True = 스파이크 발생)"""
        if change_key not in self.MAX_CHANGE:
            return False
        return abs(new_val - old_val) > self.MAX_CHANGE[change_key]

    def _filter_gcp_value(self, new_val: float, old_val: float) -> float:
        """GCP 전용 필터 - Gait Cycle Reset 허용"""
        if not self._validate_value(new_val, 'gcp'):
            self._spike_count += 1
            return old_val if old_val is not None else 0.0

        if old_val is None:
            return new_val

        # GCP 리셋 감지: 이전값이 높고(>0.7) 새값이 낮으면(<0.3) 정상 리셋
        if old_val > 0.7 and new_val < 0.3:
            return new_val

        if self._check_spike(new_val, old_val, 'gcp'):
            self._spike_count += 1
            return old_val

        return new_val

    def _filter_value(self, new_val: float, old_val: float, range_key: str, change_key: str) -> float:
        """스파이크 필터링"""
        if not self._validate_value(new_val, range_key):
            self._spike_count += 1
            return old_val if old_val is not None else 0.0

        if old_val is not None and self._check_spike(new_val, old_val, change_key):
            self._spike_count += 1
            return old_val

        return new_val

    def _create_walker_data(self, values: List[float], count: int) -> Optional[WalkerData]:
        """값 리스트를 WalkerData 객체로 변환

        펌웨어 BleComm.cpp 데이터 순서 (11개):
        [0] L_GCP  [1] R_GCP
        [2] L_Pitch  [3] R_Pitch
        [4] L_GyroY  [5] R_GyroY
        [6] L_DesForce  [7] R_DesForce
        [8] L_ActForce  [9] R_ActForce
        [10] Mark
        """
        if count != 11:
            return None

        prev = self._prev_data
        if prev is None:
            filtered = WalkerData(
                l_gcp=values[0] if self._validate_value(values[0], 'gcp') else 0.0,
                r_gcp=values[1] if self._validate_value(values[1], 'gcp') else 0.0,
                l_pitch=values[2] if self._validate_value(values[2], 'pitch') else 0.0,
                r_pitch=values[3] if self._validate_value(values[3], 'pitch') else 0.0,
                l_gyro_y=values[4] if self._validate_value(values[4], 'gyro') else 0.0,
                r_gyro_y=values[5] if self._validate_value(values[5], 'gyro') else 0.0,
                l_des_force=values[6] if self._validate_value(values[6], 'force') else 0.0,
                r_des_force=values[7] if self._validate_value(values[7], 'force') else 0.0,
                l_act_force=values[8] if self._validate_value(values[8], 'force') else 0.0,
                r_act_force=values[9] if self._validate_value(values[9], 'force') else 0.0,
                mark=int(values[10]),
            )
        else:
            filtered = WalkerData(
                l_gcp=self._filter_gcp_value(values[0], prev.l_gcp),
                r_gcp=self._filter_gcp_value(values[1], prev.r_gcp),
                l_pitch=self._filter_value(values[2], prev.l_pitch, 'pitch', 'pitch'),
                r_pitch=self._filter_value(values[3], prev.r_pitch, 'pitch', 'pitch'),
                l_gyro_y=self._filter_value(values[4], prev.l_gyro_y, 'gyro', 'gyro'),
                r_gyro_y=self._filter_value(values[5], prev.r_gyro_y, 'gyro', 'gyro'),
                l_des_force=self._filter_value(values[6], prev.l_des_force, 'force', 'force'),
                r_des_force=self._filter_value(values[7], prev.r_des_force, 'force', 'force'),
                l_act_force=self._filter_value(values[8], prev.l_act_force, 'force', 'force'),
                r_act_force=self._filter_value(values[9], prev.r_act_force, 'force', 'force'),
                mark=int(values[10]),
            )
        self._prev_data = filtered
        return filtered

    def reset(self):
        """파서 상태 초기화"""
        self._buffer = ''
        self._sample_count = 0
        self._parse_errors = 0
        self._prev_data = None
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
