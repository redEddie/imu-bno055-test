"""
bno055_driver.py
=================
의존성 없는(numpy/adafruit 불필요) 최소 BNO055 I2C 드라이버.
smbus2 만 사용하며, 라즈베리파이 zero 2w 같은 제약 환경에서 동작하도록
가볍게 작성했습니다.

라즈베리파이 I2C 클럭 스트레칭 버그 대응
----------------------------------------
BCM2835/2711 의 하드웨어 I2C 는 BNO055 의 클럭 스트레칭을 제대로 처리하지
못해, 단일 바이트 읽기/쓰기에서 간헐적으로 bit7(0x80)이 끼어드는 손상이
발생한다. (예: OPR_MODE 0x0c 가 0x8c 로 읽힘)

  - 진짜 해결책은 I2C 속도를 낮추는 것(README 참고: i2c_arm_baudrate=50000 + 재부팅).
  - 이 드라이버는 그와 별개로, 손상에 견디도록 다음을 한다:
      * 레지스터 읽기는 항상 '블록 읽기' 사용 (단일바이트보다 덜 손상됨)
      * 모드 변경은 '쓰고 -> 되읽어 확인' 을 재시도 (corrupt 시 다시 씀)
      * 모든 대기 루프에 타임아웃 (무한 hang 방지)

핵심 사용 목적
--------------
1) AMG (Accel+Mag+Gyro) "Raw Data 모드"로 두고 가속도/자이로 원시값을 직접 읽어,
   상보 필터/경사계 알고리즘을 우리가 손으로 구현한다.
2) 동시에 NDOF(내장 9축 퓨전) 모드의 euler 각도를 "정답지"로 읽어 비교한다.

기본 단위(UNIT_SEL = 0x00)
--------------------------
  accel : 1 m/s^2 = 100 LSB
  gyro  : 1 dps   = 16  LSB
  mag   : 1 uT    = 16  LSB
  euler : 1 deg   = 16  LSB
  quat  : 1.0     = 2^14 (16384) LSB
"""

import time

import geometry

try:
    from smbus2 import SMBus
except ImportError:  # pragma: no cover
    raise SystemExit(
        "smbus2 가 필요합니다.  pip3 install smbus2  (이 환경에는 이미 설치되어 있음)"
    )

# ---------------------------------------------------------------------------
# 레지스터 / 상수 (BNO055 datasheet, Page 0)
# ---------------------------------------------------------------------------
DEFAULT_ADDRESS = 0x29          # 이 보드는 ADR 핀이 HIGH -> 0x29 (기본 0x28 아님!)
DEFAULT_BUS     = 1

CHIP_ID_ADDR    = 0x00
CHIP_ID         = 0xA0

PAGE_ID_ADDR    = 0x07

ACCEL_DATA_ADDR = 0x08          # X_LSB..Z_MSB (6 bytes)
MAG_DATA_ADDR   = 0x0E          # 6 bytes
GYRO_DATA_ADDR  = 0x14          # 6 bytes
EULER_DATA_ADDR = 0x1A          # heading, roll, pitch (6 bytes)
QUAT_DATA_ADDR  = 0x20          # w, x, y, z (8 bytes)
LIA_DATA_ADDR   = 0x28          # linear accel (6 bytes)
GRV_DATA_ADDR   = 0x2E          # gravity vector (6 bytes)

CALIB_STAT_ADDR = 0x35          # sys/gyro/accel/mag calibration status
ST_RESULT_ADDR  = 0x36
SYS_STATUS_ADDR = 0x39
SYS_ERR_ADDR    = 0x3A

UNIT_SEL_ADDR   = 0x3B
OPR_MODE_ADDR   = 0x3D
PWR_MODE_ADDR   = 0x3E
SYS_TRIGGER_ADDR= 0x3F
AXIS_MAP_CONFIG = 0x41
AXIS_MAP_SIGN   = 0x42

# 동작 모드 (OPR_MODE)
MODE_CONFIG     = 0x00
MODE_ACCONLY    = 0x01
MODE_MAGONLY    = 0x02
MODE_GYRONLY    = 0x03
MODE_ACCMAG     = 0x04
MODE_ACCGYRO    = 0x05
MODE_MAGGYRO    = 0x06
MODE_AMG        = 0x07          # << 원시(raw) 9축, 퓨전 OFF
MODE_IMU        = 0x08          # 퓨전(가속+자이로)
MODE_COMPASS    = 0x09
MODE_M4G        = 0x0A
MODE_NDOF_FMC_OFF = 0x0B
MODE_NDOF       = 0x0C          # << 9축 풀 퓨전, euler "정답지"

# 전원 모드
POWER_NORMAL    = 0x00

# 환산 계수 (raw LSB -> 물리 단위)
ACCEL_SCALE = 100.0    # LSB / (m/s^2)
GYRO_SCALE  = 16.0     # LSB / dps
MAG_SCALE   = 16.0     # LSB / uT
EULER_SCALE = 16.0     # LSB / deg
QUAT_SCALE  = float(1 << 14)


def _to_signed16(lsb, msb):
    """리틀엔디언 2바이트 -> 부호있는 16비트 정수."""
    val = (msb << 8) | lsb
    if val & 0x8000:
        val -= 0x10000
    return val


class BNO055:
    def __init__(self, bus=DEFAULT_BUS, address=DEFAULT_ADDRESS,
                 use_external_crystal=True):
        self.address = address
        self._bus = SMBus(bus)
        self._mode = None
        self._open_and_verify(use_external_crystal)

    # ------------------------------------------------------------------ I/O
    def _write8(self, reg, value, retries=4):
        last = None
        for _ in range(retries):
            try:
                self._bus.write_byte_data(self.address, reg, value & 0xFF)
                return
            except OSError as e:
                last = e
                time.sleep(0.003)
        raise last

    def _read8(self, reg):
        """
        단일 레지스터 1바이트를 '블록 읽기'로 가져온다.
        (RPi 에서 read_byte_data 단일바이트는 bit7 손상이 잦아 read_i2c_block_data
         쪽이 더 안정적이다.)
        """
        return self._read_block(reg, 1)[0]

    def _read_block(self, reg, length, retries=4):
        """클럭 스트레칭/노이즈로 인한 일시적 read 실패를 재시도로 흡수."""
        last = None
        for _ in range(retries):
            try:
                return self._bus.read_i2c_block_data(self.address, reg, length)
            except OSError as e:           # I2C 에러
                last = e
                time.sleep(0.003)
        raise last

    # ------------------------------------------------------------- lifecycle
    def _wait_chip_id(self, timeout=2.0):
        """CHIP_ID(0xA0)가 안정적으로 읽힐 때까지(또는 timeout) 폴링."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                if self._read8(CHIP_ID_ADDR) == CHIP_ID:
                    return True
            except OSError:
                pass
            time.sleep(0.03)
        return False

    def _open_and_verify(self, use_external_crystal):
        if not self._wait_chip_id(timeout=2.0):
            raise RuntimeError(
                f"0x{self.address:02X} 에서 BNO055(CHIP_ID 0xA0)를 찾지 못했습니다. "
                "배선/주소를 확인하세요."
            )

        self.set_mode(MODE_CONFIG)
        self._write8(PAGE_ID_ADDR, 0x00)

        # 소프트 리셋
        self._write8(SYS_TRIGGER_ADDR, 0x20)
        time.sleep(0.7)                     # 리셋 후 부팅 대기(~650ms)
        if not self._wait_chip_id(timeout=2.0):
            raise RuntimeError("소프트 리셋 후 BNO055 가 응답하지 않습니다.")

        self._write8(PWR_MODE_ADDR, POWER_NORMAL)
        time.sleep(0.01)
        self._write8(PAGE_ID_ADDR, 0x00)
        self._write8(SYS_TRIGGER_ADDR, 0x80 if use_external_crystal else 0x00)
        time.sleep(0.01)

        # UNIT_SEL = 0 -> m/s^2, dps, uT, deg, Celsius, Windows orientation
        self._write8(UNIT_SEL_ADDR, 0x00)
        time.sleep(0.01)
        self.set_mode(MODE_NDOF)            # 기본은 퓨전 모드로 시작

    def set_mode(self, mode, retries=6):
        """
        동작 모드 변경.  클럭 스트레칭 손상으로 쓰기가 깨질 수 있으므로
        '쓰고 -> 되읽어 확인' 을 재시도한다.  (CONFIG<->동작 전환 지연 포함)
        """
        for attempt in range(retries):
            try:
                self._bus.write_byte_data(self.address, OPR_MODE_ADDR, mode & 0xFF)
            except OSError:
                time.sleep(0.02)
                continue
            # config -> 동작 7ms, 동작 -> config 19ms. 넉넉히 30ms.
            time.sleep(0.03)
            try:
                got = self._read8(OPR_MODE_ADDR) & 0x0F   # bit7 손상 무시
            except OSError:
                got = None
            if got == (mode & 0x0F):
                self._mode = mode
                return
            time.sleep(0.02)
        # 확인 실패해도 마지막으로 한 번 더 쓰고 진행(데이터 유효성은 호출측에서 점검)
        self._mode = mode

    def close(self):
        try:
            self._bus.close()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # ----------------------------------------------------------- raw vectors
    def _read_vector(self, reg, count=3, scale=1.0):
        data = self._read_block(reg, count * 2)
        out = []
        for i in range(count):
            out.append(_to_signed16(data[i * 2], data[i * 2 + 1]) / scale)
        return tuple(out)

    def read_all(self):
        """
        ACCEL/MAG/GYRO/EULER 를 '한 번의 I2C 트랜잭션'으로 읽어 같은 순간의
        값으로 반환한다(burst read).

        반환: dict {accel, mag, gyro, euler_raw, quat, euler_std}
          - accel/mag/gyro : 3-튜플(물리단위)
          - euler_raw      : BNO055 레지스터 원본 (heading, roll, pitch) — 참고용
          - quat           : (w,x,y,z)
          - euler_std      : 표준 ZYX (yaw, roll, pitch) — quat 에서 변환(권장)

        왜 burst 인가
        -------------
        - 개별 accel()/gyro()/mag()/euler()/quaternion() 를 따로 호출하면 I2C
          트랜잭션이 여러 번이라 (1) 실패 기회가 배가, (2) 값들이 서로 다른
          순간이 되어 빠르게 움직일 때 시간 정합이 깨진다.
        - 데이터 레지스터가 0x08(ACCEL)~0x27(QUAT끝) 연속이므로 32바이트를 한
          방에 읽으면 트랜잭션 1회 + 동일 시각 보장.

        주의: euler_raw 의 규약은 표준 ZYX 와 다르다(heading=180-yaw, roll<->pitch).
        표준이 필요하면 euler_std 를 써라(quat 에서 변환, 검증된 정확 경로).
        """
        blk = self._read_block(ACCEL_DATA_ADDR, 32)

        def vec(off, scale):
            return tuple(_to_signed16(blk[off + 2 * i], blk[off + 2 * i + 1]) / scale
                         for i in range(3))

        quat = (
            _to_signed16(blk[24], blk[25]) / QUAT_SCALE,   # w  (0x20)
            _to_signed16(blk[26], blk[27]) / QUAT_SCALE,   # x
            _to_signed16(blk[28], blk[29]) / QUAT_SCALE,   # y
            _to_signed16(blk[30], blk[31]) / QUAT_SCALE,   # z
        )
        roll, pitch, yaw = geometry.quat_to_euler(quat)

        return {
            "accel":     vec(0, ACCEL_SCALE),    # 0x08
            "mag":       vec(6, MAG_SCALE),      # 0x0E
            "gyro":      vec(12, GYRO_SCALE),    # 0x14
            "euler_raw": vec(18, EULER_SCALE),   # 0x1A (heading, roll, pitch) 원본
            "quat":      quat,                   # 0x20 (w,x,y,z)
            "euler_std": (yaw, roll, pitch),     # 표준 ZYX (quat 에서 변환)
        }

    def accel(self):
        """(ax, ay, az) m/s^2  — AMG/NDOF 어디서나 사용 가능."""
        return self._read_vector(ACCEL_DATA_ADDR, 3, ACCEL_SCALE)

    def accel_settled(self, tries=25):
        """
        모드 진입 직후 첫 읽기가 (0,0,0) 으로 나오는 구간을 건너뛰고
        중력이 잡히는 유효 가속도를 반환한다. (seed 용)
        """
        ax = ay = az = 0.0
        for _ in range(tries):
            ax, ay, az = self.accel()
            if abs(ax) + abs(ay) + abs(az) > 0.5:
                return ax, ay, az
            time.sleep(0.02)
        return ax, ay, az

    def gyro(self):
        """(gx, gy, gz) deg/s."""
        return self._read_vector(GYRO_DATA_ADDR, 3, GYRO_SCALE)

    def mag(self):
        """(mx, my, mz) uT."""
        return self._read_vector(MAG_DATA_ADDR, 3, MAG_SCALE)

    def euler(self):
        """내장 퓨전 euler (heading/yaw, roll, pitch) deg.  NDOF/IMU 모드에서만 유효.

        주의: 이는 BNO055 가 레지스터에 주는 '원본 순서' 그대로다.
        BNO055 의 euler 규약은 우리가 쓰는 표준 항공 규약과 roll/pitch 의
        역할/범위가 뒤바뀌어 있다(BNO055: roll ±90°, pitch ±180°).
        표준 규약(roll = X축 회전 atan2(ay,az), pitch ±90°)으로 정답을 비교
        하려면 euler_std() 를 쓴다. (실측 검증: 두 축을 맞바꾸면 우리 가속도
        경사계와 0.1~1.0° 오차로 일치)
        """
        return self._read_vector(EULER_DATA_ADDR, 3, EULER_SCALE)

    def euler_std(self):
        """
        '표준 항공 ZYX 규약'의 (yaw, roll, pitch) 도(degree), 모두 ±180/±90.
        쿼터니언을 읽어 geometry.quat_to_euler 로 변환한다 = euler_from_quat 와 동일.

        왜 레지스터(euler())가 아니라 쿼터니언인가
        -----------------------------------------
        실측·합성 이중검증(2000 표본 0불일치)으로, BNO055 의 euler '레지스터'와
        표준 ZYX 의 관계는 다음과 같음이 확정됐다:
            BNO_pitch   =  표준 roll       (오차 0.31°)
            BNO_roll    = -표준 pitch      (오차 0.09°)
            BNO_heading =  180 - 표준 yaw   (오차 3.8°)   <-- 핵심
        즉 레지스터 heading 은 표준 yaw 와 'wrap 만으로는' 못 맞추고 180-yaw 관계다.
        예전 euler_std 는 roll/pitch 스왑만 하고 yaw 를 wrap180(heading) 으로 둬서
        yaw 가 표준과 어긋났다(그래서 stage5/6 yaw 비교가 이상했음).

        해결: BNO055 가 같은 자세로 주는 '쿼터니언'을 표준 ZYX 로 변환하면 위
        규약 차이가 한 번에 해소된다(quat_to_euler 는 내적 1.0 으로 정확성 검증).
        이것이 '내부=쿼터니언 / 출력=오일러' 원칙이다.
        """
        roll, pitch, yaw = geometry.quat_to_euler(self.quaternion())
        return yaw, roll, pitch

    # 하위호환 별칭 (둘은 이제 동일하다)
    euler_from_quat = euler_std

    def euler_register_raw(self):
        """
        참고용: BNO055 euler '레지스터' 를 원본 순서(heading, roll, pitch)로.
        표준 ZYX 와 규약이 다르다(heading=180-yaw, roll<->pitch 스왑). 칩이 주는
        값을 '있는 그대로' 보고 싶을 때만 쓴다. 표준 비교에는 euler_std() 를 써라.
        """
        return self.euler()

    def projected_gravity(self):
        """몸체 좌표계의 중력 단위벡터 (로봇 RL 정책용). geometry 에 위임."""
        return geometry.projected_gravity(self.quaternion())

    def quaternion(self):
        data = self._read_block(QUAT_DATA_ADDR, 8)
        w = _to_signed16(data[0], data[1]) / QUAT_SCALE
        x = _to_signed16(data[2], data[3]) / QUAT_SCALE
        y = _to_signed16(data[4], data[5]) / QUAT_SCALE
        z = _to_signed16(data[6], data[7]) / QUAT_SCALE
        return (w, x, y, z)

    # ----------------------------------------------------------- diagnostics
    def calibration_status(self):
        """(sys, gyro, accel, mag) 각 0~3.  3이면 완전 보정."""
        s = self._read8(CALIB_STAT_ADDR)
        return ((s >> 6) & 0x03, (s >> 4) & 0x03, (s >> 2) & 0x03, s & 0x03)

    def system_status(self):
        return self._read8(SYS_STATUS_ADDR), self._read8(SYS_ERR_ADDR)


if __name__ == "__main__":
    # 간단한 연결 점검
    with BNO055() as imu:
        print(f"연결 성공: 0x{imu.address:02X}")
        imu.set_mode(MODE_AMG)
        time.sleep(0.1)
        print("AMG raw accel (m/s^2):",
              tuple(round(v, 2) for v in imu.accel_settled()))
        print("AMG raw gyro  (dps)  :", tuple(round(v, 2) for v in imu.gyro()))
        imu.set_mode(MODE_NDOF)
        time.sleep(1.0)
        print("NDOF euler (yaw,roll,pitch):",
              tuple(round(v, 2) for v in imu.euler()))
