"""
filters.py
==========
자세 추정 '필터(상태를 가진 연산)' 계층.  순수 수학은 geometry.py 에,
축 부호 정렬은 calibration.py 에 위임하고, 여기엔 상태를 들고 시간에 따라
갱신되는 필터 클래스만 둔다.

  geometry.py    : wrap180, quat 연산, accel_tilt, mag_heading, tilt_comp ...
  calibration.py : 6개 부호(Calibration). geometry 순수값 -> 정렬된 값
  filters.py     : GyroIntegrator / ComplementaryFilter / YawComplementaryFilter

각 필터는 생성 시 Calibration 을 받는다(없으면 기본 +1). 부호를 모듈 전역으로
두지 않으므로, 여러 보정을 동시에 다뤄도 안전하고 테스트가 쉽다.

좌표 약속 (센서 평평할 때 az≈+9.8 기준; 실제 부호는 calibration 이 맞춤)
  roll  : X축 회전  -> gyro gx
  pitch : Y축 회전  -> gyro gy
  yaw   : Z축 회전  -> gyro gz  (저주파 기준은 가속도가 아니라 자력계)
"""

import geometry
from geometry import wrap180          # 하위호환: filters.wrap180 으로도 접근 가능
from calibration import Calibration


# ---------------------------------------------------------------------------
# 2단계: 자이로 적분 (Dead Reckoning)
# ---------------------------------------------------------------------------
class GyroIntegrator:
    """
    각속도(dps)를 적분해 각도 추정.  angle += rate*dt.
    반응성은 즉각이지만 바이어스가 누적돼 드리프트한다.
    """

    def __init__(self, roll0=0.0, pitch0=0.0, calib=None):
        self.roll = roll0
        self.pitch = pitch0
        self.cal = calib or Calibration()

    def update(self, gx, gy, dt):
        r_rate, p_rate, _ = self.cal.gyro_rates(gx, gy, 0.0)
        self.roll += r_rate * dt
        self.pitch += p_rate * dt
        return self.roll, self.pitch


# ---------------------------------------------------------------------------
# 3단계: 상보 필터 (roll/pitch)
# ---------------------------------------------------------------------------
class ComplementaryFilter:
    """
    angle = α·(angle_prev + gyro·dt) + (1-α)·accel_angle
    α(0.95~0.99): 자이로(반응성) 비중,  (1-α): 가속도(안정성) 비중.
    """

    def __init__(self, alpha=0.98, roll0=0.0, pitch0=0.0, calib=None):
        self.alpha = alpha
        self.roll = roll0
        self.pitch = pitch0
        self.cal = calib or Calibration()

    def update(self, ax, ay, az, gx, gy, dt):
        acc_roll, acc_pitch = self.cal.acc_tilt(*geometry.accel_tilt(ax, ay, az))
        r_rate, p_rate, _ = self.cal.gyro_rates(gx, gy, 0.0)
        a = self.alpha

        # roll 은 ±180 을 넘나들 수 있으므로(특히 칩이 뒤집힌 마운팅) 자이로
        # 예측값에 가장 가까운 등가각으로 가속도 roll 을 풀어 섞는다(경계 튐 방지).
        pred_roll = self.roll + r_rate * dt
        acc_roll = pred_roll + wrap180(acc_roll - pred_roll)
        self.roll = wrap180(a * pred_roll + (1 - a) * acc_roll)
        # pitch 는 ±90 범위라 wrap 불필요.
        self.pitch = a * (self.pitch + p_rate * dt) + (1 - a) * acc_pitch
        return self.roll, self.pitch

    def seed(self, ax, ay, az):
        """가속도 각도로 초기화(빠른 수렴)."""
        self.roll, self.pitch = self.cal.acc_tilt(*geometry.accel_tilt(ax, ay, az))


# ---------------------------------------------------------------------------
# yaw 전용 상보 필터 (자이로 gz + 자력계 헤딩)
# ---------------------------------------------------------------------------
class YawComplementaryFilter:
    """
    yaw = α·(yaw_prev + gz·dt) + (1-α)·mag_heading
    저주파 기준이 가속도가 아니라 자력계라는 점, yaw 가 ±180 을 순환한다는 점만
    roll/pitch 상보필터와 다르다.
    """

    def __init__(self, alpha=0.98, yaw0=0.0, calib=None):
        self.alpha = alpha
        self.yaw = yaw0
        self.cal = calib or Calibration()

    def update(self, mx, my, gz, dt):
        a = self.alpha
        _, _, y_rate = self.cal.gyro_rates(0.0, 0.0, gz)
        predicted = self.yaw + y_rate * dt
        mag = self.cal.mag_yaw(geometry.mag_heading(mx, my))
        # mag 를 predicted 에 가장 가까운 등가각으로 풀어 ±180 경계 튐 방지.
        mag_unwrapped = predicted + wrap180(mag - predicted)
        self.yaw = wrap180(a * predicted + (1 - a) * mag_unwrapped)
        return self.yaw

    def seed(self, mx, my):
        self.yaw = self.cal.mag_yaw(geometry.mag_heading(mx, my))
