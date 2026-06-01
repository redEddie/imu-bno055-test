"""
filters.py
==========
BNO055 원시(AMG) 데이터로 자세(roll/pitch)를 추정하는 알고리즘 모음.
표준 라이브러리(math)만 사용 -> 라즈베리파이 zero 2w 에서도 가볍다.

좌표/부호 약속 (BNO055 기본 Windows orientation, 센서를 평평히 놓았을 때 az≈+9.8)
  roll  (phi,   φ) : X축 둘레 회전  -> gyro gx 가 roll rate
  pitch (theta, θ) : Y축 둘레 회전  -> gyro gy 가 pitch rate
  yaw   (psi,   ψ) : Z축 둘레 회전  -> gyro gz 가 yaw rate

가속도(중력)는 yaw 를 못 잡는다(수직축을 돌려도 중력 벡터가 안 변하므로).
그래서 yaw 의 '저주파 기준'은 가속도가 아니라 '자력계(나침반)' 가 맡는다.
이것이 roll/pitch 와 yaw 의 본질적 차이이자 9축 융합이 필요한 이유다.

좌표계 정렬 — 6개 부호 상수
--------------------------
내 표준 공식(accel_tilt/mag_heading/자이로 적분)의 축 부호와 BNO055 내장 퓨전
(euler_std)의 부호가 어긋날 수 있다(센서 마운팅/규약 차이). 이를 '출력 각도에
-1 곱하기' 로 고치면 안 된다: yaw 처럼 자이로와 자력계의 부호가 서로 다른 경우
(자이로는 맞고 mag 만 반대) 출력 한 번 뒤집기로는 표현이 불가능하기 때문이다.

그래서 부호를 '센서 채널(입력)' 단위로 둔다:
  ACC_SIGN_*  : 가속도로 푼 기준 각도(roll/pitch)의 부호
  MAG_SIGN_YAW: 자력계로 푼 yaw 기준 각도의 부호
  GYRO_SIGN_* : 자이로 rate 적분 방향의 부호

이 6개 값은 calibrate_signs.py 가 NDOF 를 기준으로 1회 측정해 calib_signs.json
에 저장하고, load_calibration() 으로 불러온다. (기본값은 모두 +1)
"""

import json
import math

# --- 가속도/자력계(저주파 기준) 채널 부호 ---
ACC_SIGN_ROLL   = +1.0
ACC_SIGN_PITCH  = +1.0
MAG_SIGN_YAW    = +1.0

# --- 자이로(고주파 적분) 채널 부호 ---
GYRO_SIGN_ROLL  = +1.0
GYRO_SIGN_PITCH = +1.0
GYRO_SIGN_YAW   = +1.0

# load_calibration 이 다루는 키 목록 (json <-> 모듈 전역 매핑)
_SIGN_KEYS = (
    "ACC_SIGN_ROLL", "ACC_SIGN_PITCH", "MAG_SIGN_YAW",
    "GYRO_SIGN_ROLL", "GYRO_SIGN_PITCH", "GYRO_SIGN_YAW",
)

CALIB_PATH = "calib_signs.json"


def load_calibration(path=CALIB_PATH, quiet=False):
    """
    calib_signs.json 에서 6개 부호를 읽어 모듈 전역에 반영한다.
    파일이 없으면 기본값(+1)을 유지하고 None 을 반환한다(에러 아님).
    필터 클래스들은 호출 시점에 이 전역을 읽으므로, 루프 시작 전에 한 번
    불러두면 그대로 적용된다.
    """
    g = globals()
    try:
        with open(path) as f:
            data = json.load(f)
    except (FileNotFoundError, ValueError):
        if not quiet:
            print(f"[calib] {path} 없음 -> 부호 기본값(+1) 사용. "
                  "calibrate_signs.py 로 보정 권장.")
        return None
    applied = {}
    for k in _SIGN_KEYS:
        if k in data:
            g[k] = +1.0 if float(data[k]) >= 0 else -1.0
            applied[k] = g[k]
    if not quiet:
        print(f"[calib] {path} 적용: "
              + " ".join(f"{k.split('_',1)[1].lower()}={'+' if v>0 else '-'}"
                         for k, v in applied.items()))
    return applied


def save_calibration(signs, path=CALIB_PATH):
    """6개 부호 dict 를 calib_signs.json 으로 저장."""
    out = {k: (1 if signs.get(k, 1) >= 0 else -1) for k in _SIGN_KEYS}
    with open(path, "w") as f:
        json.dump(out, f, indent=2)
    return out


def wrap180(a):
    """각도를 [-180, 180) 범위로 접는다. yaw 처럼 360° 순환하는 양에 필수.
    (±180 은 같은 각도이며 이 식은 그 경계를 -180 으로 돌려준다.)"""
    return (a + 180.0) % 360.0 - 180.0


# ---------------------------------------------------------------------------
# 1단계: 가속도 기반 경사계 (Tiltmeter)
# ---------------------------------------------------------------------------
def accel_tilt(ax, ay, az):
    """
    가속도 원시값만으로 (roll, pitch) 를 도(deg) 단위로 계산.

        roll  = atan2(ay, az)
        pitch = atan2(-ax, sqrt(ay^2 + az^2))

    정지/저주파에선 정확하지만, 충격/선형가속(고주파) 노이즈에 매우 취약하다.
    ACC_SIGN_* 로 NDOF 규약과 부호를 맞춘다(calibrate_signs.py 가 결정).
    """
    roll  = ACC_SIGN_ROLL  * math.degrees(math.atan2(ay, az))
    pitch = ACC_SIGN_PITCH * math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
    return roll, pitch


def mag_heading(mx, my):
    """
    자력계로 yaw(헤딩)를 계산. (가속도가 못 잡는 yaw 의 저주파 기준)

        heading = atan2(my, mx)

    주의: 이건 '센서가 수평일 때'만 맞는 단순식이다(tilt 보상 안 함).
    실습에선 X/Y 축은 가만히 두고 Z(yaw)만 돌려가며 보면 충분히 의미 있다.
    가속도가 roll/pitch 의 기준이듯, 자력계가 yaw 의 기준임을 체감하는 게 목적.
    MAG_SIGN_YAW 로 NDOF/자이로 규약과 부호를 맞춘다.
    """
    return MAG_SIGN_YAW * math.degrees(math.atan2(my, mx))


# ---------------------------------------------------------------------------
# 2단계: 자이로 적분 (Dead Reckoning)
# ---------------------------------------------------------------------------
class GyroIntegrator:
    """
    각속도(dps)를 시간에 대해 누적 적분해 각도를 추정.
        angle += rate * dt
    반응성은 즉각적이지만, 미세 바이어스가 쌓여 드리프트가 발생한다.
    """

    def __init__(self, roll0=0.0, pitch0=0.0):
        self.roll = roll0
        self.pitch = pitch0

    def update(self, gx, gy, dt):
        self.roll  += GYRO_SIGN_ROLL  * gx * dt
        self.pitch += GYRO_SIGN_PITCH * gy * dt
        return self.roll, self.pitch


# ---------------------------------------------------------------------------
# 3단계: 상보 필터 (Complementary Filter)
# ---------------------------------------------------------------------------
class ComplementaryFilter:
    """
    angle = α·(angle_prev + gyro·dt) + (1-α)·accel_angle

    α(0.95~0.99): 자이로(고주파, 반응성) 비중.
    (1-α)        : 가속도(저주파, 안정성/드리프트 보정) 비중.
    """

    def __init__(self, alpha=0.98, roll0=0.0, pitch0=0.0):
        self.alpha = alpha
        self.roll = roll0
        self.pitch = pitch0

    def update(self, ax, ay, az, gx, gy, dt):
        acc_roll, acc_pitch = accel_tilt(ax, ay, az)
        a = self.alpha
        # roll 은 ±180 을 넘나들 수 있다(특히 칩이 뒤집힌 마운팅). 자이로 예측값에
        # 가장 가까운 등가각으로 가속도 roll 을 풀어준 뒤 섞어 경계 튐을 막는다.
        pred_roll = self.roll + GYRO_SIGN_ROLL * gx * dt
        acc_roll = pred_roll + wrap180(acc_roll - pred_roll)
        self.roll = wrap180(a * pred_roll + (1 - a) * acc_roll)
        # pitch 는 ±90 범위라 wrap 불필요.
        self.pitch = a * (self.pitch + GYRO_SIGN_PITCH * gy * dt) + (1 - a) * acc_pitch
        return self.roll, self.pitch

    def seed(self, ax, ay, az):
        """시작 시 가속도 각도로 초기화하면 수렴이 빠르다."""
        self.roll, self.pitch = accel_tilt(ax, ay, az)


# ---------------------------------------------------------------------------
# yaw 전용 상보 필터 (자이로 gz + 자력계 헤딩)
# ---------------------------------------------------------------------------
class YawComplementaryFilter:
    """
    yaw = α·(yaw_prev + gz·dt) + (1-α)·mag_heading

    roll/pitch 의 상보필터와 구조는 같지만 두 가지가 다르다:
      - 저주파 기준이 '가속도' 가 아니라 '자력계(나침반)' 다.
      - yaw 는 ±180° 를 넘나들며 순환하므로, 자이로 예측값에 가까운 쪽으로
        자력계 각도를 풀어준 뒤(wrap) 섞고, 결과도 wrap180 한다.
    """

    def __init__(self, alpha=0.98, yaw0=0.0):
        self.alpha = alpha
        self.yaw = yaw0

    def update(self, mx, my, gz, dt):
        a = self.alpha
        predicted = self.yaw + GYRO_SIGN_YAW * gz * dt
        mag = mag_heading(mx, my)
        # mag 를 predicted 에 가장 가까운 등가각으로 풀어준다(±180 경계 튐 방지).
        mag_unwrapped = predicted + wrap180(mag - predicted)
        self.yaw = wrap180(a * predicted + (1 - a) * mag_unwrapped)
        return self.yaw

    def seed(self, mx, my):
        self.yaw = mag_heading(mx, my)
