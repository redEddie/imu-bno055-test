"""
calibration.py
==============
'축 부호 정렬' 계층.  geometry.py 의 순수 기하각(부호 없음)을 BNO055 내장
퓨전(NDOF, euler_std)의 좌표 규약에 맞추는 6개 부호를 관리한다.

왜 출력이 아니라 '센서 채널' 부호인가
-------------------------------------
출력 각도에 -1 을 곱하는 방식은 yaw 처럼 자이로와 자력계의 부호가 서로 다른
경우(자이로는 맞고 mag 만 반대)를 표현할 수 없다. 그래서 부호를 입력 채널
단위로 둔다:
    acc.roll, acc.pitch   : 가속도로 푼 roll/pitch 부호
    mag.yaw               : 자력계로 푼 yaw 부호
    gyro.roll/pitch/yaw   : 자이로 rate 적분 방향 부호

이 6개는 calibrate_signs.py 가 NDOF 기준으로 1회 측정해 calib_signs.json 에
저장한다.  여기서는 load/save 와, 부호를 적용하는 얇은 헬퍼만 제공한다.

설계 메모
---------
예전엔 이 부호들이 filters.py 의 모듈 전역이었고 accel_tilt 등이 직접 곱했다.
그 탓에 'tilt_compensated_heading 은 부호 미보정 roll/pitch 가 필요한데
accel_tilt 는 부호가 박혀 나온다' 는 모순이 생겼다. 이제 geometry 는 순수
기하만 계산하고(부호 0), 부호 정렬은 이 Calibration 객체가 명시적으로 적용한다.
"""

import json

# 6개 부호 키 (json <-> 내부 dict)
SIGN_KEYS = (
    "ACC_SIGN_ROLL", "ACC_SIGN_PITCH", "MAG_SIGN_YAW",
    "GYRO_SIGN_ROLL", "GYRO_SIGN_PITCH", "GYRO_SIGN_YAW",
)

DEFAULT_PATH = "calib_signs.json"


class Calibration:
    """6개 부호(+1/-1)를 들고, geometry 출력에 부호를 적용하는 작은 상태 객체."""

    def __init__(self, signs=None):
        # 기본값: 모두 +1 (보정 안 됨)
        self.signs = {k: +1.0 for k in SIGN_KEYS}
        if signs:
            self.update(signs)

    # --- 부호 갱신 ---
    def update(self, signs):
        for k in SIGN_KEYS:
            if k in signs:
                self.signs[k] = +1.0 if float(signs[k]) >= 0 else -1.0
        return self

    def get(self, key):
        return self.signs[key]

    # --- 부호 적용 헬퍼 (geometry 순수값 -> 정렬된 값) ---
    def acc_tilt(self, roll, pitch):
        """geometry.accel_tilt 결과에 acc 부호 적용."""
        return (self.signs["ACC_SIGN_ROLL"] * roll,
                self.signs["ACC_SIGN_PITCH"] * pitch)

    def mag_yaw(self, heading):
        """mag 헤딩에 mag.yaw 부호 적용."""
        return self.signs["MAG_SIGN_YAW"] * heading

    def gyro_rates(self, gx, gy, gz):
        """자이로 (gx,gy,gz) 에 축별 부호 적용 -> (roll_rate, pitch_rate, yaw_rate)."""
        return (self.signs["GYRO_SIGN_ROLL"] * gx,
                self.signs["GYRO_SIGN_PITCH"] * gy,
                self.signs["GYRO_SIGN_YAW"] * gz)

    # --- 영속화 ---
    def save(self, path=DEFAULT_PATH):
        out = {k: (1 if self.signs[k] >= 0 else -1) for k in SIGN_KEYS}
        with open(path, "w") as f:
            json.dump(out, f, indent=2)
        return out

    def label(self):
        """사람이 읽는 한 줄 요약. 예: acc.roll=+ acc.pitch=- mag.yaw=+ ..."""
        def lab(k):
            src, _, axis = k.lower().partition("_sign_")
            return f"{src}.{axis}"
        return " ".join(f"{lab(k)}={'+' if self.signs[k] > 0 else '-'}"
                        for k in SIGN_KEYS)


def load(path=DEFAULT_PATH, quiet=False):
    """
    calib_signs.json -> Calibration 객체.
    파일이 없으면 기본값(+1) 객체를 돌려준다(에러 아님).
    """
    try:
        with open(path) as f:
            data = json.load(f)
    except (FileNotFoundError, ValueError):
        if not quiet:
            print(f"[calib] {path} 없음 -> 부호 기본값(+1). calibrate_signs.py 로 보정 권장.")
        return Calibration()
    cal = Calibration(data)
    if not quiet:
        print(f"[calib] {path} 적용: {cal.label()}")
    return cal
