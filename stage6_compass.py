#!/usr/bin/env python3
"""
6단계: 나침반(heading) — BNO055 NDOF 가 정답
============================================
세 가지 heading 을 비교한다 (모두 ±180°, 가운데 '|' = 0°):
    RAW  : 틸트보상 없는 mag_heading (점 .)  - 기울이면 틀어진다
    TILT : 쿼터니언 기반 틸트보상     (별 *)  - 기울여도 NDOF 를 따라감
    NDOF : BNO055 내장 절대 yaw       (샵 #)  - 정답 (실용은 이걸 쓴다)

결론 (이 실습에서 얻은 것)
-------------------------
- '실용 heading' 은 BNO055 의 NDOF yaw(#)를 그대로 쓰는 게 정답이다. 칩이
  9축(가속도+자이로+자력계)을 융합하고 하드/소프트아이언 보정까지 내부에서
  하므로 가장 정확하다. 로봇 heading 제어도 이 값을 기준으로 삼으면 된다.
- 자력계로 직접 heading 을 구하는 건 (1) mag 축이 가속도 축과 1:1 로 안 맞고
  (2) mag 전용 캘리브레이션(하드/소프트아이언)을 따로 해야 해서 손이 많이 간다.
  단, '쿼터니언으로 mag 벡터를 월드로 회전' 하면 축 정렬 없이 틸트보상이 되어
  TILT(*) 가 NDOF 를 ~1.5° 로 추종한다(상수 offset 은 자북 기준 차이).
- 즉 "기울여도 방위가 맞는 나침반"의 원리는 확인했고, 정밀·실용은 NDOF 사용.

⚠ 자력계 보정 필수
------------------
cal 마지막 자리(mag)가 3 이어야 NDOF yaw 가 정확하다. calib_offsets.json 이
있으면 시작 시 자동 복원하고, 없으면 8자로 휘저어 보정하라.

실행:  python3 stage6_compass.py
종료:  Ctrl-C
"""

import json
import time

import geometry
import calibration
from geometry import wrap180
from bno055_driver import BNO055, MODE_NDOF

OFFSETS_PATH = "calib_offsets.json"


def load_offsets():
    """저장된 보정 offset(calib_offsets.json)을 읽어 리스트로. 없으면 None."""
    try:
        with open(OFFSETS_PATH) as f:
            return json.load(f)["offsets"]
    except (FileNotFoundError, ValueError, KeyError):
        return None

PLOT_WIDTH = 61
ANGLE_RANGE = 180.0


def _bar(values_with_marks):
    cells = [" "] * PLOT_WIDTH
    center = PLOT_WIDTH // 2
    cells[center] = "|"
    for angle, ch in values_with_marks:
        a = max(-ANGLE_RANGE, min(ANGLE_RANGE, angle))
        idx = int(round((a + ANGLE_RANGE) / (2 * ANGLE_RANGE) * (PLOT_WIDTH - 1)))
        cells[idx] = ch
    return "".join(cells)


def main():
    cal = calibration.load()        # MAG_SIGN_YAW 등 부호 보정 적용
    offsets = load_offsets()        # 저장된 자력계/가속도 보정 offset

    with BNO055() as imu:
        if offsets is not None:
            imu.set_calibration_offsets(offsets)
            print(f"[6단계] 보정 offset 복원됨 ({OFFSETS_PATH})", flush=True)
        else:
            print("[6단계] 저장된 보정 offset 없음 -> 8자로 직접 보정 필요", flush=True)
        imu.set_mode(MODE_NDOF)
        print("[6단계] NDOF 수렴 대기...", flush=True)
        time.sleep(2.0)

        print("  . = RAW(틸트보상X)   * = TILT(쿼터니언 틸트보상)   # = NDOF(정답)")
        print("  cal 마지막 자리(mag)가 3 인지 확인(아니면 살짝 8자로 흔들기).")
        print("  센서를 '기울인 채' 수평 회전 -> RAW(Δraw)는 크게 흔들리고")
        print("  TILT(Δtilt)는 NDOF 를 따라가면(작으면) 틸트보상 성공.")
        print("  ※ 절대정확도 한계(~수십°)는 mag 전용 보정을 직접 안 해서다(NDOF 는 칩이 함).")
        print("  Ctrl-C 로 종료\n")

        try:
            while True:
                try:
                    s = imu.read_all()       # accel/mag/gyro/quat/euler_std 동일 순간
                except OSError:
                    time.sleep(0.02); continue
                mx, my, mz = s["mag"]
                q = s["quat"]
                ax, ay, az = s["accel"]
                r_roll, r_pitch = geometry.accel_tilt(ax, ay, az)   # 표시용 기울기

                # RAW: 틸트보상 없는 단순 헤딩 (기울이면 틀어짐)
                h_raw  = cal.mag_yaw(geometry.mag_heading(mx, my))                    # .
                # TILT: 쿼터니언으로 mag 를 월드로 돌려 수평면 헤딩 (틸트보상 자동)
                h_tilt = cal.mag_yaw(geometry.quat_tilt_heading(q, mx, my, mz))       # *
                n_yaw = wrap180(s["euler_std"][0])                                    # #

                calstat = imu.calibration_status()
                line = _bar([(h_raw, "."), (h_tilt, "*"), (n_yaw, "#")])
                d_tilt = wrap180(h_tilt - n_yaw)   # 내 틸트보상이 정답과 얼마나 다른가
                d_raw  = wrap180(h_raw - n_yaw)
                print(f"[{line}]  "
                      f"RAW{h_raw:6.0f} TILT{h_tilt:6.0f} NDOF{n_yaw:6.0f}"
                      f"  Δtilt{d_tilt:+5.0f} Δraw{d_raw:+5.0f}"
                      f"  rp({r_roll:+4.0f},{r_pitch:+4.0f}) "
                      f"cal{calstat[0]}{calstat[1]}{calstat[2]}{calstat[3]}",
                      end="\r", flush=True)
                time.sleep(0.03)
        except KeyboardInterrupt:
            print("\n종료.")


if __name__ == "__main__":
    main()
