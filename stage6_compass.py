#!/usr/bin/env python3
"""
6단계: 틸트 보상 나침반 (내 구현 B  vs  BNO055 NDOF 정답 A)
==========================================================
yaw(방위)를 자력계로 구할 때, 센서가 기울면 단순식 atan2(my,mx) 는 틀어진다.
가속도로 구한 roll/pitch 로 자기장 벡터를 수평면으로 되돌린 뒤 헤딩을
계산하는 '틸트 보상' 을 직접 구현하고, BNO055 가 칩 내부에서 9축 융합으로
주는 절대 yaw(정답) 와 같아지는지 비교한다.

세 헤딩을 한 화면에서 비교 (모두 ±180°, 가운데 '|' = 0°/자북 근처)
    RAW  : 틸트보상 없는 mag_heading   (점 .)  - 기울이면 틀어진다
    TILT : 내가 구현한 틸트보상 나침반   (별 *)  <- B
    NDOF : BNO055 내장 절대 yaw          (샵 #)  <- A, 정답

합격 기준
---------
센서를 '기울인 채로' 수평 회전시켜도 TILT(*) 가 NDOF(#) 를 잘 따라가고,
RAW(.) 만 혼자 틀어지면 성공. (수평일 때는 셋 다 비슷하다 — 차이는 기울일 때 난다)

⚠ 자력계 보정 필수
------------------
화면 cal 의 마지막 자리(mag, 0~3)가 3 이 되도록, 시작 전 센서를 공중에서
천천히 8자로 휘저어라. mag<3 이면 NDOF yaw 자체가 안 맞아 비교가 무의미하다.

실행:  python3 stage6_compass.py
종료:  Ctrl-C
"""

import time

import geometry
import calibration
from geometry import wrap180
from bno055_driver import BNO055, MODE_NDOF

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

    with BNO055() as imu:
        imu.set_mode(MODE_NDOF)
        print("[6단계] NDOF 수렴 대기...", flush=True)
        time.sleep(2.0)

        print("  . = RAW(틸트보상X)   * = TILT(내 틸트보상)   # = NDOF(정답)")
        print("  cal 마지막 자리(mag)가 3 이 되도록 8자로 흔들어 보정하세요.")
        print("  센서를 '기울인 채' 수평 회전 -> RAW 만 틀어지고 TILT 는 NDOF 를 따라가면 성공.")
        print("  Ctrl-C 로 종료\n")

        try:
            while True:
                ax, ay, az = imu.accel()
                mx, my, mz = imu.mag()
                # 틸트보상 입력용 '순수 기하' roll/pitch (부호보정 전)
                r_roll, r_pitch = geometry.accel_tilt(ax, ay, az)

                h_raw  = cal.mag_yaw(geometry.mag_heading(mx, my))                    # .
                h_tilt = cal.mag_yaw(
                    geometry.tilt_compensated_heading(mx, my, mz, r_roll, r_pitch))   # *
                n_yaw, _, _ = imu.euler_std()                                         # #
                n_yaw = wrap180(n_yaw)

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
