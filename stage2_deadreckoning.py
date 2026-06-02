#!/usr/bin/env python3
"""
2단계: 자이로스코프 기반 '데드 레코닝(Dead Reckoning)'
=====================================================
가속도를 전혀 쓰지 않고, AMG(raw) 모드의 각속도(dps)만 시간 적분한다.

    angle += angular_velocity * dt

실습 포인트
-----------
- 빠르게 회전시켜도 지연 없이 즉각 반응한다(반응성 최고).
- 그런데 센서를 책상 위에 "완전히 가만히" 둬도, 미세 바이어스가 누적되어
  각도가 혼자 1°, 5°, 10°... 끊임없이 흘러간다(드리프트).
  -> 화면의 'drift' 열이 0 에서 점점 멀어지는 걸 관찰한다.

실행:  python3 stage2_deadreckoning.py
종료:  Ctrl-C
"""

import time

from bno055_driver import BNO055, MODE_AMG
import calibration
from filters import GyroIntegrator


def main():
    with BNO055() as imu:
        imu.set_mode(MODE_AMG)
        time.sleep(0.05)

        integ = GyroIntegrator(calib=calibration.load())   # 0,0 에서 시작
        print("[2단계] 자이로 적분 (AMG raw 모드).  센서를 가만히 두고 드리프트를 보세요.")
        print("        Ctrl-C 로 종료\n")
        print(f"{'gx':>7} {'gy':>7} | {'roll(°)':>9} {'pitch(°)':>9} | {'경과(s)':>7}")
        print("-" * 50)

        t0 = time.perf_counter()
        t_prev = t0
        try:
            while True:
                now = time.perf_counter()
                dt = now - t_prev
                t_prev = now

                gx, gy, gz = imu.gyro()
                roll, pitch = integ.update(gx, gy, dt)

                print(f"{gx:7.2f} {gy:7.2f} | "
                      f"{roll:9.2f} {pitch:9.2f} | {now - t0:7.1f}",
                      end="\r", flush=True)
                time.sleep(0.01)        # ~100 Hz (적분은 빠를수록 좋다)
        except KeyboardInterrupt:
            print(f"\n종료.  누적 드리프트 -> roll {integ.roll:+.2f}°, "
                  f"pitch {integ.pitch:+.2f}°  (정지 상태였다면 이 값이 곧 오차)")


if __name__ == "__main__":
    main()
