#!/usr/bin/env python3
"""
1단계: 가속도계 기반 '경사계(Tiltmeter)'
=========================================
자이로를 전혀 쓰지 않고, AMG(raw) 모드의 가속도 원시값 + 삼각함수만으로
roll/pitch 를 계산한다.

    roll  = atan2(ay, az)
    pitch = atan2(-ax, sqrt(ay^2 + az^2))

실습 포인트
-----------
- 센서를 천천히 기울이면 각도가 정확하게 따라온다.
- 센서를 "툭툭" 치거나 팍 움직이면(충격/선형가속 노이즈) 각도가 무섭게 요동친다.
  -> 가속도만으로는 안 된다는 걸 눈으로 확인한다.

실행:  python3 stage1_tiltmeter.py
종료:  Ctrl-C
"""

import time

from bno055_driver import BNO055, MODE_AMG
import geometry
import calibration


def main():
    cal = calibration.load()        # 축 부호 정렬 (없으면 +1 기본)
    with BNO055() as imu:
        imu.set_mode(MODE_AMG)          # 퓨전 OFF, 순수 raw
        time.sleep(0.05)
        print("[1단계] 가속도 경사계 (AMG raw 모드).  Ctrl-C 로 종료\n")
        print(f"{'ax':>7} {'ay':>7} {'az':>7} | {'roll(°)':>9} {'pitch(°)':>9}")
        print("-" * 46)
        try:
            while True:
                ax, ay, az = imu.accel()
                roll, pitch = cal.acc_tilt(*geometry.accel_tilt(ax, ay, az))
                print(f"{ax:7.2f} {ay:7.2f} {az:7.2f} | "
                      f"{roll:9.2f} {pitch:9.2f}", end="\r", flush=True)
                time.sleep(0.02)        # ~50 Hz
        except KeyboardInterrupt:
            print("\n종료.")


if __name__ == "__main__":
    main()
