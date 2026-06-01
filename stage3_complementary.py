#!/usr/bin/env python3
"""
3단계: 상보 필터(Complementary Filter)
======================================
1단계(가속도: 고주파 충격에 취약)와 2단계(자이로: 저주파 드리프트에 취약)를
수학적으로 결합한다.

    angle = α·(angle_prev + gyro·dt) + (1-α)·accel_angle

실습 포인트
-----------
- 센서를 마구 흔들어도 요동치지 않고(자이로의 반응성),
  가만히 둬도 흐르지 않는다(가속도의 안정성).
- 화면에 3가지를 함께 띄워 비교한다:
    ACC  : 가속도만 (거칠다)
    GYRO : 자이로 적분만 (흐른다)
    COMP : 상보 필터 (부드럽고 안정적)
- α 값을 인자로 바꿔가며 '튜닝' 해본다:
    python3 stage3_complementary.py 0.98
    python3 stage3_complementary.py 0.90   # 가속도 비중↑ -> 더 흔들림
    python3 stage3_complementary.py 0.995  # 자이로 비중↑ -> 더 부드럽지만 드리프트 살짝

실행:  python3 stage3_complementary.py [alpha]
종료:  Ctrl-C
"""

import sys
import time

from bno055_driver import BNO055, MODE_AMG
import filters
from filters import accel_tilt, GyroIntegrator, ComplementaryFilter


def main():
    alpha = float(sys.argv[1]) if len(sys.argv) > 1 else 0.98
    if not (0.0 < alpha < 1.0):
        raise SystemExit("alpha 는 0~1 사이여야 합니다 (예: 0.98)")

    filters.load_calibration()      # 부호 보정 적용 (없으면 +1 기본 + 안내)

    with BNO055() as imu:
        imu.set_mode(MODE_AMG)
        time.sleep(0.05)

        # 시작값을 가속도 각도로 맞춰 수렴을 빠르게 (첫 읽기 0,0,0 구간은 건너뜀)
        ax, ay, az = imu.accel_settled()
        comp = ComplementaryFilter(alpha=alpha)
        comp.seed(ax, ay, az)
        integ = GyroIntegrator(comp.roll, comp.pitch)

        print(f"[3단계] 상보 필터  alpha={alpha}  (AMG raw 모드).  Ctrl-C 로 종료\n")
        print(f"{'':>6} | {'ACC':>17} | {'GYRO':>17} | {'COMP':>17}")
        print(f"{'':>6} | {'roll':>8}{'pitch':>9} | "
              f"{'roll':>8}{'pitch':>9} | {'roll':>8}{'pitch':>9}")
        print("-" * 70)

        t_prev = time.perf_counter()
        try:
            while True:
                now = time.perf_counter()
                dt = now - t_prev
                t_prev = now

                ax, ay, az = imu.accel()
                gx, gy, gz = imu.gyro()

                a_roll, a_pitch = accel_tilt(ax, ay, az)
                g_roll, g_pitch = integ.update(gx, gy, dt)
                c_roll, c_pitch = comp.update(ax, ay, az, gx, gy, dt)

                print(f"{'angle':>6} | {a_roll:8.1f}{a_pitch:9.1f} | "
                      f"{g_roll:8.1f}{g_pitch:9.1f} | "
                      f"{c_roll:8.1f}{c_pitch:9.1f}", end="\r", flush=True)
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("\n종료.")


if __name__ == "__main__":
    main()
