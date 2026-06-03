#!/usr/bin/env python3
"""
BNO055 보정 상태 실시간 모니터
==============================
sys/gyro/accel/mag 보정도(각 0~3)를 실시간 막대로 보여준다. 각 센서를
'어떻게' 움직여야 보정되는지 다르므로, 이 화면을 보며 맞는 동작을 익힌다.

  gyro  : 센서를 '가만히' 책상에 둔다           -> gyro 3
  accel : 6면(앞/뒤/좌/우/위/아래)을 잠깐씩 정지 -> accel 3
  mag   : 공중에서 '8자(∞)'로 손목 돌려 휘젓기   -> mag 3   (★ 평평히 두면 안 됨)
  sys   : 위 셋이 충분해지면 자동으로 3

목표: 네 칸 모두 [###] (=3) 가 되면 NDOF heading 을 신뢰할 수 있다.
종료: Ctrl-C
"""

import time
from bno055_driver import BNO055, MODE_NDOF


def bar(v):
    """0~3 -> [###...] 모양."""
    return "[" + "#" * v + "." * (3 - v) + "]"


def main():
    with BNO055() as imu:
        imu.set_mode(MODE_NDOF)
        print("[보정 모니터] NDOF.  네 칸 모두 [###] 이 목표.  Ctrl-C 종료")
        print("  gyro=가만히 / accel=6면 정지 / mag=공중 8자 / sys=자동\n")
        try:
            while True:
                try:
                    sysq, g, a, m = imu.calibration_status()
                except OSError:
                    time.sleep(0.05); continue
                done = "  <-- 완료! NDOF heading 신뢰 가능" if (m == 3 and sysq == 3) else ""
                print(f"  sys{bar(sysq)} gyro{bar(g)} accel{bar(a)} mag{bar(m)}{done}   ",
                      end="\r", flush=True)
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n종료.")


if __name__ == "__main__":
    main()
