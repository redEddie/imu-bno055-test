#!/usr/bin/env python3
"""
보정 -> 실시간 확인 -> 저장  (한 프로세스에서)
=============================================
BNO055 드라이버는 생성(__init__)마다 소프트 리셋을 하므로, '보정'과 '저장'을
'다른 프로세스'로 나누면 저장 시점엔 보정이 이미 리셋되어 0 이 된다.
이 스크립트는 한 번 연결한 뒤, 보정 막대를 실시간으로 보여주다가 4축이 모두
3 이 되면(또는 사용자가 Enter) 그 자리에서 offset 을 파일로 저장한다.

동작
----
1) NDOF 모드로 연결(이때 리셋은 1회뿐).
2) 화면에 sys/gyro/accel/mag 막대를 실시간 표시.
   - gyro: 가만히  / accel: 6면 정지 / mag: 공중 8자 / sys: 자동
3) 4축 모두 3(sys/gyro/accel/mag = 3333)이 되면 자동 저장, 또는 Enter 로 수동 저장.
4) offset 22B 를 calib_offsets.json 에 기록.

사용법
------
  python3 calibrate_and_save.py            # 4축 3 되면 자동 저장
  python3 calibrate_and_save.py --manual   # 아무 때나 Enter 로 저장
종료: Ctrl-C
"""

import sys
import json
import time
import select

from bno055_driver import BNO055, MODE_NDOF

PATH = "calib_offsets.json"


def bar(v):
    return "[" + "#" * v + "." * (3 - v) + "]"


def enter_pressed():
    """논블로킹: Enter 입력 있으면 True. tty 아니면 항상 False."""
    if not sys.stdin.isatty():
        return False
    if select.select([sys.stdin], [], [], 0)[0]:
        sys.stdin.readline()
        return True
    return False


def main():
    auto = "--manual" not in sys.argv

    with BNO055() as imu:
        imu.set_mode(MODE_NDOF)
        time.sleep(1.0)
        print("[보정&저장] 4축 모두 [###] 목표.  "
              + ("4축 3되면 자동저장." if auto else "Enter 로 저장."))
        print("  gyro=가만히 / accel=6면 정지 / mag=공중 8자(★) / sys=자동")
        print("  Ctrl-C 종료\n")

        best_mag = 0
        saved = False
        try:
            while not saved:
                try:
                    sysq, g, a, m = imu.calibration_status()
                except OSError:
                    time.sleep(0.05); continue
                best_mag = max(best_mag, m)

                ready = (sysq == 3 and g == 3 and a == 3 and m == 3)   # 4축 모두 3 (3333)
                hint = "  <<< 준비됨!" + (" 저장 중..." if auto else " Enter 로 저장")
                print(f"  sys{bar(sysq)} gyro{bar(g)} accel{bar(a)} mag{bar(m)}"
                      f"  (mag최고 {best_mag}){hint if ready else '         '}",
                      end="\r", flush=True)

                do_save = (auto and ready) or (enter_pressed())
                if do_save:
                    offsets = imu.get_calibration_offsets()   # CONFIG 잠깐 전환
                    with open(PATH, "w") as f:
                        json.dump({"offsets": offsets,
                                   "cal_at_save": [sysq, g, a, m]}, f, indent=2)
                    print(f"\n\n저장 완료 -> {PATH}")
                    print(f"  보정도 sys={sysq} gyro={g} accel={a} mag={m}, offset {len(offsets)}B")
                    print("  주의: mag offset 은 '이 로봇 장착 상태' 에서만 유효.")
                    saved = True
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n중단(저장 안 함).")


if __name__ == "__main__":
    main()
