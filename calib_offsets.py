#!/usr/bin/env python3
"""
BNO055 보정 offset 저장 / 복원
==============================
8자 보정으로 얻은 자력계/가속도/자이로 offset 을 파일(calib_offsets.json)에
저장하고, 다음 부팅 때 복원한다. BNO055 는 전원이 끊기면 보정이 사라지므로,
이 파일로 "매번 8자 흔들기"를 생략할 수 있다.

  python3 calib_offsets.py save     # 현재 보정 offset 을 파일로 저장
  python3 calib_offsets.py load     # 파일의 offset 을 센서에 복원
  python3 calib_offsets.py show     # 현재 센서 offset/보정상태 출력

★ 중요 — 자력계(mag) offset 은 '환경 의존'이다
  하드/소프트아이언 보정은 센서 주변 금속·모터·배터리의 자기왜곡을 학습한 값.
  따라서 저장한 offset 은 '그 센서가 그 로봇에 장착된 상태'에서만 유효하다.
    - 로봇(BDX)에 최종 장착한 뒤 save 해야 실제로 쓸 수 있다.
    - 센서만 따로 들고 보정한 값은 로봇에 붙이면 안 맞는다(모터 자기장 추가).
  gyro/accel offset 은 환경 무관이라 어디서 저장해도 된다.

저장 전 'save' 는 cal(mag,sys)이 3 인지 확인하고, 낮으면 경고한다.
"""

import sys
import json
import time

from bno055_driver import BNO055, MODE_NDOF

PATH = "calib_offsets.json"


def cmd_save():
    with BNO055() as imu:
        imu.set_mode(MODE_NDOF)
        time.sleep(1.0)
        sysq, g, a, m = imu.calibration_status()
        print(f"현재 보정도: sys={sysq} gyro={g} accel={a} mag={m}")
        if m < 3 or sysq < 2:
            print("⚠ mag/sys 보정이 낮습니다. 8자로 더 흔든 뒤 저장을 권합니다.")
            print("  (그래도 저장하려면 계속 진행됩니다)")
        offsets = imu.get_calibration_offsets()
        with open(PATH, "w") as f:
            json.dump({"offsets": offsets,
                       "cal_at_save": [sysq, g, a, m]}, f, indent=2)
        print(f"저장 완료 -> {PATH}  ({len(offsets)} bytes)")
        print("  주의: mag offset 은 '이 로봇 장착 상태' 에서만 유효합니다.")


def cmd_load():
    try:
        with open(PATH) as f:
            data = json.load(f)
    except FileNotFoundError:
        raise SystemExit(f"{PATH} 없음. 먼저 'python3 calib_offsets.py save' 하세요.")
    offsets = data["offsets"]
    with BNO055() as imu:
        imu.set_calibration_offsets(offsets)
        imu.set_mode(MODE_NDOF)
        time.sleep(1.0)
        sysq, g, a, m = imu.calibration_status()
        print(f"복원 완료. 복원 직후 보정도: sys={sysq} gyro={g} accel={a} mag={m}")
        print("  mag 가 아직 낮으면 센서를 살짝 8자로 흔들면 빠르게 3 으로 회복됩니다.")


def cmd_show():
    with BNO055() as imu:
        imu.set_mode(MODE_NDOF)
        time.sleep(0.5)
        sysq, g, a, m = imu.calibration_status()
        print(f"보정도: sys={sysq} gyro={g} accel={a} mag={m}")
        print("현재 센서 offset(22B):", imu.get_calibration_offsets())


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "show"
    if cmd == "save":
        cmd_save()
    elif cmd == "load":
        cmd_load()
    elif cmd == "show":
        cmd_show()
    else:
        raise SystemExit("사용법: python3 calib_offsets.py [save|load|show]")


if __name__ == "__main__":
    main()
