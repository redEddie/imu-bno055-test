#!/usr/bin/env python3
"""
5단계: X / Y / Z 축 전환 실시간 비교
====================================
stage4 의 한 축(roll) 비교를 세 축으로 확장한다. 한 화면에서 키로 축을
바꿔가며, 내 필터가 BNO055 정답을 따라가는지 본다.

    X축(roll)  : 기준 = ACC(가속도)   vs COMP(상보필터) vs NDOF(정답)
    Y축(pitch) : 기준 = ACC(가속도)   vs COMP(상보필터) vs NDOF(정답)
    Z축(yaw)   : 기준 = MAG(자력계)   vs COMP(상보필터) vs NDOF(정답)

왜 Z만 기준이 다른가?
---------------------
가속도(중력)는 yaw 를 못 잡는다. 수직축(Z)을 돌려도 중력 벡터가 안 변하기
때문이다. 그래서 yaw 의 저주파 기준은 가속도가 아니라 '자력계(나침반)' 가
맡는다. roll/pitch 는 가속도가, yaw 는 자력계가 자이로의 드리프트를 잡아준다
— 이것이 9축 융합(NDOF)이 필요한 이유다.

조작 (실시간을 해치지 않는 논블로킹 키 입력)
-------------------------------------------
    x / y / z : 해당 축으로 즉시 전환
    space/엔터 : X -> Y -> Z 순환
    + / -      : alpha 실시간 증감 (튜닝)
    q 또는 Ctrl-C : 종료
(파이프/리다이렉트 등 tty 가 아니면 키 입력 없이 X 축으로 그냥 돈다.)

세 축 필터는 매 루프 '동시에' 갱신되므로, 축을 바꿔도 재수렴 없이 즉시
전환된다(실시간 유지).

실행:  python3 stage5_axes.py [alpha] [시작축 x|y|z]
"""

import sys
import time
import select

from bno055_driver import BNO055, MODE_NDOF
import geometry
import calibration
from geometry import wrap180
from filters import ComplementaryFilter, YawComplementaryFilter

PLOT_WIDTH = 61          # 막대 칸 수 (홀수 -> 가운데가 0°)
ANGLE_RANGE = 180.0      # 모든 축 공통: 양 끝 ±180° (clip 없음)

AXES = ("x", "y", "z")
AXIS_TITLE = {
    "x": "X축 / roll  (기준=ACC 가속도)",
    "y": "Y축 / pitch (기준=ACC 가속도)",
    "z": "Z축 / yaw   (기준=MAG 자력계)",
}
REF_LABEL = {"x": "ACC", "y": "ACC", "z": "MAG"}


def _bar(values_with_marks):
    cells = [" "] * PLOT_WIDTH
    center = PLOT_WIDTH // 2
    cells[center] = "|"
    for angle, ch in values_with_marks:
        a = max(-ANGLE_RANGE, min(ANGLE_RANGE, angle))
        idx = int(round((a + ANGLE_RANGE) / (2 * ANGLE_RANGE) * (PLOT_WIDTH - 1)))
        cells[idx] = ch
    return "".join(cells)


class KeyReader:
    """tty 면 cbreak 모드로 논블로킹 1글자 읽기, 아니면 아무것도 안 함."""

    def __init__(self):
        self.enabled = sys.stdin.isatty()
        self._fd = None
        self._saved = None

    def __enter__(self):
        if self.enabled:
            import termios, tty
            self._fd = sys.stdin.fileno()
            self._saved = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
        return self

    def __exit__(self, *exc):
        if self.enabled and self._saved is not None:
            import termios
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved)

    def get(self):
        """대기 중인 키 1글자 반환, 없으면 None (블로킹 안 함)."""
        if not self.enabled:
            return None
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None


def main():
    alpha = float(sys.argv[1]) if len(sys.argv) > 1 else 0.98
    axis = sys.argv[2].lower() if len(sys.argv) > 2 and sys.argv[2].lower() in AXES else "x"

    cal = calibration.load()        # 부호 보정 적용 (없으면 +1 기본 + 안내)

    with BNO055() as imu:
        imu.set_mode(MODE_NDOF)
        print("[5단계] NDOF 퓨전 엔진 수렴 대기 중...", flush=True)
        time.sleep(2.0)

        ax, ay, az = imu.accel_settled()
        mx, my, mz = imu.mag()
        comp = ComplementaryFilter(alpha=alpha, calib=cal)
        comp.seed(ax, ay, az)
        yaw_f = YawComplementaryFilter(alpha=alpha, calib=cal)
        yaw_f.seed(mx, my)

        print("  막대 범위: -180° (왼쪽) ~ +180° (오른쪽),  가운데 '|' = 0°")
        print("  . = 기준(ACC/MAG)   * = COMP(내 상보필터)   # = NDOF(정답)")
        print("  키: [x][y][z] 축선택  [space/enter] 순환  [+/-] alpha  [q] 종료")
        print("  cal = sys/gyro/accel/mag (각 0~3, 3=완전보정)\n")

        t_prev = time.perf_counter()
        try:
            with KeyReader() as keys:
                while True:
                    # ---- 입력 처리 (논블로킹) ----
                    k = keys.get()
                    if k:
                        if k in ("q", "Q"):
                            break
                        elif k in AXES:
                            axis = k
                        elif k in (" ", "\n", "\r"):
                            axis = AXES[(AXES.index(axis) + 1) % 3]
                        elif k == "+":
                            alpha = min(0.999, round(alpha + 0.005, 3))
                            comp.alpha = yaw_f.alpha = alpha
                        elif k == "-":
                            alpha = max(0.50, round(alpha - 0.005, 3))
                            comp.alpha = yaw_f.alpha = alpha

                    # ---- 센서 읽기 (burst: 1회 I2C 트랜잭션, 모두 같은 순간) + dt ----
                    s = imu.read_all()
                    ax, ay, az = s["accel"]
                    gx, gy, gz = s["gyro"]
                    mx, my, mz = s["mag"]
                    b_head, b_roll, b_pitch = s["euler"]   # 원본순서(heading,roll,pitch)
                    now = time.perf_counter()
                    dt = now - t_prev
                    t_prev = now

                    # ---- 세 축 필터 동시 갱신 (전환 시 재수렴 없음) ----
                    a_roll, a_pitch = cal.acc_tilt(*geometry.accel_tilt(ax, ay, az))
                    c_roll, c_pitch = comp.update(ax, ay, az, gx, gy, dt)
                    c_yaw = yaw_f.update(mx, my, gz, dt)
                    a_yaw = cal.mag_yaw(geometry.mag_heading(mx, my))
                    # euler_std 와 동일 변환: 축 스왑(roll<->pitch) + yaw ±180
                    n_roll, n_pitch = b_pitch, b_roll
                    n_yaw = wrap180(b_head)
                    calstat = imu.calibration_status()

                    # ---- 선택 축의 (기준, COMP, 정답) 고르기 ----
                    if axis == "x":
                        ref, cmp, ndof = a_roll, c_roll, n_roll
                    elif axis == "y":
                        ref, cmp, ndof = a_pitch, c_pitch, n_pitch
                    else:  # z
                        ref, cmp, ndof = a_yaw, c_yaw, n_yaw

                    line = _bar([(ref, "."), (cmp, "*"), (ndof, "#")])
                    err = wrap180(cmp - ndof)
                    rlab = REF_LABEL[axis]
                    print(f"[{axis}|a{alpha:.3f}] [{line}]  "
                          f"{rlab}{ref:7.1f} COMP{cmp:7.1f} NDOF{ndof:7.1f}"
                          f"  Δ{err:+5.1f}  cal{calstat[0]}{calstat[1]}{calstat[2]}{calstat[3]}",
                          end="\r", flush=True)
                    time.sleep(0.02)        # ~50 Hz
        except KeyboardInterrupt:
            pass
        print(f"\n종료. (마지막 축={axis}, alpha={alpha})")


if __name__ == "__main__":
    main()
