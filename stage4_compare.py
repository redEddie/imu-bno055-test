#!/usr/bin/env python3
"""
4단계: 최종 마스터 — 내 상보 필터 vs BNO055 '정답지'
====================================================
matplotlib 가 없는 CLI 환경(라즈베리파이 zero 2w)에 맞춰, 터미널에 ASCII
실시간 막대 그래프로 3개의 pitch 곡선을 동시에 그린다.

    ACC   : 가속도만으로 계산한 거친 roll           (점 .)
    COMP  : 내가 직접 구현한 상보 필터 roll          (별 *)
    NDOF  : BNO055 내장 퓨전 엔진의 roll(정답)        (샵 #)

합격 기준: COMP(*) 곡선이 NDOF(#) 정답 곡선을 부드럽게 잘 따라가면 성공!

핵심 트릭 1 (모드 토글 금지!)
----------------------------
BNO055 는 NDOF(fusion) 모드에서도 가속도/자이로 '데이터 레지스터'를 그대로
갱신한다. 즉 NDOF 한 모드만 켜둔 채로
  - raw accel/gyro  -> 내 상보필터 입력
  - euler           -> 정답
을 동시에 읽을 수 있다.
(AMG<->NDOF 를 매 루프 토글하면 퓨전 엔진이 수렴할 시간을 못 얻어 euler 가
0 으로 나온다. 그래서 토글하지 않는다.)

핵심 트릭 2 (euler 축 규약 변환)
-------------------------------
BNO055 의 euler 규약은 표준 항공 규약과 roll/pitch 가 뒤바뀌어 있다
(BNO055: roll ±90°, pitch ±180°). 그래서 정답은 euler() 가 아니라
euler_std() 로 읽어 우리의 (roll, pitch) 와 같은 축에 맞춘다.
(실측: 맞바꾸면 가속도 경사계와 0.1~1.0° 오차로 일치)

실행:  python3 stage4_compare.py [alpha]
종료:  Ctrl-C
"""

import sys
import time

from bno055_driver import BNO055, MODE_NDOF
import geometry
import calibration
from filters import ComplementaryFilter

PLOT_WIDTH = 61          # 막대 칸 수 (홀수면 가운데가 0°)
ANGLE_RANGE = 90.0       # 화면 양 끝이 ±90°


def _bar(values_with_marks):
    """
    values_with_marks: [(angle_deg, char), ...]
    -ANGLE_RANGE..+ANGLE_RANGE 를 PLOT_WIDTH 칸에 매핑한 한 줄 문자열.
    """
    cells = [" "] * PLOT_WIDTH
    center = PLOT_WIDTH // 2
    cells[center] = "|"      # 0° 기준선
    for angle, ch in values_with_marks:
        a = max(-ANGLE_RANGE, min(ANGLE_RANGE, angle))
        idx = int(round((a + ANGLE_RANGE) / (2 * ANGLE_RANGE) * (PLOT_WIDTH - 1)))
        cells[idx] = ch
    return "".join(cells)


def main():
    alpha = float(sys.argv[1]) if len(sys.argv) > 1 else 0.98

    cal = calibration.load()        # 부호 보정 적용 (없으면 +1 기본 + 안내)

    with BNO055() as imu:
        imu.set_mode(MODE_NDOF)
        print("[4단계] NDOF 퓨전 엔진 수렴 대기 중...", flush=True)
        time.sleep(2.0)                         # 퓨전 수렴 + 데이터 안정화

        ax, ay, az = imu.accel_settled()
        comp = ComplementaryFilter(alpha=alpha, calib=cal)
        comp.seed(ax, ay, az)                   # 정답 근처에서 시작

        print(f"[4단계] 상보필터 vs BNO055 정답지   alpha={alpha}")
        print(f"  막대 범위: -{ANGLE_RANGE:.0f}° (왼쪽) ~ +{ANGLE_RANGE:.0f}° (오른쪽), "
              f"가운데 '|' = 0°")
        print("  roll(±180°) 비교.  . = ACC(가속도만)  * = COMP(내 상보필터)  # = NDOF(정답)")
        print("  cal=sys/gyro/accel/mag (각 0~3, 3이 완전보정)   Ctrl-C 로 종료\n")

        t_prev = time.perf_counter()
        try:
            while True:
                ax, ay, az = imu.accel()        # NDOF 모드에서도 raw 갱신됨
                gx, gy, gz = imu.gyro()
                now = time.perf_counter()
                dt = now - t_prev
                t_prev = now

                a_roll, a_pitch = cal.acc_tilt(*geometry.accel_tilt(ax, ay, az))
                c_roll, c_pitch = comp.update(ax, ay, az, gx, gy, dt)
                _, n_roll, n_pitch = imu.euler_std()   # 표준 규약으로 정답 읽기
                calstat = imu.calibration_status()

                # roll(±180°)이 자세 변화를 더 넓게 보여줘 비교가 잘 보인다.
                line = _bar([(a_roll, "."), (c_roll, "*"), (n_roll, "#")])
                err = c_roll - n_roll
                print(f"[{line}]  "
                      f"ACC{a_roll:6.1f} COMP{c_roll:6.1f} NDOF{n_roll:6.1f}"
                      f"  Δ{err:+5.1f}  cal{calstat[0]}{calstat[1]}{calstat[2]}{calstat[3]}",
                      end="\r", flush=True)
                time.sleep(0.02)                # ~50 Hz
        except KeyboardInterrupt:
            print("\n종료.")


if __name__ == "__main__":
    main()
