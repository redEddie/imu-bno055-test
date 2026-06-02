#!/usr/bin/env python3
"""
센서 좌표축 전수 진단 (마운팅/보정 무시, raw 그대로)
====================================================
accel/gyro/mag 의 각 물리축(x,y,z)이 BNO055 내장 NDOF 의 자세축
(roll/pitch/yaw)과 어떻게 대응하는지를 '상관(correlation)'으로 전부
드러낸다. 부호 보정(calib_signs)도, 마운팅 가정도 적용하지 않은 순수 raw.

읽는 법
-------
센서를 roll/pitch/yaw 세 방향으로 충분히 천천히 돌리는 동안:
  - accel/mag 는 '값 자체' 와 NDOF '각도' 의 상관 (정적 자세 센서)
  - gyro 는 '값(각속도)' 와 NDOF '각도의 변화율' 의 상관 (동적 센서)
을 표로 출력한다. 표의 각 칸은 -1.0 ~ +1.0:
  +1 에 가까움 = 그 물리축이 그 자세축과 같은 방향으로 강하게 대응
  -1 에 가까움 = 반대 방향으로 대응
   0 에 가까움 = 무관

이 표를 BNO055 데이터시트의 축 정의(또는 기대한 좌표계)와 대조하면,
"x축이 사실 pitch에 매핑된다", "z(mag)가 yaw에 -로 대응한다" 같은
실제 배선/규약을 마운팅과 무관하게 확정할 수 있다.

사용법:
    python3 diagnose_axes.py            # 12초 측정
    python3 diagnose_axes.py 20         # 측정 시간(초)

측정 중 roll/pitch/yaw 를 골고루(각 >60°) 돌려야 표가 또렷해진다.
"""

import sys
import time
import math

from bno055_driver import BNO055, MODE_NDOF
from geometry import wrap180


def corr(xs, ys):
    """피어슨 상관계수. 분산이 0이면 0 반환."""
    n = len(xs)
    if n < 2:
        return 0.0
    mx = sum(xs) / n
    my = sum(ys) / n
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    sxx = sum((x - mx) ** 2 for x in xs)
    syy = sum((y - my) ** 2 for y in ys)
    if sxx <= 1e-12 or syy <= 1e-12:
        return 0.0
    return sxy / math.sqrt(sxx * syy)


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 12.0

    with BNO055() as imu:
        imu.set_mode(MODE_NDOF)
        print("[진단] NDOF 수렴 대기...", flush=True)
        time.sleep(2.0)

        print("=" * 60)
        print(f" {duration:.0f}초간 센서를 roll/pitch/yaw 모두 천천히 크게 돌리세요.")
        print(" (raw 축이 어느 자세축에 대응하는지 상관으로 측정합니다)")
        print("=" * 60)
        for n in (3, 2, 1):
            print(f"  {n}...", end=" ", flush=True)
            time.sleep(1.0)
        print("시작!\n")

        # 수집 버퍼 — 모두 '변화량(delta)' 으로 저장한다.
        #   이유: raw 값 vs 절대각도(피어슨)는 sin/cos 비선형 + yaw 의 ±180 wrap
        #   때문에 상관이 망가진다. 한 스텝 변화량끼리 비교하면 국소적으로 선형이라
        #   견고하고, 절대 오프셋/마운팅/wrap 에 영향받지 않는다.
        dA = {ax: [] for ax in "xyz"}    # d(accel)
        dG = {ax: [] for ax in "xyz"}    # gyro 는 이미 '각속도(=각도변화율)' 이므로 그대로
        dM = {ax: [] for ax in "xyz"}    # d(mag)
        dN = {"roll": [], "pitch": [], "yaw": []}   # d(NDOF angle) (wrap-safe)
        mv_total = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}  # 누적 이동량(범위 아님)

        pa = imu.accel()
        pm = imu.mag()
        pny, pnr, pnp = imu.euler_std()      # euler_std 가 이미 yaw 를 ±180 으로 줌
        t_prev = time.perf_counter()
        t_end = t_prev + duration

        while time.perf_counter() < t_end:
            ax, ay, az = imu.accel()
            gx, gy, gz = imu.gyro()
            mx, my, mz = imu.mag()
            ny, nr, npi = imu.euler_std()
            now = time.perf_counter()
            dt = now - t_prev
            t_prev = now
            if dt <= 0:
                continue

            dnr = wrap180(nr - pnr)
            dnp = wrap180(npi - pnp)
            dny = wrap180(ny - pny)

            dA["x"].append(ax - pa[0]); dA["y"].append(ay - pa[1]); dA["z"].append(az - pa[2])
            dM["x"].append(mx - pm[0]); dM["y"].append(my - pm[1]); dM["z"].append(mz - pm[2])
            dG["x"].append(gx); dG["y"].append(gy); dG["z"].append(gz)   # 이미 rate
            dN["roll"].append(dnr); dN["pitch"].append(dnp); dN["yaw"].append(dny)
            mv_total["roll"] += abs(dnr)
            mv_total["pitch"] += abs(dnp)
            mv_total["yaw"] += abs(dny)
            pa = (ax, ay, az); pm = (mx, my, mz)
            pnr, pnp, pny = nr, npi, ny

            print(f"  측정중... 샘플 {len(dA['x']):4d}   "
                  f"NDOF r{nr:+4.0f} p{npi:+4.0f} y{ny:+4.0f}", end="\r", flush=True)
            time.sleep(0.02)

        print("\n")
        print(f" NDOF 누적 이동량:  roll {mv_total['roll']:.0f}°  "
              f"pitch {mv_total['pitch']:.0f}°  yaw {mv_total['yaw']:.0f}°")
        for axis in ("roll", "pitch", "yaw"):
            if mv_total[axis] < 200:
                print(f"   ⚠ {axis} 를 더 돌려야 그 열의 신뢰도가 올라갑니다.")
        print()

        def table(title, data, ref):
            print(f" [{title}]   (행=물리축, 열=NDOF 자세축, 값=상관 -1..+1)")
            print(f"   {'':4s}{'roll':>8s}{'pitch':>8s}{'yaw':>8s}")
            for ax in "xyz":
                row = "   " + ax + "  "
                cells = []
                for axis in ("roll", "pitch", "yaw"):
                    c = corr(data[ax], ref[axis])
                    cells.append(f"{c:+8.2f}")
                # 가장 강한 대응 표시
                vals = [corr(data[ax], ref[a]) for a in ("roll", "pitch", "yaw")]
                k = max(range(3), key=lambda i: abs(vals[i]))
                tag = f"  -> {('roll','pitch','yaw')[k]} {'+' if vals[k]>0 else '-'}"
                print(row + "".join(cells) + tag)
            print()

        table("d(ACCEL) vs d(NDOF angle)", dA, dN)
        table("d(MAG)   vs d(NDOF angle)", dM, dN)
        table("GYRO     vs d(NDOF angle)", dG, dN)

        print(" 해석: 각 물리축 행에서 |상관|이 가장 큰 열이 그 축의 '진짜' 대응.")
        print("       부호(+/-)가 그 축의 방향. 이 표가 곧 좌표계 정렬의 근거다.")
        print(" (모두 변화량 기준 상관 -> sin/cos 비선형·절대오프셋·yaw wrap 에 견고)")


if __name__ == "__main__":
    main()
