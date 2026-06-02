#!/usr/bin/env python3
"""
좌표계 부호 1회 보정 (NDOF 기준)
================================
내 표준 공식(accel_tilt/mag_heading/자이로 적분)의 축 부호를 BNO055 내장
퓨전(euler_std)에 맞춘다. 6개 부호를 측정해 calib_signs.json 에 저장하면,
이후 stage 스크립트는 보정 단계 없이 즉시 정렬된 값으로 시작한다.

원리 — 델타 상관(delta correlation)
-----------------------------------
센서를 자유롭게 돌리는 동안, 매 순간 '내가 부호 +1 로 계산한 각도'의 변화량과
'NDOF 정답 각도'의 변화량을 곱해 누적한다.
  - 둘이 같은 방향으로 움직이면 곱이 +  -> 부호 +1
  - 반대로 움직이면 곱이 -          -> 부호 -1
각도 차이는 wrap180 으로 풀어 ±180 경계에서도 안전하다.

  ACC_SIGN_ROLL/PITCH : accel_tilt(+1) 의 roll/pitch 변화 vs NDOF roll/pitch 변화
  MAG_SIGN_YAW        : mag_heading(+1) 의 yaw 변화      vs NDOF yaw 변화
  GYRO_SIGN_ROLL/...  : gyro rate(gx,gy,gz)              vs NDOF roll/pitch/yaw 변화

자이로는 'rate' 라 적분 없이 d(NDOF)/이동방향과 직접 상관시킨다.

사용법
------
    python3 calibrate_signs.py            # 8초 동안 측정
    python3 calibrate_signs.py 12         # 측정 시간(초) 지정

화면 안내대로 센서를 roll/pitch/yaw 세 방향 모두 천천히 충분히 흔들어라.
한 축이라도 거의 안 움직이면 그 축은 '신뢰도 낮음' 으로 표시되고 부호를
+1 로 둔다(다시 측정 권장).
"""

import sys
import time

import geometry
from geometry import wrap180
from calibration import Calibration
from bno055_driver import BNO055, MODE_NDOF

DURATION_DEFAULT = 8.0
MIN_MOTION_DEG = 40.0     # 한 축에서 이 정도(누적 |변화|)는 움직여야 신뢰

# 보정은 'NDOF 대비 부호' 를 순수하게 재야 하므로, geometry 의 부호 미보정
# 순수 기하 함수를 그대로 쓴다(부호가 안 섞임).
raw_accel_angles = geometry.accel_tilt
raw_mag_yaw = lambda mx, my: geometry.mag_heading(mx, my)


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else DURATION_DEFAULT

    with BNO055() as imu:
        imu.set_mode(MODE_NDOF)
        print("[보정] NDOF 수렴 대기...", flush=True)
        time.sleep(2.0)

        print("=" * 60)
        print(f" 지금부터 {duration:.0f}초간 센서를 자유롭게 돌리세요.")
        print("  · X축(roll) 좌우로 기울이기")
        print("  · Y축(pitch) 앞뒤로 기울이기")
        print("  · Z축(yaw) 수평으로 빙글 돌리기")
        print("  세 방향 모두 충분히(>40°) 움직여야 정확합니다.")
        print("=" * 60)
        for n in (3, 2, 1):
            print(f"  {n}...", end=" ", flush=True)
            time.sleep(1.0)
        print("시작!\n")

        # 상관 누적기: corr[채널] += d_mine * d_ndof,  motion[축] += |d_ndof|
        corr = {k: 0.0 for k in (
            "ACC_SIGN_ROLL", "ACC_SIGN_PITCH", "MAG_SIGN_YAW",
            "GYRO_SIGN_ROLL", "GYRO_SIGN_PITCH", "GYRO_SIGN_YAW")}
        motion = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}

        # 직전 표본
        ax, ay, az = imu.accel(); mx, my, mz = imu.mag()
        p_aroll, p_apitch = raw_accel_angles(ax, ay, az)
        p_myaw = raw_mag_yaw(mx, my)
        p_nyaw, p_nroll, p_npitch = imu.euler_std()
        t_prev = time.perf_counter()
        t_end = t_prev + duration

        while time.perf_counter() < t_end:
            ax, ay, az = imu.accel()
            gx, gy, gz = imu.gyro()
            mx, my, mz = imu.mag()
            now = time.perf_counter(); dt = now - t_prev; t_prev = now

            aroll, apitch = raw_accel_angles(ax, ay, az)
            myaw = raw_mag_yaw(mx, my)
            nyaw, nroll, npitch = imu.euler_std()

            # NDOF 변화량 (wrap-safe)
            d_nroll  = wrap180(nroll  - p_nroll)
            d_npitch = wrap180(npitch - p_npitch)
            d_nyaw   = wrap180(nyaw   - p_nyaw)
            motion["roll"]  += abs(d_nroll)
            motion["pitch"] += abs(d_npitch)
            motion["yaw"]   += abs(d_nyaw)

            # 가속도/자력계: 내 각도(부호+1) 변화 vs NDOF 변화
            corr["ACC_SIGN_ROLL"]  += wrap180(aroll  - p_aroll)  * d_nroll
            corr["ACC_SIGN_PITCH"] += wrap180(apitch - p_apitch) * d_npitch
            corr["MAG_SIGN_YAW"]   += wrap180(myaw   - p_myaw)   * d_nyaw

            # 자이로: rate * dt(=각도 증분) vs NDOF 변화
            corr["GYRO_SIGN_ROLL"]  += (gx * dt) * d_nroll
            corr["GYRO_SIGN_PITCH"] += (gy * dt) * d_npitch
            corr["GYRO_SIGN_YAW"]   += (gz * dt) * d_nyaw

            p_aroll, p_apitch, p_myaw = aroll, apitch, myaw
            p_nroll, p_npitch, p_nyaw = nroll, npitch, nyaw

            bar = int(min(1.0, (now - (t_end - duration)) / duration) * 30)
            print(f"  측정중 [{'#'*bar}{'.'*(30-bar)}]  "
                  f"roll{motion['roll']:5.0f}° pitch{motion['pitch']:5.0f}° "
                  f"yaw{motion['yaw']:5.0f}°", end="\r", flush=True)
            time.sleep(0.02)

        print("\n")
        # 부호 결정 + 신뢰도 판정
        axis_of = {
            "ACC_SIGN_ROLL": "roll", "GYRO_SIGN_ROLL": "roll",
            "ACC_SIGN_PITCH": "pitch", "GYRO_SIGN_PITCH": "pitch",
            "MAG_SIGN_YAW": "yaw", "GYRO_SIGN_YAW": "yaw",
        }
        signs = {}
        low_conf = []
        print(" 채널            상관값        부호   (해당 축 이동량)")
        print(" " + "-" * 56)
        for k, c in corr.items():
            axis = axis_of[k]
            enough = motion[axis] >= MIN_MOTION_DEG
            s = +1 if c >= 0 else -1
            if not enough:
                s = +1
                low_conf.append(axis)
            signs[k] = s
            flag = "" if enough else "  <- 이동 부족, +1 기본"
            print(f" {k:16s} {c:+11.1f}    {'+1' if s>0 else '-1'}"
                  f"     ({axis} {motion[axis]:.0f}°){flag}")

        Calibration(signs).save()
        print(f"\n calib_signs.json 저장 완료.")
        if low_conf:
            uniq = ", ".join(sorted(set(low_conf)))
            print(f" ⚠ [{uniq}] 축이 충분히 안 움직였습니다. 그 축을 더 돌려 "
                  "다시 보정하세요:  python3 calibrate_signs.py")
        else:
            print(" ✅ 세 축 모두 신뢰도 충분. 이제 stage 스크립트를 실행하세요:")
            print("    python3 stage5_axes.py 0.98")


if __name__ == "__main__":
    main()
