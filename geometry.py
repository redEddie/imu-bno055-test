"""
geometry.py
===========
IMU 좌표/자세 변환의 '순수 수학' 계층.  상태도, 하드웨어 I/O 도, 부호 보정도
여기엔 없다.  표준 라이브러리 math 만 사용한다(라즈베리파이 zero 2w 친화).

설계 원칙 (이번 디버깅에서 얻은 교훈)
------------------------------------
- 자세의 '내부 표현'은 쿼터니언(또는 회전행렬)이 정답이다. 오일러각은 사람이
  보기 위한 '출력 표현'일 뿐이며, 359°↔0° 같은 경계(wrapping) 버그의 온상이다.
- 그래서 이 파일은 두 묶음으로 나뉜다:
    (1) 쿼터니언 연산 + 쿼터니언→오일러 변환  (권장 경로)
    (2) raw accel/mag 로 직접 각도를 푸는 고전식 (학습/비교용)
- 모든 함수는 '부호 보정(축 정렬)을 적용하지 않은' 순수 기하값을 반환한다.
  센서/마운팅 규약에 맞추는 부호·축 정렬은 calibration.py 가 담당한다.

쿼터니언 표기
-------------
이 모듈의 쿼터니언은 (w, x, y, z) 스칼라-우선(Hamilton) 순서다.
BNO055 의 quaternion() 출력과 같은 순서이므로 그대로 넘기면 된다.
"""

import math


# ---------------------------------------------------------------------------
# 각도 정규화 (wrapping)
# ---------------------------------------------------------------------------
def wrap180(a):
    """각도를 [-180, 180) 로 접는다. yaw 처럼 360° 순환하는 양에 필수.
    (±180 은 같은 각이며 이 식은 경계를 -180 으로 돌려준다.)"""
    return (a + 180.0) % 360.0 - 180.0


def wrap360(a):
    """각도를 [0, 360) 으로 접는다. (헤딩을 0~360 으로 보고 싶을 때)"""
    return a % 360.0


def angle_diff(a, b):
    """두 각도의 최단 차이 a-b 를 [-180,180) 로. (오차/미분 계산용)"""
    return wrap180(a - b)


# ---------------------------------------------------------------------------
# 쿼터니언 연산  ('내부 표현' 경로)
# ---------------------------------------------------------------------------
def quat_normalize(q):
    """(w,x,y,z) 정규화. 영벡터면 단위 쿼터니언 반환."""
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return (w / n, x / n, y / n, z / n)


def quat_rotate_inverse(q, v):
    """
    쿼터니언 q (w,x,y,z) 의 '역회전' 을 벡터 v 에 적용.
    = 월드 좌표 벡터 v 를 몸체(body) 좌표로 가져온다.

    projected_gravity 등에 쓰인다. (Open_Duck_Mini 의 quat_rotate_inverse 와
    동일한 연산; 단 여기선 (w,x,y,z) 순서를 받는다.)
    """
    w, x, y, z = q
    vx, vy, vz = v
    # a = v*(2w^2-1)
    a = (vx * (2.0 * w * w - 1.0),
         vy * (2.0 * w * w - 1.0),
         vz * (2.0 * w * w - 1.0))
    # b = cross(qvec, v) * 2w
    cx = y * vz - z * vy
    cy = z * vx - x * vz
    cz = x * vy - y * vx
    b = (cx * 2.0 * w, cy * 2.0 * w, cz * 2.0 * w)
    # c = qvec * dot(qvec, v) * 2
    d = (x * vx + y * vy + z * vz) * 2.0
    c = (x * d, y * d, z * d)
    return (a[0] - b[0] + c[0],
            a[1] - b[1] + c[1],
            a[2] - b[2] + c[2])


def quat_rotate(q, v):
    """
    쿼터니언 q (w,x,y,z) 의 '정회전' 을 벡터 v 에 적용.
    = 몸체(body) 좌표 벡터 v 를 월드 좌표로 가져온다.  (quat_rotate_inverse 의 역)
    """
    w, x, y, z = q
    vx, vy, vz = v
    a = (vx * (2.0 * w * w - 1.0),
         vy * (2.0 * w * w - 1.0),
         vz * (2.0 * w * w - 1.0))
    cx = y * vz - z * vy
    cy = z * vx - x * vz
    cz = x * vy - y * vx
    b = (cx * 2.0 * w, cy * 2.0 * w, cz * 2.0 * w)
    d = (x * vx + y * vy + z * vz) * 2.0
    c = (x * d, y * d, z * d)
    # 정회전은 +b (역회전은 -b)
    return (a[0] + b[0] + c[0],
            a[1] + b[1] + c[1],
            a[2] + b[2] + c[2])


def quat_tilt_heading(q, mx, my, mz):
    """
    쿼터니언 기반 틸트보상 헤딩(yaw).  roll/pitch 를 손으로 안 쓴다.

    원리
    ----
    센서가 읽은 자기장 벡터(body 좌표)를, 쿼터니언으로 '월드 좌표'로 회전시키면
    기울기가 자동으로 제거된다. 그 월드 자기장의 수평면(x,y) 성분으로 헤딩을
    구한다:
        m_world = R(q) · m_body
        heading = atan2(m_world_y, m_world_x)

    accel 로 roll/pitch 를 따로 구해 mag 를 수동으로 펴는 고전 틸트보상과 달리,
    회전 전체를 쿼터니언이 처리하므로 'accel-mag 축이 1:1 로 안 맞는' 문제에
    영향받지 않는다(둘을 수동 정렬할 필요가 없음).

    주의: q 는 BNO055 NDOF 쿼터니언(이미 9축 융합). mag 자체의 하드/소프트
    아이언 왜곡이 남아 있으면 절대 정확도엔 한계가 있다.
    """
    wx, wy, wz = quat_rotate(q, (mx, my, mz))
    return math.degrees(math.atan2(wy, wx))


def projected_gravity(q):
    """
    몸체 좌표계에서 본 중력 단위벡터.  로봇 RL 정책이 자세 입력으로 쓰는 바로
    그 양이다(Open_Duck_Mini v2 의 핵심 관측). 월드 중력 [0,0,-1] 을 몸체로
    역회전한다.

    오일러각과 달리 짐벌락/불연속이 없어 신경망·제어에 적합하다.
    """
    return quat_rotate_inverse(q, (0.0, 0.0, -1.0))


def quat_to_euler(q):
    """
    쿼터니언 (w,x,y,z) -> 오일러각 (roll, pitch, yaw) 도(degree).
    표준 항공 ZYX 규약:
        roll  : X축 회전 (±180)
        pitch : Y축 회전 (±90, asin -> 짐벌락 지점)
        yaw   : Z축 회전 (±180)
    결과는 모두 [-180,180) 또는 pitch [-90,90] 로 정규화된다.
    이것이 '내부=쿼터니언 -> 출력=오일러' 의 표준 변환 계층이다.
    """
    w, x, y, z = quat_normalize(q)

    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation) — asin, 짐벌락 지점에서 ±90 로 클램프
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


# ---------------------------------------------------------------------------
# raw 센서 -> 각도 직접 계산  (고전식, 학습/비교용)  — 모두 부호 보정 없음
# ---------------------------------------------------------------------------
def accel_tilt(ax, ay, az):
    """
    가속도(중력) 원시값만으로 (roll, pitch) 를 도(degree) 로.  부호 보정 없음.

        roll  = atan2(ay, az)
        pitch = atan2(-ax, sqrt(ay^2 + az^2))

    정지/저주파엔 정확하지만 충격·선형가속(고주파)에 취약하다. yaw 는 못 구한다
    (수직축 회전에 중력 벡터가 안 변하므로).
    """
    roll = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
    return roll, pitch


def mag_to_accel_frame(mx, my, mz):
    """
    BNO055 자력계 raw 축을 가속도(중력) 축과 같은 좌표계로 재매핑한다.

    실측 배경 (accel-mag 축 교차상관)
    --------------------------------
    센서를 여러 축으로 크게 기울인 336표본에서 accel 3축과 mag 3축의 상관을
    구하면 대각선이 강한 음수였다:
        acc_x↔mag_x -0.93,  acc_y↔mag_y -0.93,  acc_z↔mag_z -0.82
    즉 mag 는 accel 과 '같은 축 순서'지만 '세 축 모두 부호 반대'다.
    따라서 가속도 좌표계로 맞추려면 세 축 부호만 뒤집으면 된다:

        x' = -mx,   y' = -my,   z' = -mz

    (단순 헤딩 atan2(my,mx) 에선 x,y 부호가 상쇄돼 안 드러나지만, 틸트보상은
     mz 부호에 민감해 이 매핑이 필요하다.)

    주의: 잔여 편차와 절대 오프셋은 mag 전용 하드/소프트아이언 보정을 직접 안
    한 탓(BNO055 NDOF 는 칩이 그걸 해서 더 정확). 정밀 heading 은 NDOF 를 쓰고,
    이 함수는 '틸트보상 원리 학습/검증' 용이다.
    """
    return (-mx, -my, -mz)


def mag_heading(mx, my):
    """
    자력계로 yaw(헤딩)를 계산.  부호 보정 없음.

        heading = atan2(my, mx)

    '센서가 수평일 때만' 맞는 단순식(틸트 보상 없음). 기울이면 틀어진다.
    틸트까지 보상하려면 tilt_compensated_heading 을 쓴다.
    """
    return math.degrees(math.atan2(my, mx))


def tilt_compensated_heading(mx, my, mz, roll_deg, pitch_deg):
    """
    틸트 보상 나침반 헤딩(yaw).  부호 보정 없음.

    가속도로 구한 roll/pitch 로 자기장 벡터를 수평면으로 되돌린 뒤 헤딩 계산.
    -> 센서가 기울어도 자북을 가리키는 진짜 나침반.

        Xh = mx·cosθ + my·sinφ·sinθ + mz·cosφ·sinθ
        Yh = my·cosφ - mz·sinφ
        heading = atan2(-Yh, Xh)          (φ=roll, θ=pitch)

    여기 들어가는 roll/pitch 는 '순수 기하각'(accel_tilt 의 부호 미보정 출력)
    이어야 식이 성립한다. 지구 자기장은 아래로 기울어(복각) mz 도 들어간다.
    """
    phi = math.radians(roll_deg)
    theta = math.radians(pitch_deg)
    sin_p, cos_p = math.sin(phi), math.cos(phi)
    sin_t, cos_t = math.sin(theta), math.cos(theta)

    Xh = mx * cos_t + my * sin_p * sin_t + mz * cos_p * sin_t
    Yh = my * cos_p - mz * sin_p
    return math.degrees(math.atan2(-Yh, Xh))
