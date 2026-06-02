# BNO055 IMU 직접 구현 실습 (라즈베리파이 zero 2w / CLI 전용)

BNO055는 스스로 9축 센서 퓨전을 하는 똑똑한 칩이지만, **그 내부 원리를 이해하려고**
센서를 **AMG(Raw Data) 모드**로 두고 자세 추정 알고리즘을 직접 구현한 실습입니다.
matplotlib 없이 **터미널(CLI)** 만으로 동작하도록 만들었습니다.

## 하드웨어 / 환경
- 센서: BNO055 @ **I2C bus 1, 주소 `0x29`** (이 보드는 ADR 핀 HIGH → 기본 0x28 아님)
- 머신: Raspberry Pi Zero 2W
- 의존성: `smbus2` 만 사용 (numpy/matplotlib/adafruit **불필요**, 표준 `math`)

연결 확인:
```bash
i2cdetect -y 1        # 0x29 가 보이면 OK
python3 bno055_driver.py   # CHIP_ID/raw/euler 한 번 찍어보기
```

## 코드 구조 (계층 분리)
센서퓨전에서 가장 흔한 버그가 359°↔0° 경계(wrapping) 처리다. 그래서 실무처럼
**내부 표현=쿼터니언 / 미분·적분=쿼터니언 / 사람이 보는 출력=오일러** 로 계층을
나눴다. 변환·연산을 파일마다 흩지 않고 한 곳에 모은다:

```
geometry.py     순수 수학·변환만 (상태/IO/부호보정 없음)
                wrap180, 쿼터니언 연산, quat_to_euler, projected_gravity,
                accel_tilt, mag_heading, tilt_compensated_heading
calibration.py  축 부호 정렬 계층 (6부호 Calibration 객체, load/save/apply)
filters.py      필터 클래스만 (Gyro/Complementary/Yaw). 수학은 geometry 에 위임
bno055_driver.py 하드웨어 I/O + 스케일링. euler_std / euler_from_quat /
                projected_gravity 는 geometry 로 위임
```
- **geometry 는 부호 없는 순수 기하**만 계산한다. 센서/마운팅 규약 정렬은
  `calibration.Calibration` 이 명시적으로 적용한다(예전엔 accel_tilt 에 부호가
  박혀 있어, 부호 미보정 입력이 필요한 틸트보상과 충돌했다 — 이제 해소).
- 필터는 생성 시 `calib=` 로 Calibration 을 주입받는다(없으면 +1 기본).

## 파일 구성
| 파일 | 내용 |
|------|------|
| `geometry.py` | 순수 변환: `wrap180`, `quat_to_euler`, `projected_gravity`, `quat_rotate_inverse`, `accel_tilt`, `mag_heading`, `tilt_compensated_heading` |
| `calibration.py` | `Calibration`(6부호 적용), `load()` — 축 부호 정렬 계층 |
| `bno055_driver.py` | smbus2 드라이버 (AMG/NDOF). `euler_std`, `euler_from_quat`, `projected_gravity` |
| `filters.py` | `GyroIntegrator`, `ComplementaryFilter`, `YawComplementaryFilter` (수학은 geometry 위임) |
| `stage1_tiltmeter.py` | 1단계: 가속도만 경사계 |
| `stage2_deadreckoning.py` | 2단계: 자이로 적분(드리프트 관찰) |
| `stage3_complementary.py` | 3단계: 상보 필터 (ACC/GYRO/COMP 비교) |
| `stage4_compare.py` | 4단계: 내 상보필터 vs BNO055 정답(NDOF), roll 한 축 ASCII 그래프 |
| `stage5_axes.py` | 5단계: **X/Y/Z 축 전환** 실시간 비교 (키로 축·alpha 변경) |
| `stage6_compass.py` | 6단계: **틸트 보상 나침반**(내 구현) vs NDOF 절대 yaw 비교 |

## 단계별 실행
```bash
python3 calibrate_signs.py         # (먼저 1회) 좌표계 부호를 NDOF 기준으로 보정
python3 stage1_tiltmeter.py        # 천천히 기울이면 정확 / 툭 치면 요동 → 가속도의 한계
python3 stage2_deadreckoning.py    # 가만 둬도 각도가 흘러감 → 자이로 드리프트
python3 stage3_complementary.py 0.98   # 흔들어도 안정 + 즉각 반응 (alpha 튜닝)
python3 stage4_compare.py 0.98     # COMP(*) 가 NDOF(#) 정답을 따라가면 성공! (roll 한 축)
python3 stage5_axes.py 0.98        # X/Y/Z 세 축을 키로 전환하며 비교
python3 stage6_compass.py          # 틸트보상 나침반(*) 이 NDOF yaw(#) 를 따라가면 성공
```
종료는 모두 `Ctrl-C` (stage5 는 `q` 도 가능).

### 6단계: 틸트 보상 나침반 (`stage6_compass.py`)
yaw(방위)를 자력계로 구할 때, 센서가 기울면 단순식 `atan2(my,mx)` 는 틀어진다.
가속도로 구한 roll/pitch 로 자기장 벡터를 **수평면으로 되돌린 뒤**(틸트 보상)
헤딩을 계산하면, 기울여도 정확한 자북을 가리키는 진짜 나침반이 된다.

세 헤딩을 비교: `RAW(.)` 틸트보상 없음 · `TILT(*)` 내 틸트보상 구현 ·
`NDOF(#)` BNO055 칩의 9축 절대 yaw(정답). **센서를 기울인 채 수평 회전**시키면
RAW 만 틀어지고 TILT 는 NDOF 를 따라간다.

핵심: **자력계는 raw 값 자체는 roll/pitch/yaw 에 다 반응하지만, 자세추정에서의
역할은 yaw 뿐이다. 단 yaw 를 구하려면 가속도의 roll/pitch 를 빌려야 한다**
(중력=수직 기준 → roll/pitch, 자기장=수평 기준 → yaw, 둘이 협력).

```
Xh = mx·cosθ + my·sinφ·sinθ + mz·cosφ·sinθ
Yh = my·cosφ - mz·sinφ
heading = atan2(-Yh, Xh)              # φ=roll, θ=pitch
```
> ⚠ 자력계 보정 필수: `cal` 마지막 자리(mag)가 3 이 되도록 시작 전 센서를
> 공중에서 8자로 휘저어라. mag<3 이면 NDOF yaw 자체가 안 맞아 비교가 무의미.
> 지구 자기장은 수평이 아니라 아래로 기울어(복각, 한국 ~53°) `mz` 도 식에 들어간다.

### 0단계(권장): 좌표계 부호 보정 (`calibrate_signs.py`)
내 표준 공식과 BNO055 내장 퓨전은 **축 부호 규약이 다를 수 있다**(센서를
어떻게 들고 움직이느냐에 따라 pitch/yaw 가 정답과 반대로 움직임). 이를
**출력 각도에 -1 곱하기로 고치면 안 된다** — yaw 는 자이로는 맞는데 자력계만
반대인 경우가 있어, 출력 한 번 뒤집기로는 표현이 안 되기 때문이다.

그래서 부호를 **센서 채널(입력) 6개**로 둔다:
`ACC_SIGN_ROLL/PITCH`, `MAG_SIGN_YAW`, `GYRO_SIGN_ROLL/PITCH/YAW`.

`calibrate_signs.py` 는 **NDOF 를 기준**으로 이 6개를 1회 측정한다(델타 상관:
내 각도 변화와 NDOF 각도 변화가 같은 방향이면 +1, 반대면 -1). 결과는
`calib_signs.json` 에 저장되고, 이후 stage3/4/5 가 시작 시 자동으로 불러
**보정 단계 없이 즉시 정렬된 값**으로 시작한다.

```bash
python3 calibrate_signs.py          # 8초간 측정 (또는 인자로 초 지정)
```
측정 중 화면 안내대로 **roll·pitch·yaw 세 방향 모두 충분히(>40°) 천천히**
돌려야 한다. 한 축이라도 덜 움직이면 그 축은 "이동 부족 → +1 기본"으로
표시되니, 그 축을 더 돌려 다시 측정하면 된다.
> `calib_signs.json` 은 센서/마운팅/측정 세션마다 다른 값이다. 이 저장소에는
> 실험 기록용으로 한 번 측정한 값을 포함해 두었지만, 다른 환경에서는 맞지
> 않을 수 있으니 `python3 calibrate_signs.py` 로 다시 측정하라. 파일이 없으면
> 모든 부호 +1 로 동작하며 안내가 뜬다.

### 5단계: X / Y / Z 축 전환 (`stage5_axes.py`)
한 화면에서 축을 바꿔가며 본다. 세 축 필터를 **매 루프 동시에 갱신**하므로
축을 바꿔도 재수렴 없이 즉시 전환된다(실시간 유지).

| 키 | 동작 |
|----|------|
| `x` `y` `z` | 해당 축으로 즉시 전환 |
| `space` / `enter` | X → Y → Z 순환 |
| `+` / `-` | alpha 실시간 증감 (튜닝) |
| `q` | 종료 |

축마다 **저주파 기준이 다르다**:
- **X(roll) · Y(pitch)** : 기준 = **ACC(가속도/중력)**
- **Z(yaw)** : 기준 = **MAG(자력계/나침반)**

왜? 가속도(중력)는 yaw 를 못 잡는다 — 수직축을 돌려도 중력 벡터가 안 변하기
때문. 그래서 yaw 의 드리프트는 자력계가 잡아준다. **roll/pitch=가속도,
yaw=자력계**가 자이로를 보정하는 것, 이것이 9축 융합(NDOF)이 필요한 이유다.

> ⚠️ **Z(yaw)는 mag 보정 전엔 절댓값이 안 맞는다.** 화면의 `cal` 마지막 자리
> (mag, 0~3)가 3 이 되도록 센서를 공중에서 8자로 천천히 흔들어 보정하라.
> 또한 이 실습의 `mag_heading` 은 틸트 보상이 없으므로(센서 수평 가정),
> yaw 비교는 X/Y 축을 평평히 둔 채 Z 만 돌릴 때 의미가 크다.

> roll 은 ±180° 를 넘나들 수 있어(특히 칩이 뒤집힌 마운팅) 상보필터 내부에서
> `wrap180` 으로 경계를 처리한다. yaw 도 동일. 덕분에 칩이 뒤집혀 roll 이
> −180° 근처에 앉아 있어도 안정적으로 추적한다(실측 중앙값 오차 X 0.75°, Y 0.78°).

## 핵심 수식 요약
- **경사계(Tilt):** `roll = atan2(ay, az)`, `pitch = atan2(-ax, √(ay²+az²))`
- **자이로 적분:** `angle += gyro_rate · dt`
- **상보 필터:** `angle = α·(angle + gyro·dt) + (1-α)·accel_angle`  (α≈0.95~0.99)

## 튜닝 / 트러블슈팅
- **α(alpha)**: 클수록(0.99) 부드럽지만 드리프트가 약간 남고, 작을수록(0.90)
  가속도 노이즈가 더 들어옴. 0.98 부근에서 균형점을 찾는 게 실습 목표.
- **roll 이 NDOF 와 ~180° 차이로 나오면**: 센서가 평평할 때 `az` 가 음수(-9.8)인
  마운팅이다(칩이 뒤집힘). 정상 마운팅(az≈+9.8)으로 두거나, 어느 쪽이든
  `pitch` 는 그대로 일치한다(실측 확인: ACC/COMP/NDOF pitch 오차 0.06° 수준).
- **축이 반대로 움직이면** 수동으로 고치지 말고 `python3 calibrate_signs.py` 로
  6개 부호를 재측정한다(`calib_signs.json` 갱신). 부호는 `calibration.Calibration`
  이 관리하며 geometry 순수값에 적용된다.
- **NDOF 정답이 흔들리면**: `cal sgam` 값이 보일 때, 각 항목이 3 이 되도록
  센서를 8자로 흔들고(mag), 여러 자세로 잠깐씩 정지(accel/gyro)시켜 보정한다.

### ⚠️ 라즈베리파이 I2C 클럭 스트레칭 버그 (이 보드에서 실제 발생)
이 Pi(bus 1, 100kHz)에서는 **단일 바이트 읽기/쓰기에 간헐적으로 bit7(0x80)이
끼어드는 손상**이 관측됐다. 예: `OPR_MODE` 0x0c → 0x8c 로 읽힘. 모드 쓰기가
깨지면 센서가 CONFIG 에 머물러 **모든 데이터가 정확히 0.00** 으로 나온다.

`bno055_driver.py` 는 이를 소프트웨어로 흡수한다:
- 레지스터 읽기는 항상 **블록 읽기**(`read_i2c_block_data`) 사용 — 단일바이트보다 안정적
- 모드 변경은 **쓰고 → 되읽어 확인**을 재시도(`set_mode`), bit7 은 마스킹
- 모든 대기 루프에 **타임아웃** (무한 hang 방지)

위 대응만으로 실측상 손상 0건(40/40)으로 동작한다. 그래도 불안정하면
하드웨어 차원에서 **I2C 속도를 낮추는 것이 근본 해결책**이다(재부팅 필요):
```bash
# /boot/firmware/config.txt 에 추가
dtparam=i2c_arm=on,i2c_arm_baudrate=50000   # 100k -> 50k (또는 10000 까지)
sudo reboot
```
```
