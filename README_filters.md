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

## 파일 구성
| 파일 | 내용 |
|------|------|
| `bno055_driver.py` | smbus2 기반 최소 드라이버 (AMG raw / NDOF / calib) |
| `filters.py` | `accel_tilt`, `mag_heading`, `wrap180`, `GyroIntegrator`, `ComplementaryFilter`, `YawComplementaryFilter` |
| `stage1_tiltmeter.py` | 1단계: 가속도만 경사계 |
| `stage2_deadreckoning.py` | 2단계: 자이로 적분(드리프트 관찰) |
| `stage3_complementary.py` | 3단계: 상보 필터 (ACC/GYRO/COMP 비교) |
| `stage4_compare.py` | 4단계: 내 상보필터 vs BNO055 정답(NDOF), roll 한 축 ASCII 그래프 |
| `stage5_axes.py` | 5단계: **X/Y/Z 축 전환** 실시간 비교 (키로 축·alpha 변경) |

## 단계별 실행
```bash
python3 calibrate_signs.py         # (먼저 1회) 좌표계 부호를 NDOF 기준으로 보정
python3 stage1_tiltmeter.py        # 천천히 기울이면 정확 / 툭 치면 요동 → 가속도의 한계
python3 stage2_deadreckoning.py    # 가만 둬도 각도가 흘러감 → 자이로 드리프트
python3 stage3_complementary.py 0.98   # 흔들어도 안정 + 즉각 반응 (alpha 튜닝)
python3 stage4_compare.py 0.98     # COMP(*) 가 NDOF(#) 정답을 따라가면 성공! (roll 한 축)
python3 stage5_axes.py 0.98        # X/Y/Z 세 축을 키로 전환하며 비교
```
종료는 모두 `Ctrl-C` (stage5 는 `q` 도 가능).

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
- **자이로 방향이 반대로 적분되면** `filters.py` 의 `GYRO_SIGN_ROLL` /
  `GYRO_SIGN_PITCH` 를 `-1.0` 로 바꿔 부호를 맞춘다.
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
