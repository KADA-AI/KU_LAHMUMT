# LAH 3차원 경로계획 PPO 모델 이식 명세

## 1. 배포 모델

이 폴더에는 현재 프로젝트에서 누적 학습량이 가장 큰 정상 종료 모델을 수록하였다.

| 항목 | 값 |
|---|---|
| 모델 파일 | `model.zip` |
| 원본 학습 실행 | `20260719-025156-lah_ppo_multiterrain_adaptive_v2_1p2m_fi` |
| 누적 학습 시간단계 | 1,201,401 step |
| 완료 에피소드 | 18,473 episode |
| 알고리즘 | Stable-Baselines3 PPO |
| 정책망 | Actor-Critic MLP, 관측 68 → 256 → 256 → 행동 2, Tanh |
| 속도 제어 | 사용하지 않음. 수평속도 50 m/s 고정 |
| 학습된 정책 결정 간격 | 1, 2, 3, 4, 5 s |
| 권장 추론 | `deterministic=True` |

`inference_config.json`은 원본 학습 설정에서 로컬 PC의 재학습 체크포인트 절대경로만 제거한 이식용 설정이다. DEM 파일과 경로 생성 코드는 이 폴더에 포함하지 않는다.

## 2. 가장 중요한 해석

이 PPO 모델은 시작점과 종료점을 한 번 입력받아 전체 경로를 직접 출력하는 모델이 아니다. 현재 항공기 상태, 목표 상대방향, DEM 지형, 적 노출위험, 엄폐 안전정보와 A* 기준경로 정보를 합친 **68차원 관측 벡터**를 입력받아, 다음 구간에 적용할 **방위각 변화 명령과 수직속도 명령**을 출력하는 폐루프 정책이다.

따라서 다른 시스템에 이식할 때도 다음 계산부가 모델 외부에 필요하다.

1. DEM 고도·경사·전방 지형 프로파일 계산
2. 예상 적 위치 가설에 대한 노출위험 계산
3. 엄폐도, 안전 마스크와 적합도 계산
4. 목표까지의 A* 기준경로와 기준고도 계산
5. PPO 행동을 적용하는 기동 적분과 목표 도달 판정
6. 필요 시 경로점에 대한 수평 1회 평활화

관측 정의나 순서를 바꾸면 재학습 없이 정상적인 결과를 보장할 수 없다.

## 3. 실행 환경과 모델 로드

제작 환경은 Python 3.12.7, Stable-Baselines3 2.9.0, PyTorch 2.13.0+cpu, Gymnasium 1.3.0, NumPy 2.5.1이다. 동일 주 버전을 우선 권장한다. 별도의 `VecNormalize` 통계 파일은 사용하지 않는다. 정규화는 아래 관측식에서 직접 수행하며, 최종 관측은 `NaN→0`, `+Inf→5`, `-Inf→-5`로 바꾼 뒤 `[-5, 5]`로 제한한다.

```python
import numpy as np
from stable_baselines3 import PPO

model = PPO.load("model.zip", device="cpu")

# 아래 명세와 동일한 순서로 직접 구성해야 한다.
observation = np.asarray(observation_68, dtype=np.float32)
assert observation.shape == (68,)

action, _ = model.predict(observation, deterministic=True)
action = np.asarray(action, dtype=np.float32)
assert action.shape == (2,)
```

배치 추론은 관측을 `(N, 68)`로 전달하며, 행동은 `(N, 2)`로 반환된다.

## 4. 좌표와 공통 기호

- `x, y`: DEM과 동일한 투영좌표계의 미터 좌표
- `z`: DEM 수직기준과 일치하는 MSL 유사 고도(m)
- `AGL = z - DEM(x, y)`
- `ψ`: 현재 기수방향(rad)
- `dx, dy, dz`: 목표 위치에서 현재 위치를 뺀 값
- `d_xy = sqrt(dx² + dy²)`
- `S = max(초기 수평 목표거리, 500 m)`
- `forward = cos(ψ)dx + sin(ψ)dy`
- `right = -sin(ψ)dx + cos(ψ)dy`
- `bearing_error = wrap(atan2(dy, dx) - ψ)`
- `A* 기준경로`: 적 위험·엄폐·지형을 반영하여 외부 계획기가 만든 기준 폴리라인
- 운용구역 경계거리: 구역 내부는 양수, 외부는 음수

## 5. 모델 입력: 68차원 `float32` 관측

모델의 관측공간은 `Box(low=-5, high=5, shape=(68,), dtype=float32)`이다.

| 인덱스 | 입력값 | 계산 및 단위 |
|---:|---|---|
| 0 | 운용구역 X 위치 | `2(x-xmin)/(xmax-xmin)-1` |
| 1 | 운용구역 Y 위치 | `2(y-ymin)/(ymax-ymin)-1` |
| 2 | 지상고 | `AGL/300` |
| 3 | 수평속도 | `speed/72` |
| 4 | 기수방향 sin | `sin(ψ)` |
| 5 | 기수방향 cos | `cos(ψ)` |
| 6 | 수직속도 | `vertical_rate/8.9` |
| 7 | 방위각 변화율 | `yaw_rate/0.08726646`; 분모는 5 deg/s |
| 8 | 롤 | `roll/0.52359878`; 분모는 30 deg |
| 9 | 피치 | `pitch/0.52359878`; 분모는 30 deg |
| 10 | 기체 전방 목표거리 | `forward/S` |
| 11 | 기체 우측 목표거리 | `right/S` |
| 12 | 목표 고도차 | `dz/300` |
| 13 | 수평 목표거리 | `d_xy/S` |
| 14 | 목표 방위오차 sin | `sin(bearing_error)` |
| 15 | 목표 방위오차 cos | `cos(bearing_error)` |
| 16 | 목표 접근도 | `1-d_xy/S` |
| 17 | 임무 경과비율 | `elapsed_s/320` |
| 18 | 현재 엄폐도 | DEM 엄폐 분석값 `[0,1]` |
| 19 | 현재 안전 마스크 | 안전영역이면 1, 아니면 0 |
| 20 | 현재 경로 적합도 | 엄폐·지형 기반 적합도 `[0,1]` |
| 21 | 현재 적 탐지위험 | `0.4×기대위험 + 0.6×CVaR 위험` |
| 22 | 현재 노출비율 | `노출된 적 가설 수/max(위협범위 내 가설 수,1)` |
| 23 | 최근접 예상 적 거리 | `nearest_enemy_m/9000` |
| 24 | 운용구역 경계 여유 | 부호 있는 최근접 경계거리(m) `/1000` |
| 25 | 현재 지형경사 | `slope_deg/45` |
| 26 | 목표 지상고 | `target_AGL/300`; 경사 5 deg 이하 180 m, 그 외 110 m |
| 27 | 목표 지상고 오차 | `(AGL-target_AGL)/300` |
| 28 | A* 기준경로 횡방향 오차 | 기준 폴리라인까지 최단거리(m) `/1000` |
| 29 | A* 기준경로 진행률 | 최근접 투영점의 누적 진행률 `[0,1]` |
| 30 | A* 900 m 전방점의 기체 전방성분 | `lookahead_forward/900` |
| 31 | A* 900 m 전방점의 기체 우측성분 | `lookahead_right/900` |
| 32 | 직전 방위각 명령 | 직전 행동 `action[0]` |
| 33 | 직전 수직속도 명령 | 직전 행동 `action[1]` |
| 34 | A* 기준고도 오차 | `(baseline_z-current_z)/300` |
| 35 | 전방 지형 요구상승률 | `required_climb_rate/5` |
| 36 | 전방 최소 예상 지상고 여유 | `(lookahead_min_AGL-50-20)/300` |
| 37 | 정책 결정 간격 | `2(dt-1)/(5-1)-1` |

인덱스 37은 `dt=1,2,3,4,5 s`에 각각 `-1,-0.5,0,0.5,1`을 입력한다. 학습되지 않은 다른 시간 간격을 임의로 사용하지 않는다.

### 전방 지형·엄폐 fan: 인덱스 38~67

지형 fan은 거리 바깥 반복, 상대방위 안쪽 반복 순서이다. 각 방향마다 두 값이 연속해서 들어간다.

- `terrain_delta = (현재점부터 해당 거리까지의 최대 DEM 고도 - 현재 DEM 고도)/500`
- `cover_at_end = 해당 fan 끝점의 엄폐도 [0,1]`

| 인덱스 | 거리(m) | 기수 기준 상대방위 | 값 |
|---:|---:|---:|---|
| 38, 39 | 150 | -90 deg | `terrain_delta`, `cover_at_end` |
| 40, 41 | 150 | -45 deg | `terrain_delta`, `cover_at_end` |
| 42, 43 | 150 | 0 deg | `terrain_delta`, `cover_at_end` |
| 44, 45 | 150 | +45 deg | `terrain_delta`, `cover_at_end` |
| 46, 47 | 150 | +90 deg | `terrain_delta`, `cover_at_end` |
| 48, 49 | 450 | -90 deg | `terrain_delta`, `cover_at_end` |
| 50, 51 | 450 | -45 deg | `terrain_delta`, `cover_at_end` |
| 52, 53 | 450 | 0 deg | `terrain_delta`, `cover_at_end` |
| 54, 55 | 450 | +45 deg | `terrain_delta`, `cover_at_end` |
| 56, 57 | 450 | +90 deg | `terrain_delta`, `cover_at_end` |
| 58, 59 | 900 | -90 deg | `terrain_delta`, `cover_at_end` |
| 60, 61 | 900 | -45 deg | `terrain_delta`, `cover_at_end` |
| 62, 63 | 900 | 0 deg | `terrain_delta`, `cover_at_end` |
| 64, 65 | 900 | +45 deg | `terrain_delta`, `cover_at_end` |
| 66, 67 | 900 | +90 deg | `terrain_delta`, `cover_at_end` |

## 6. 모델 출력: 2차원 `float32` 행동

행동공간은 `Box(low=-1, high=1, shape=(2,), dtype=float32)`이다. 속도 명령은 출력하지 않는다.

| 인덱스 | 출력 | 물리량 변환 |
|---:|---|---|
| 0 | 방위각 변화 명령 | `yaw_rate = action[0] × feasible_yaw_limit`; 50 m/s에서는 최대 약 ±5 deg/s |
| 1 | 수직속도 명령 | 양수는 `action[1] × 5 m/s`, 음수도 `action[1] × 5 m/s`; 결과 범위 약 ±5 m/s |

현재 설정에서는 수평속도를 50 m/s로 유지한다. 한 번 계산한 행동은 선택한 정책 결정 간격 `dt` 동안 유지하되, 기동 상태는 최대 1 s 간격으로 적분한다. 목표 도달은 수평거리 90 m 이하이면서 고도오차 35 m 이하일 때로 정의한다. 한 구간의 시간 제한은 320 s이다.

## 7. 시스템 수준 경로 출력

PPO 직접 출력은 행동 2개뿐이다. 다음 절차로 행동을 반복 적용해야 3차원 경로가 만들어진다.

1. 현재 상태로 68차원 관측 생성
2. `model.predict(..., deterministic=True)` 실행
3. 행동을 물리량으로 변환하고 상태 적분
4. 새 위치에서 관측을 갱신
5. 목표 도달 또는 320 s까지 반복
6. 경유점이 있으면 구간별로 반복
7. 필요 시 수평좌표에 한 번만 `0.25×이전점 + 0.50×현재점 + 0.25×다음점` 평활화 적용

현재 빠른 경로 모드에서 평활화는 X/Y에만 적용하며 PPO가 만든 Z 고도는 바꾸지 않는다. 출력 시간 간격을 1 s로 선택하면 50 m/s 기준 경로점 간격은 대략 50 m이다. 5 m 간격을 사용하려면 출력 시간 간격을 비우고 별도의 거리 기반 재표본화를 적용해야 한다.

권장 경로점 최소 필드는 다음과 같다.

```text
x_m, y_m, z_msl_m, time_s, heading, vertical_rate_mps
```

## 8. 무결성 확인

| 파일 | 크기 | SHA-256 |
|---|---:|---|
| `model.zip` | 2,049,099 byte | `948AB508C6D43AF17FAD8F346A87980AE1632C3A9388904E064D84892E6DE530` |
| `inference_config.json` | 3,968 byte | `48621AE5E6F01AA40871732615AAD6811B45E227ED6431E4029FD8BDE0680E75` |

이 모델은 시뮬레이션 기반 경로 후보 생성용이다. 이식 시스템에서 실제 운용 경로로 사용하려면 별도의 DEM 충돌, 비행영역, 기체 제약조건 검증 절차를 적용해야 한다.
