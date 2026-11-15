# EX00 — 공통 실험 계약 (Observation / HumanBelief / SolverCommand)

실험 범위: [`docs/experiment_roadmap.md`](../docs/experiment_roadmap.md) EX00.

**목적:** Track A(인식, D415 markerless/GT)와 Track B(솔버, `share/sim/`)가 서로 다른 입력
가정을 쓰지 않도록, 둘 사이를 오가는 데이터의 필드 이름·단위·좌표계·shape를 고정한다. 이 문서는
`share/sim/synthetic_human.py`가 **이미 실제로 생성하는** 필드를 그대로 공식화한 것이다 (새로
발명한 포맷이 아니라, 기존 Track B 코드를 Track A와 맞물릴 수 있게 명문화).

## 1. Observation (Track A가 생산, `obs_*`)

카메라 1프레임(15Hz, D415 기준)당 1개. `synthetic_human.py`가 생성하는 합성 버전과, 실제
EX02(`share/ex02_markerless/`)가 생산해야 할 실데이터 버전을 같이 명시한다.

| 필드 | 타입/shape | 단위/좌표계 | 비고 |
|---|---|---|---|
| `obs_t` | float64, (N,) | 초(s), 에피소드 시작 기준 | 합성은 `np.arange`, 실데이터는 `image_timestamp_ns / 1e9` |
| `obs_W` | float64, (N,3) | m, NERO base(합성) / 카메라→base 보정 후(실데이터) | 손목 위치, 가려지면 NaN |
| `obs_R` | float64, (N,3,3) | SO(3) 회전행렬 | 손목 orientation, 가려지면 NaN |
| `obs_E` | float64, (N,3) | m | 팔꿈치 위치, 가려지면 NaN |
| `obs_psi` | float64, (N,) | rad, `[-pi, pi)` wrap | SEW 트위스트 각, 가려지면 NaN |
| `obs_wrist_visible` | bool, (N,) | - | |
| `obs_elbow_visible` | bool, (N,) | - | `wrist_visible=False`면 반드시 `elbow_visible=False`도 성립(손목 가림 시 팔꿈치도 가림 처리) |

**실데이터(EX02)가 추가로 채워야 하는 필드** (현재 markerless 파이프라인엔 아직 없음):
`frame_id`, `landmark_confidence`(mediapipe visibility), `depth_validity`(ROI valid_ratio),
`torso_transform`, `sequence_id`, `subject_id`(비식별).

## 2. HumanBelief (Track A가 생산, `belief_*`)

| 필드 | 타입/shape | 단위 | 비고 |
|---|---|---|---|
| `belief_t` | float64, (N,) | s | `obs_t`와 동일 타임라인 |
| `belief_wrist_T` | float64, (N,4,4) | 동차변환, m | |
| `belief_W` | float64, (N,3) | m | 손목 평균 위치 |
| `belief_wrist_cov` | float64, (N,6,6) | tangent-space 공분산 | 대칭·양semi-definite 이어야 함 |
| `belief_psi_mean` | float64, (N,) | rad | |
| `belief_psi_sigma` | float64, (N,) | rad | von Mises 근사 표준편차 |
| `belief_psi_kappa` | float64, (N,) | - | `1/sigma^2` |
| `belief_occlusion_state` | int64, (N,) | 0=VISIBLE/1=ELBOW_HIDDEN/2=WRIST_HIDDEN | `synthetic_human.py`의 `VISIBLE/ELBOW_HIDDEN/WRIST_HIDDEN` 상수와 일치해야 함 |
| `belief_age_wrist_ms` | float64, (N,) | ms, >=0 | 마지막으로 보인 뒤 경과 시간 |
| `belief_age_elbow_ms` | float64, (N,) | ms, >=0 | |

합성 데이터는 **oracle belief**(진짜 오차를 sigma(t)로 그대로 보고)이고 학습된 belief가 아님 -
G1 상한선 취급, 실제 belief 성능으로 주장 금지(`synthetic_human.py` 주석 참고).

## 3. SolverCommand (Track B가 생산, `run_baselines.py`의 `Q`/`Qreq`/`held`)

| 필드 | 타입/shape | 단위 | 비고 |
|---|---|---|---|
| `q`(`Q`) | float64, (N,7) | rad, shield 통과 후 | 실제 명령, velocity-cap(`qd_max`) 적용됨 |
| `q_target`(`Qreq`) | float64, (N,7) | rad, shield 통과 전 | 방법(B0-B8/P)이 요청한 원 목표 |
| `held` | bool, (N,) | - | `True`면 이번 틱은 HOLD(새 목표 대신 이전 q 유지) - NaN 입력/age>hold_s일 때 |
| `status` | (문서상 정의, 코드엔 아직 별도 필드 없음) | TRACK/HOLD/SAFE_STOP | `held=True`→HOLD, 나머지→TRACK. SAFE_STOP은 아직 시뮬레이션 코드에 없음(EX08에서 추가 필요) |
| `t_ctrl`(`gt_t`) | float64, (N,) | s, 100Hz 제어주기 | 카메라(15Hz)보다 빠른 제어 루프 |

**갭**: `status`에 SAFE_STOP이 없고, `reason`(왜 HOLD/SAFE_STOP인지) 필드도 없음 - EX08(실기
안전검증) 전에 추가해야 함.

## 4. Dataset manifest (`synthetic_human.py`가 `manifest.json`으로 이미 생성)

```json
{
  "git_sha": "...", "urdf": "nero_description.urdf", "reach_J4_m": [...],
  "config": {...}, "args": {...},
  "occlusion_state_codes": {"visible": 0, "elbow_hidden": 1, "wrist_hidden": 2},
  "files": [{"file": "...", "scenario": "...", "seed": 0, "index": 0, ...}, ...]
}
```

실데이터(EX02/EX05)는 여기에 `calibration_sha`(extrinsic 계산에 쓴 커밋), `sensor_serial`,
`consent_status`, `subject_id`(비식별)를 추가로 넣어야 한다 - 아직 실데이터 매니페스트 생성기는 없음.

## 5. 검증 (`validate_ex00_contract.py`)

```bash
python3 validate_ex00_contract.py <npz 파일 또는 디렉토리>
```

필드 존재/shape/dtype, 타임스탬프 단조증가, `psi` 범위, 가림 시 NaN 일관성, `wrist_cov`
대칭·반정치, 회전행렬 직교성, `age_*_ms` 비음수를 검사한다. 통과 기준(로드맵 EX00 "통과" 조건):
합성 샘플 1개가 모든 검사를 통과하면 A→B 데이터 전송 계약이 성립한 것으로 간주.
