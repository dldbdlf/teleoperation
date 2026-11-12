# share/sim — NERO 기구학 (Roadmap Phase 1)

ROS 없이 Python(numpy)만으로 돈다. 결과 해석: [../docs/02_kinematics_validation.md](../docs/02_kinematics_validation.md)

| 파일 | 내용 | Roadmap |
|---|---|---|
| `kinematics.py` | URDF FK, SEW 각 ψ로 매개화한 SRS 해석적 IK (최대 8개 해), 관절 한계 아래 ψ feasible 구간 | 1.1–1.3 |
| `analysis_wrist_feasibility.py` | 손목 자세 실행 가능 비율과 원인 관절 분석 → `results/` | 1.4 |
| `tests/test_kinematics.py` | URDF 체인 FK 대조, IK 왕복·완전성, feasible 구간 안/밖 검사 | 1.1–1.3 |
| `synthetic_human.py` | EX04/G1용 가짜 사람 궤적 + 관측 + oracle belief 생성 (S1–S6) | P1 / EX04 |
| `tests/test_synthetic_human.py` | 생성 궤적 기하(팔 길이, ψ 왕복), 가림 마스크, S3/S6 조건, belief calibration | P1 / EX04 |
| `baselines.py` | EX04 비교군 B0–B8 (공통 IK, ψ 매핑 포함) | P1 / EX04 |
| `run_baselines.py` | 공통 하네스(100 Hz 제어, 안전 장치)로 B0–B8 실행 + 지표 → `outputs/` | P1 / EX04 |
| `tests/test_baselines.py` | Jacobian, ψ 매핑, SEW-Mimic 정확 재현, AEAC 수렴·해제, AKC 보정, B7, 하네스 안전 제약 | P1 / EX04 |
| `assets/nero_description.urdf` | 공식 URDF 사본 (`agilexrobotics/agx_arm_urdf` @ `680fa92`, MIT, `assets/LICENSE.agx_arm_urdf`) | — |

```bash
pip install numpy scipy pytest matplotlib jparse-robotics OneEuroFilter
cd share/sim
python -m pytest -q tests
python analysis_wrist_feasibility.py --n 1000 --grid 180   # 16코어 기준 약 10분
```

## EX04 가짜 사람 궤적 (`synthetic_human.py`)

```bash
python synthetic_human.py --n-traj 20 --seeds 0 1 2 --plot   # outputs/synthetic/ (git 제외)
```

궤적마다 `.npz` 1개. 모두 NERO base 좌표, 사람 팔은 로봇 팔 길이로 이미 매핑됐다고 가정한다.

| 키 접두어 | 주기 | 내용 | 쓰는 쪽 |
|---|---|---|---|
| `gt_` | 100 Hz | 정답 link7 목표 `T`, 손목 `W`, ψ, 팔꿈치 `E`, `psi_defined` | 평가 |
| `cam_gt_` | 15 Hz | 같은 정답을 카메라 시각에 샘플 + `psi_feasible`(정답 ψ로 IK 가능), `wrist_feasible`(어떤 ψ로든 가능, 36칸 격자) | 평가 |
| `obs_` | 15 Hz | 노이즈 낀 `W`, 손목 회전 `R`, `E`, 그로부터 계산한 ψ. 가려지면 NaN | B0–B4 |
| `belief_` | 15 Hz | oracle `HumanBelief`: 손목 평균 `wrist_T` + 6×6 공분산, ψ 평균·σ·κ, 가림 상태(0 보임, 1 팔꿈치 가림, 2 손목 가림), 가림 경과 ms | B7, P |

| 시나리오 | 내용 |
|---|---|
| S1 | 손목 ready 고정, ψ 한 바퀴 sweep |
| S2 | 손목을 J4 최대 도달 거리 근처까지 폈다가 복귀 (ψ 불안정 구간) |
| S3 | 손목 회전을 키워 모든 ψ에서 불가능한 자세를 지났다가 복귀 (생성 시 확인) |
| S4 | 팔꿈치만 가림 |
| S5 | 손목·팔꿈치 모두 가림 |
| S6 | 팔꿈치 가림 + 가림 도중 ψ 방향 반전 |

S4–S6은 `--dropouts`(기본 0.2/0.5/1.0/2.0 s)마다 생성. 같은 (시나리오, seed, 번호, dropout)은 항상 같은 궤적이므로 모든 baseline이 같은 파일을 읽으면 된다. `manifest.json`에 설정, git commit, 파일 목록을 남긴다.

**Oracle belief:** belief는 관측으로부터 계산하지 않는다. 오차를 `σ(t) × 단위분산 AR(1) 노이즈`로 뽑고 그 σ(t)를 그대로 보고하므로 공분산이 정의상 맞다 (테스트에서 2σ 안 비율 92–98% 확인). σ는 가려진 시간에 비례해 커진다. EX04의 "oracle Σ" 상한이며 학습된 belief 성능이 아니다.

**가정 (사람 실측 아님):** 움직임은 ready 손목 주변의 느린 사인파 합, ψ 중심은 원 전체 균등(`--psi-centre-deg`로 조정), ψ belief는 단봉(von Mises 1개). 노이즈 크기는 `Noise` 기본값(손목 15 mm, 회전 5°, 팔꿈치 20 mm)이며 D415 실측으로 바꿔야 한다.

## EX04 비교군 (`baselines.py`, `run_baselines.py`)

```bash
python synthetic_human.py --n-traj 10 --seeds 0 1 --dropouts 0.5 1.0 --psi-centre-deg -30 30 --out outputs/syn_down
python run_baselines.py outputs/syn_down --psi-map mirror --out outputs/baselines_mirror   # summary.md, per_traj.csv
```

| ID | 방법 | 출처 | 코드 |
|---|---|---|---|
| B0 | 손목 DLS IK + 기본 자세 복귀 | 표준 DLS | 재구현 |
| B1 | SEW 방향 정렬 (SEW-Mimic) | arXiv 2602.01632 Alg. 1–3 | 재구현 (공식 코드 "심사 후 공개", 제3자 구현은 라이선스 없음) |
| B2 | 적응형 팔꿈치 오프셋 AEAC + J-PARSE | arXiv 2610.16855 Sec. III | J-PARSE는 공식 `jparse-robotics`(MIT), 나머지 재구현 (공개 저장소는 프로젝트 페이지뿐) |
| B3 | B0 + One Euro 필터 | Casiez et al. CHI 2012 | `OneEuroFilter`(BSD, 저자 공식) |
| B4 | B0 + 등속 Kalman | 표준 CV KF | 재구현 |
| B5 | AKC 팔 길이 보정 + 공통 IK | arXiv 2606.19240 Sec. II-C/D | 재구현 (코드 없음) |
| B6 | 학습형 ψ (손목 자세 → ψ MLP) + 공통 IK | Biomimetics 11(6):408, Polimi 2019 | 재구현 (코드 없음, 본문 접근 불가라 망 구조는 가정) |
| B7 | 단봉 불확실성 가중 제약 IK | 로드맵 정의 | 재구현 |
| B8 | belief MAP 모드 → 일반 제약 IK (순차식) | 로드맵 정의 | 재구현 |

- **공통 조건:** 15 Hz 관측(가려지면 마지막 값 유지), 100 Hz 제어, q0 = READY, 안전 장치(관절 한계, 관절 속도 ≤ π rad/s, 손목 입력이 0.5 s보다 오래되면 정지). 사람 ψ 매핑(`--psi-map`: none/neg/flip/mirror/offset)은 모든 방법의 팔꿈치·ψ 입력에 똑같이 적용.
- **공통 IK (B5, B6, B8):** 요청 ψ에서 해석적 IK → 불가능하면 2° 격자에서 가장 가까운 가능 ψ → 해 중 현재 q에 가장 가까운 것 → 손목 자체가 불가능하면 정지.
- **출력 보간:** 프레임마다 목표를 한 번 정하는 방법(B1, B5–B8)은 새 목표까지 카메라 한 주기(약 66 ms) 동안 선형 보간한다 (2610.16855의 출력 보간층과 같은 역할). 15 Hz 계단 입력 때문에 jerk가 부풀려지는 것을 막기 위해서다. `--no-interp`로 끈다.
- **지표:** 손목 위치·자세 오차(100 Hz, P50/P95), 목표 ψ 가능 비율, 가능 구간 ψ 오차 P90, 불가능 구간 regret(가장 가까운 가능 ψ와의 차이), ψ 점프(0.2 s 안에 60° 넘게 바뀐 횟수), 관절 jerk RMS(100 Hz 관절각의 3차 차분, rad/s³, 이 하네스 안에서만 비교), 정지 비율, 계산 시간 P95.
- **복귀 지표 (가림 종료 후 1 s):** 안전 장치 전 요청 점프의 최대값(°), 관절 이동 경로 합(°), 속도 제한에 걸린 tick 비율(%). 최대 관절 속도는 대부분 속도 제한값에 붙어서 쓰지 않는다.
- **알려진 실패 (버그 아님, S1 ψ 한 바퀴):** B2는 사람 ψ가 불가능 구간을 지나면 AEAC가 팔꿈치를 관절 한계로 밀고, 적분 후 clip(논문 식 4) 때문에 손목 우선순위가 깨져 손목 오차가 수백 mm까지 커진다. B7은 국소 최적화라 불가능 구간 반대편으로 넘어가지 못하고 갇힌다(목적함수 값이 반대편 해보다 3배 이상 높은 지역 최소). 둘 다 가능 범위를 모르는 방법의 한계다.
- **논문에 없어 정한 값**은 `baselines.py` docstring에 `UNTUNED`/`ASSUMPTION`으로 표시했다. 테스트 데이터로 튜닝하지 않았다. 예외로 B2의 k_e는 정지 자세 하나에서 60° 팔꿈치 이동이 약 0.6 s에 수렴하도록 골랐다.
- **B6 학습 데이터:** 시험 seed와 겹치지 않는 seed 1000의 S4 궤적 60개(정답 손목 자세 → 매핑된 정답 ψ). 가짜 궤적에서는 ψ가 손목 자세와 독립으로 뽑히므로 B6는 사실상 평균 ψ를 배운다. 실제 사람 데이터에서 다시 학습해야 의미가 있다.
- **B8:** oracle belief가 단봉이라 MAP 모드 = 평균이다. 다봉 belief가 생기면 가장 무거운 모드를 쓴다(`b_psi_modes`).

## 규약

- 도구 프레임 = `link7`. 단위 m, rad.
- NERO의 "앞"은 **−x** (ready 자세에서 도구가 x = −0.30 m).
- ψ = stereographic SEW 각 (Elias & Wen, arXiv 2307.13122). 특이 방향 −z(베이스 아래), 기준 방향 −x.
  앞으로 뻗었을 때 ψ = 0 이 팔꿈치 아래, +90° 가 팔꿈치 −y 쪽. ready 자세는 ψ = ±180° (팔꿈치 위).
- 시각화(1.5)는 아직 없다.
