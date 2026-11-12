# share/sim — NERO 기구학 (Roadmap Phase 1)

ROS 없이 Python(numpy)만으로 돈다. 결과 해석: [../docs/02_kinematics_validation.md](../docs/02_kinematics_validation.md)

| 파일 | 내용 | Roadmap |
|---|---|---|
| `kinematics.py` | URDF FK, SEW 각 ψ로 매개화한 SRS 해석적 IK (최대 8개 해), 관절 한계 아래 ψ feasible 구간 | 1.1–1.3 |
| `analysis_wrist_feasibility.py` | 손목 자세 실행 가능 비율과 원인 관절 분석 → `results/` | 1.4 |
| `tests/test_kinematics.py` | URDF 체인 FK 대조, IK 왕복·완전성, feasible 구간 안/밖 검사 | 1.1–1.3 |
| `synthetic_human.py` | EX04/G1용 가짜 사람 궤적 + 관측 + oracle belief 생성 (S1–S6) | P1 / EX04 |
| `tests/test_synthetic_human.py` | 생성 궤적 기하(팔 길이, ψ 왕복), 가림 마스크, S3/S6 조건, belief calibration | P1 / EX04 |
| `assets/nero_description.urdf` | 공식 URDF 사본 (`agilexrobotics/agx_arm_urdf` @ `680fa92`, MIT, `assets/LICENSE.agx_arm_urdf`) | — |

```bash
pip install numpy pytest matplotlib
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

## 규약

- 도구 프레임 = `link7`. 단위 m, rad.
- NERO의 "앞"은 **−x** (ready 자세에서 도구가 x = −0.30 m).
- ψ = stereographic SEW 각 (Elias & Wen, arXiv 2307.13122). 특이 방향 −z(베이스 아래), 기준 방향 −x.
  앞으로 뻗었을 때 ψ = 0 이 팔꿈치 아래, +90° 가 팔꿈치 −y 쪽. ready 자세는 ψ = ±180° (팔꿈치 위).
- 시각화(1.5)는 아직 없다.
