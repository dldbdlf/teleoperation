# 로드맵

> 작성일 2026-11-02 · 상태: **초안** · 주차 기준: W1 = 2026-11-02 주
>
> 일정은 예상치다. 각 Gate의 결과에 따라 다음 단계의 범위와 순서를 바꾸며, 바꿀 때는 [decision.md](decision.md)에 기록한다.
> 알고리즘 설명: [docs/01_literature_novelty_audit.md](docs/01_literature_novelty_audit.md) Part C–E

---

## 한눈에 보기

```
W1  W2  W3  W4  W5  W6  W7  W8  W9  W10 W11 W12  W13~
[P0]                                                     확인·정리
[ P1 ]                                                   Sim 기구학 뼈대
    [   P2 (Sim oracle 실험)  ]                          ── Gate G1
[ V: 카메라·관절 추정·정답 rig (병행)  ]
                [   P3 (실데이터 E1·E2)  ]               ── Gate G2
                            [    P4 (Solver·Benchmark)    ]
                                        [   P5 (실기)        ]
                                                    [ P6 논문 ]
```

| 트랙 | 제안 담당 | 확정 담당 |
|---|---|---|
| 기구학 · Solver · 시뮬레이션 · NERO 제어 (P1, P2, P4, P5) | goeka138 | (미정) |
| Vision · 데이터 · 불확실성 모델 (V, P3) | yuil | (미정) |

---

## Phase 0. 확인·정리 (W1)

나머지 일정이 이 확인 결과에 따라 달라진다.

| # | 할 일 | 산출물 |
|---|---|---|
| 0.1 | 역할 확정 | `decision.md` 기록 |
| 0.2 | AprilTag 프로토타입 코드 커밋 (`headam/` 또는 fork) | 커밋 링크 |
| 0.3 | Biomimetics 2026 Hybrid IK 원문 확인 (신규성 최대 위험) | 01 문서 Part A/B ⚠️ 해소 |
| 0.4 | RGB-D 카메라 모델, 추가 카메라 확보 가능 여부 | `docs/hardware_inventory.md` |
| 0.5 | NERO 관절 위치 명령 경로(pyAgxArm / ROS2 controller)와 수신 주기 | 동일 |
| 0.6 | TRIDOT 장갑 모델·출력 형식, 사용 여부 | 동일 |
| 0.7 | 사람 영상 수집에 IRB가 필요한지 학교에 확인 | `decision.md` 기록 |

---

## Phase 1. Sim 기구학 뼈대 (W1–W2)

카메라 없이 진행한다. ROS 없이 Python(Pinocchio)으로 개발한다.

| # | 할 일 | 완료 기준 |
|---|---|---|
| 1.1 | 공식 URDF(`agilexrobotics/agx_arm_urdf`) 로드, FK | URDF FK 동작, 링크 길이 0.310 / 0.270 m 재확인 |
| 1.2 | **SRS 해석적 IK**: (손목 pose, ψ) → 관절각 7개 | 무작위 관절각 → FK → IK 복원 오차 ≈ 0 |
| 1.3 | 관절 한계로부터 **ψ feasible 구간** 계산 | 구간 내부 ψ는 모두 한계 만족, 외부는 위반 (테스트) |
| 1.4 | **손목 실행 가능성 분석**: J6 범위(−0.733 ~ 0.873 rad) 때문에 손목 자세가 불가능한 비율 | 결과 그림·표 |
| 1.5 | 시각화 (MuJoCo 또는 meshcat) | 궤적 재생 가능 |

산출물: `share/sim/kinematics.py` + 테스트, `docs/02_kinematics_validation.md` (시뮬레이션 부분)

---

## Phase 2. Sim oracle 실험 (W2–W4) → Gate G1

정답 동작과 정답 불확실성 Σ를 알고 있는 조건에서 solver 자체의 가치를 확인한다.

| # | 할 일 |
|---|---|
| 2.1 | 가짜 사람 팔 궤적: 도달, **ψ sweep**, **팔 펴기**, **J6 한계 초과 손목**, 가림 구간 |
| 2.2 | 사람 → 로봇 매핑: 몸통 프레임, NERO 가상 어깨, 팔 길이 비율 |
| 2.3 | 노이즈 모델: 깊이 방향 큰 노이즈, 가림 시 노이즈 증가·관측 결손. 정답 Σ 보존 |
| 2.4 | 의도 변수 변환 + 불확실성 전파 (ψ의 von Mises κ 포함) |
| 2.5 | Feasible-MAP solver (oracle Σ 입력) |
| 2.6 | Baseline: B1 손목 IK, B2 SEW 방향 추종, B3 수동 deadband (2610.16855 방식), B4 B1 + 필터 |
| 2.7 | 지표: 손목 오차, ψ 오차(elbow offset 구간별), jerk, 관절 한계 위반, 가림 후 불연속 |

**Gate G1 (W4):** oracle Σ 조건에서 Feasible-MAP가 팔 펴기 / 가림 / J6 한계 장면에서 손목 정확도를 해치지 않으면서 ψ 오차·jerk·불연속을 개선하는가?
- **Go** → P3, P4 진행
- **No-Go** → C1 재검토 (C2 / C3 전환 검토), `decision.md` 기록

---

## Track V. 카메라·관절 추정·정답 rig (W1–W5, 병행)

| # | 할 일 | 비고 |
|---|---|---|
| V.1 | RGB-D 카메라 연결 | WSL은 USB 직접 접근 불가 → `usbipd` 또는 Windows 측 처리 결정 |
| V.2 | 3D 관절 추정 파이프라인 (MediaPipe + depth) | S, E, W, 손 landmark + confidence + 시각 |
| V.3 | 몸통 프레임 계산 (매 프레임 갱신 vs 초기 고정 비교) | |
| V.4 | 로깅 형식 확정 (입력, 중간값, 시각) | 영상은 git 밖 공유 드라이브 |
| V.5 | **정답 측정 rig 설계·구축**: 다중 시점 카메라 + AprilTag (상완·전완·손등) | 프로토타입 tag 검출 재활용 |
| V.6 | 정답 rig 정확도 검증 (팔 길이 분산, 기지 길이 물체) | 이 정확도가 이후 모든 결과의 바닥 |

---

## Phase 3. 실데이터 E1·E2 (W4–W6) → Gate G2

IRB 확인(0.7)이 끝나야 시작한다.

| # | 할 일 |
|---|---|
| 3.1 | 파일럿 데이터 수집: 피험자 4–6명, 1인당 10–15분, P2와 같은 동작 세트 |
| 3.2 | 데이터셋 버전 확정, **피험자 단위 분할 파일 커밋** |
| 3.3 | **E1**: 단일 시점 ψ 오차가 elbow offset·가림에 따라 커지는가 |
| 3.4 | 실제 노이즈 통계와 P2 노이즈 모델 비교 → 필요하면 P2 재실행 |
| 3.5 | 불확실성 네트워크 학습 (heteroscedastic NLL) |
| 3.6 | **E2**: 예측 Σ의 coverage, 예측 σ와 실제 오차의 상관 |

**Gate G2 (W6):** 01 문서 Part E.2의 기준 (coverage 목표 ±10%p, Spearman ρ > 0.3 — 제안값, 데이터 확보 후 조정 가능)
- **Go** → 학습 Σ로 P4
- **No-Go** → C1 전제 붕괴. C2 / C3 전환, 결과도 기록

---

## Phase 4. Solver 완성·Benchmark (W6–W10)

| # | 할 일 |
|---|---|
| 4.1 | 학습 Σ를 쓰는 Feasible-MAP (실데이터 재생) |
| 4.2 | 추가 baseline: B5 IK + residual MLP/GRU, B6 learned ψ target |
| 4.3 | Ablation: 불확실성 제거, Mahalanobis → Euclidean, 해석적 전파 제거, temporal term 제거, ψ term 제거 |
| 4.4 | **오차 분리**: Vision / Embodiment / Numerical (oracle substitution) |
| 4.5 | solver 계산 시간 (평균 / P95 / P99) |

산출물: `docs/03_baseline_failure_analysis.md`, `docs/04_proposed_solver.md`, `docs/05_benchmark_results.md`

---

## Phase 5. 실기 (W8–W12)

| # | 할 일 | 완료 기준 |
|---|---|---|
| 5.1 | NERO 관절 위치 명령 경로 구현 (MoveIt Servo 대체) | 저속 단일 관절 명령 확인 |
| 5.2 | **안전 레이어 한 곳으로 통합** (관절·속도·작업공간·충돌·통신 끊김) | 두 사람 승인 |
| 5.3 | 녹화 데이터 시뮬레이션 재생 테스트 | 위반 0건 |
| 5.4 | 실기 FK 검증 (실측 관절 vs URDF FK) | `02_kinematics_validation.md` 보완 |
| 5.5 | 실시간 end-to-end 지연 측정 | 평균 / P95 / P99 |
| 5.6 | 조작 과제 3–5종 정의, 사용자 연구 파일럿 | 학습에 쓰지 않은 사용자 포함 |
| 5.7 | Execution 오차 측정 (명령 vs 실측) | 오차 분리 완성 |
| 5.8 | REVO2 Pro 통합 (시스템 데모용, 본 Contribution과 별개) | 손 6구동기 매핑 동작 |

산출물: `docs/06_real_robot_evaluation.md`

---

## Phase 6. 논문 (W12~)

| # | 할 일 |
|---|---|
| 6.1 | 게재처별 적합성 재평가 (`docs/07_publication_assessment.md`) |
| 6.2 | 목표 학회·저널 마감일 확인 후 역산 일정 확정 |
| 6.3 | 본 사용자 연구 (8–12명, 효과 크기·분산 근거로 규모 확정) |
| 6.4 | 코드·데이터 공개 범위 결정 (피험자 동의 범위 내) |

---

## Gate 요약

| Gate | 시점 | 질문 | No-Go 시 |
|---|---|---|---|
| G1 | W4 | 정답 Σ를 알 때 Feasible-MAP가 이득이 있는가 | C1 재검토, C2 / C3 검토 |
| G2 | W6 | 학습한 Σ가 실제 오차를 반영하는가 | C1 전제 붕괴, 전환 |
| G3 | W10 | Benchmark에서 baseline 대비 개선이 재현되는가 | 범위 축소 (조건 한정) 또는 전환 |
| G4 | W12 | 실기에서 안전하고 실시간으로 동작하는가 | 실기 범위 축소, 시뮬레이션 결과 중심 |

## 주요 위험

| 위험 | 영향 | 대응 |
|---|---|---|
| Biomimetics 논문이 같은 아이디어를 포함 | 신규성 상실 | 0.3에서 최우선 확인 |
| 정답 rig 정확도 부족 | E1·E2 결과 신뢰 불가 | V.6에서 먼저 검증, 부족하면 AprilTag 보강 |
| IRB 지연 | P3 지연 | P2를 공개 모션 데이터셋으로 확장해 대기 |
| WSL 카메라 연결 문제 | V 지연 | Windows 측 처리 경로 확보 |
| "confidence-weighted IK와 같다"는 리뷰 | 신규성 약화 | 해석적 전파 vs 직접 가중 ablation (4.3) |
