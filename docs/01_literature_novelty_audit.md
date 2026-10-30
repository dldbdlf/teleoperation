# 01. Literature Review & Novelty Audit

> Stage 1 산출물 · 작성일 2026-10-29 · 상태: **초안 (실험 결과 없음)**
>
> 이 문서의 모든 "신규성"·"게재 가능성" 판단은 **가설**이다. 실험으로 입증되기 전까지 확정적으로 인용하지 않는다.

**검증 수준 표기**

| 표기 | 의미 |
|---|---|
| ✅ | 원문(arXiv HTML 본문)을 직접 확인 |
| 🟡 | 초록 / 공식 프로젝트 페이지 / 학회 proceedings 페이지만 확인 |
| ⚠️ | 원문 접근 실패, 검색 요약만 확인 → **인용 전 반드시 원문 재확인** |

---

## Part A. Verified Related Work

### A.1 지정된 10편

| # | 공식 제목 | 출판 정보 | 입력 | 기본 Solver | 학습 방법 | 보정 대상 | 검증 |
|---|---|---|---|---|---|---|---|
| 1 | DexPilot: Vision Based Teleoperation of Dexterous Robotic Hand-Arm System | ICRA 2020, pp. 9164–9170 · [arXiv:1910.03135](https://arxiv.org/abs/1910.03135) | 맨손 관찰 (depth 기반 다중 카메라) | 손: kinematic retargeting 최적화, 팔: 손목 pose 추종 | 손 pose 추정 네트워크 (retargeting 자체는 비학습) | 손가락 매핑 | 🟡 |
| 2 | ResPilot: Teleoperated Finger Gaiting via Gaussian Process Residual Learning | CoRL 2024, PMLR v270 · [arXiv:2409.09140](https://arxiv.org/abs/2409.09140) · [PMLR](https://proceedings.mlr.press/v270/naughton25a.html) | 손 keypoint | 최적화 기반 hand retargeting | **GP residual** (소수 hand-labeled calibration pose) | retargeter 출력 → 레이블 관절각 차이 | 🟡 |
| 3 | AnyTeleop: A General Vision-Based Dexterous Robot Arm-Hand Teleoperation System | RSS 2023 · DOI 10.15607/RSS.2023.XIX.015 · [arXiv:2307.04577](https://arxiv.org/abs/2307.04577) | RGB / RGB-D 카메라 | 최적화 기반 retargeting + 팔 motion generation | 손 pose 추정 (retargeting 비학습) | 범용성(다수 로봇·카메라) | 🟡 |
| 4 | **A Closed-Form Geometric Retargeting Solver for Upper Body Humanoid Robot Teleoperation** (SEW-Mimic) | arXiv preprint, 2026-02 · [arXiv:2602.01632](https://arxiv.org/abs/2602.01632) | S/E/W keypoint + 손 방향 (MediaPipe 또는 Quest) | Paden–Kahan subproblem 기반 **closed-form** 방향 정렬 | 없음 (retargeter 비학습) | 상완·전완·손 방향 정렬 | ✅ |
| 5 | A Hybrid Inverse Kinematics Framework for Biomimetic Redundancy Resolution in 7-DoF Humanoid Arms | Biomimetics 11(6):408, 2026 · [MDPI](https://www.mdpi.com/2313-7673/11/6/408) | (확인 필요) | 하이브리드 IK | **경량 NN: end-effector pose → operator-specific SEW angle** | 여유자유도(팔꿈치) | ⚠️ (403, 원문 미확인) |
| 6 | ReDexT: Learning a Generalizable Residual Policy for Dexterous Retargeting | 2026 (게재처 미확인) · [Project](https://aim-uofa.github.io/ReDexT/) · [GitHub](https://github.com/aim-uofa/ReDexT) | 사람 hand-object trajectory | offline IK | **RL residual policy** | 접촉·물체 동역학으로 인한 실행 오차 | 🟡 |
| 7 | Physical AI for Cross-Embodiment Human-to-Robot Motion Retargeting with Residual Deep Koopman Learning (CE-MR) | Actuators 15(10):529, 2026 · [MDPI](https://www.mdpi.com/2076-0825/15/10/529) | 착용형 IMU 2개 | posture-aware IK | **Residual Deep Koopman**: constant-velocity 예측 + residual (200 ms horizon) | 지연(latency) 보상용 사람 동작 예측 | ⚠️ (요약만) |
| 8 | Smooth Operator: A Real-Time Sampling-Based Algorithm for Kinematic Hand Retargeting | arXiv preprint, 2026-07 · [arXiv:2607.07491](https://arxiv.org/abs/2607.07491) | MANUS 장갑 | **gradient-free sampling-based retargeter** | 없음 | gradient 기반 retargeter의 local minima jitter | 🟡 |
| 9 | Geometric Retargeting: A Principled, Ultrafast Neural Hand Retargeting Algorithm (GeoRT) | arXiv 2025-03 · [arXiv:2503.07541](https://arxiv.org/abs/2503.07541) | 손가락 keypoint | 없음 (신경망이 직접 매핑) | 비지도 학습 MLP, 기하 목적함수 | 손 retargeting 자체 (1 kHz) | 🟡 |
| 10 | AnyDexRT: Calibration-Free Dexterous Hand Retargeting with Few-Shot Human Guidance | arXiv preprint, 2026-07 · [arXiv:2607.08341](https://arxiv.org/abs/2607.08341) · [Project](https://chenxi-wang.github.io/projects/anydexrt/) | 손 keypoint | 자기지도 shape matching | fingertip correspondence 학습 + contact classifier | 손 calibration 의존성 | 🟡 |

### A.2 추가 검색으로 찾은 근접 연구 (2024–2026)

| 제목 | 출판 | 핵심 | 우리와의 관련성 | 검증 |
|---|---|---|---|---|
| **Human-Like Redundancy Resolution for Real-Time Bimanual Humanoid Teleoperation** | arXiv:[2610.16855](https://arxiv.org/abs/2610.16855), 2026-10 | XR S/E/W keypoint → J-PARSE 손목 task + null-space에 **elbow-offset 크기로 gating된 팔꿈치 방향 cost (AEAC)**. 학습 없음. | 팔꿈치 신뢰도를 *휴리스틱*으로 다룸 → 우리 아이디어의 가장 가까운 비학습 버전 | ✅ |
| **Seeing Through Occlusion: Deterministic Arm Kinematic Correction for Robot Teleoperation** (AKC) | arXiv:[2606.19240](https://arxiv.org/abs/2606.19240), 2026-06 | 단일 RGB-D, 일정 bone length 제약으로 가려진 관절 깊이를 결정론적으로 복원, Vicon 대비 검증 | Vision 오차 보정(가림)을 이미 다룸 — 학습·불확실성은 없음 | 🟡 |
| Grounding Generative Policies in Physics: Optimization-Guided Diffusion for Robot Control | arXiv:[2606.24208](https://arxiv.org/abs/2606.24208), 2026-06 | IK로 레이블한 **neural reachability surrogate**가 infeasible target에 pose 보정 offset 출력 | "학습된 feasibility-aware target correction" 신규성을 잠식 | ⚠️ |
| MimicIK: Real-Time Generative IK from Teleoperation with FK Consistency | arXiv:[2606.15148](https://arxiv.org/abs/2606.15148), 2026-06 | 관절 변화량 예측 + differentiable FK consistency loss | neural IK 계열 baseline 후보 | ⚠️ |
| Learned swivel-angle mapping (Polimi 계열, IEEE Access 2019 / IJARS) | [UWE](https://uwe-repository.worktribe.com/OutputFile/4047244), [Polimi](https://re.public.polimi.it/handle/11311/1101012) | 사람 skeleton 데이터로 6D 목표 pose → swivel angle 학습, KUKA LWR4+ 원격조작 | **"learned null-space / swivel target" 은 이미 존재** | ⚠️ |
| Redundancy parameterization and IK of 7-DOF revolute manipulators (Elias & Wen) | arXiv:[2307.13122](https://arxiv.org/abs/2307.13122) | 일반화 / stereographic SEW angle, subproblem 기반 IK | NERO의 SEW 파라미터화 및 feasible swivel 구간 계산의 기반 | 🟡 |

**확인하지 못한 것 (조작하지 않음):** Biomimetics 논문의 입력 센서·로봇·정량 결과, CE-MR의 세부 수치, ReDexT의 게재처, SEW-Mimic 코드 공개 여부(논문에는 "after review" 공개 예정으로 기재).

---

## Part B. Exact Overlap and Research Gap

### B.1 가장 가까운 5편과 정확한 중복 요소

| 논문 | 실제 Contribution (논문 명시) | 논문이 인정한 한계 | 분석상 한계 (우리 판단) | **우리 아이디어와 겹치는 부분** |
|---|---|---|---|---|
| **SEW-Mimic** (2602.01632) | S/E/W 방향 정렬의 closed-form 해, ~0.6 ms, 팔꿈치 직접 제어 | joint limit·self-collision은 최적성 증명 밖; 정확도는 keypoint 품질에 의존; 사용자 연구 N=8 소규모 | keypoint 노이즈를 **모델링하지 않음**. 팔이 거의 펴지면(S-E-W 공선) 상완 방향이 불안정해지는데 이를 다루는 장치 없음. 위치(translation)는 맞추지 않음 | 어깨 기준 좌표계, S/E/W 기반 팔꿈치 보존, MediaPipe 입력 — **기본 retargeting 부분 전체** |
| **Human-Like Redundancy Resolution** (2610.16855) | 손목 우선 + null-space 팔꿈치 방향 cost, offset 크기 기반 adaptive weight, J-PARSE | circle trajectory에서 손목 오차가 XR-Teleop보다 큼; 양팔 독립 해결 | 신뢰도 = **elbow offset 크기에 대한 수동 deadband 램프**. 관측 노이즈·가림·깊이 오차를 반영하지 않음. XR tracker 입력이라 vision 노이즈 미검증 | "팔꿈치 정보를 얼마나 신뢰할 것인가"라는 문제의식, 손목 우선 · 팔꿈치 보조 우선순위 구조 |
| **Hybrid IK – Biomimetics** (2026) | NN이 EE pose → operator-specific SEW angle 학습 | ⚠️ 미확인 | (원문 확인 후 기재) | **Candidate B의 "learned null-space/swivel target"** — 이것만으로는 신규성 없음 |
| **ResPilot** (CoRL 2024) | 최적화 retargeter 위 GP residual, 소수 레이블 pose | (손 전용) | GP 분산은 *데이터 밀도* 불확실성(epistemic)이지 **관측 노이즈(aleatoric)**가 아님. 팔 여유자유도 문제는 다루지 않음 | **"IK + learned residual"** 구조 자체 (Candidate A) |
| **AKC** (2606.19240) | 단일 RGB-D, bone length 제약으로 가림 시 관절 깊이 복원, Vicon 검증 | (초록 수준만 확인) | 결정론적 보정 → 보정 결과가 얼마나 믿을 만한지 solver에 전달하지 않음 | 단일 RGB-D 가림 문제, bone-length 기반 vision 보정 |

### B.2 단순 조합 vs 독립적 Contribution 구분

다음은 **이미 존재하거나 단순 조합**으로 간주해야 한다.

- IK + residual MLP/GRU (ResPilot, ReDexT 계열) → Candidate A
- 학습된 swivel / null-space target (Polimi 2019, Biomimetics 2026) → Candidate B 단독
- confidence weighting, adaptive gain (2610.16855의 offset gating), One Euro / Kalman filtering
- bone-length 제약 기반 가림 보정 (AKC)
- 학습된 reachability surrogate로 infeasible target 보정 (2606.24208)
- 사람 동작 예측으로 지연 보상 (CE-MR)

### B.3 Research Gap (가설)

조사 범위 내에서 다음을 **함께** 다룬 팔 retargeting 연구는 찾지 못했다 (부재의 증거가 아니라 검색 범위 내 미발견).

> **G1.** 단일 시점 vision 추정의 *관측별(heteroscedastic) 불확실성*을 **로봇 retargeting에 의미 있는 변수 공간 (손목 SE(3), swivel angle ψ)** 으로 전파하는 방법. 특히 ψ의 불확실성은 elbow offset $h$가 작을수록 기하학적으로 발산하는데 (≈ $\sigma_{E\perp}/h$), 기존 연구는 이를 수동 deadband(2610.16855) 또는 무시(SEW-Mimic)로 처리한다.
>
> **G2.** 그 불확실성을 **로봇의 실행 가능 집합(joint limit, reachability, 속도 한계) 위의 제약 추정 문제**로 결합하는 것 — 즉 "도달 불가능한 목표를 어떻게 수정할 것인가"를 Euclidean 투영이 아니라 *관측 불확실성의 형태*에 따라 결정하는 것.
>
> **G3.** Vision / Embodiment / Numerical / Execution 오차를 **oracle substitution**으로 분리 측정한 팔 retargeting 벤치마크. (SEW-Mimic은 정렬 오차·속도만, 2610.16855는 XR 입력만 보고.)

**신규성 위험:** (i) Biomimetics 논문 원문에 불확실성 모델링이 포함되어 있을 가능성, (ii) 인간 자세추정 분야의 heteroscedastic keypoint uncertainty 자체는 잘 알려져 있음 → 우리의 기여는 "불확실성 추정"이 아니라 **"redundancy 공간으로의 전파 + 제약 MAP solver"** 여야 한다. 이 경계가 흐려지면 reviewer는 "confidence weighting"으로 평가할 것이다.

---

## Part C. Three Candidate Algorithm Contributions

### C1. Feasible-MAP Retargeting: Uncertainty-Propagated Redundancy Resolution
사람 의도 변수 $z=(p_w, R_w, \psi)$ 에 대한 **사후분포**를 학습하고, 로봇 실행 가능 집합 위에서 **제약 MAP 추정**으로 관절각을 구한다. ψ 분포의 집중도는 학습된 keypoint 공분산을 elbow-offset 기하로 *해석적으로 전파*해 얻는다 (휴리스틱 deadband 대체).

### C2. Amortized Intent-Preserving Embodiment Projection
오프라인 전역 최적화(비인과적, 전체 시퀀스 사용)로 "의도 보존" 로봇 궤적을 만들고, 이를 실시간 네트워크로 amortize하여 infeasible 목표를 보정.

### C3. Uncertainty-Gated Occlusion Recovery
가림 구간 동안 belief를 유지하고, 추적 복귀 시 불확실성 기반 trust-region으로 불연속 없이 재수렴.

### 비교

| 항목 | C1 Feasible-MAP | C2 Amortized Projection | C3 Occlusion Recovery |
|---|---|---|---|
| 신규성 | **중–상** (G1+G2 결합; 2610.16855의 원리적 일반화) | 중–하 (2606.24208, GeoRT류 amortization과 근접) | 하–중 (AKC, filtering과 근접) |
| 구현 난이도 | 중 (확률 head + 1-D/소규모 QP solver) | 상 (오프라인 전역 최적화 + 학습) | 하–중 |
| 데이터 요구 | 중: 기준 좌표가 있는 사람 데이터 (로봇 데이터 불필요) | 상: 다양한 목표에 대한 최적 레이블 대량 (시뮬레이션으로 생성 가능) | 중: 가림 시나리오 데이터 |
| 실험 가능성 | **높음**: 핵심 전제(E1·E2)를 로봇 없이 검증 가능 | 중 | 높음 |
| 실패 시 신호 | 예측 분산이 실제 오차와 상관 없음 → 조기 중단 가능 | 학습 오차가 solver보다 큼 | 필터 대비 개선 없음 |

**권장: C1을 주 Contribution으로**, C3는 C1의 temporal term에 흡수 (ablation), C2와 Candidate A/B(residual MLP, learned swivel)는 baseline으로 사용.

### C0. 현행 프로토타입: AprilTag 상대 손목 추종 (알고리즘 후보 아님)

손등 AprilTag 1개를 단안 웹캠으로 추적하고, clutch를 켠 시점부터의 상대 변위·회전을 NERO tcp에 더해 MoveIt Servo로 추종한다. 세부 구성과 매핑 수식은 [prototype_apriltag_wrist_teleop.md](prototype_apriltag_wrist_teleop.md) 참고. 정량 측정은 아직 없다.

- **신규성: 없음.** 마커 기반 상대 매핑 teleop은 표준 구성이며 C1–C3와 같은 축에서 비교하지 않는다.
- **연구 설정과의 차이:** 절대 대응이 없어 Embodiment 오차를 정의할 수 없고, 팔꿈치(ψ)를 다루지 않으며, Servo에는 null-space 목표를 넣을 수 없다. 따라서 E3 baseline(B1–B6)으로 직접 쓰기 어렵다.
- **활용:** (1) E.0 기준 측정 rig의 손목 pose 기준값 후보 (정확도 검증 필요), (2) 실기 단계의 안전·통신 인프라, (3) 사용자 연구에서 "conventional teleop" 참고 조건.
- **부수 확인:** NERO URDF는 SRS 구조 (J1–J3, J5–J7 축 교차, elbow offset 없음, |SE| 0.310 m, |EW| 0.270 m). D.3의 ψ 1-D 축소가 URDF 기준으로 성립한다. 실기 FK 검증은 Stage 2에서 한다.

---

## Part D. Recommended Mathematical Formulation (C1)

### D.1 좌표계
- 몸통 프레임 $T_{torso}$: 양 어깨 중점 원점, $x$ = 오른→왼 어깨, $z$ = 골반 중점→어깨 중점 (Gram–Schmidt 직교화). 순수 위치 차분 $p_E - p_S$는 몸통 회전을 정규화하지 못하므로 SE(3) 변환을 사용.
- 비교 조건: (a) 매 프레임 갱신 torso frame, (b) 초기 프레임 고정. 노이즈 전파 관점에서 (a)는 어깨 추정 노이즈가 모든 변수에 들어가고, (b)는 몸통 이동에 취약 — 두 조건 모두 E1에서 측정.
- 로봇 측: URDF 기반 virtual shoulder frame $T_{vs}$ (NERO의 J1–J2 축 교점 근방; **URDF 확인 전 정의 금지**), 길이 스케일 $s = L_R / L_H$.

### D.2 의도 변수와 사후분포
관측 $o_t$ = (단일 RGB-D keypoint, landmark confidence, 과거 $k$ 프레임). 의도 변수:

$$
z_t = (p_w,\; R_w,\; \psi) \in \mathbb{R}^3 \times SO(3) \times S^1
$$

$\psi$ = 어깨–손목 축 주위 팔 평면의 회전각 (SEW / swivel angle, Elias & Wen의 generalized 정의 사용).

네트워크 $g_\phi$ 는 keypoint 수준의 **heteroscedastic 공분산** $\Sigma_S, \Sigma_E, \Sigma_W$ 및 손 방향 공분산을 출력하고, 의도 공간으로는 1차 전파(delta method)한다:

$$
\Sigma_z = J_h(\hat P)\, \mathrm{blkdiag}(\Sigma_S,\Sigma_E,\Sigma_W)\, J_h(\hat P)^\top
$$

여기서 $h$ 는 keypoint → 의도 변수의 해석적 사상. ψ에 대해서는 von Mises 근사

$$
\psi \sim \mathrm{vM}(\mu_\psi, \kappa_\psi), \qquad \kappa_\psi \approx \frac{d_\perp^2}{\sigma_{\perp}^2}
$$

($d_\perp$: 팔꿈치의 어깨–손목 축까지 거리, $\sigma_\perp^2$: 해당 방향 투영 분산). **핵심 성질:** 팔이 펴질수록 $\kappa_\psi \to 0$ 이 *데이터와 기하로부터* 유도되며, 2610.16855의 수동 deadband를 대체한다.

**학습 목표:** 다중 시점 기준 좌표에서 계산한 $z^*_t$ 에 대한 음의 로그우도

$$
\mathcal{L}(\phi) = -\log \mathcal{N}(p^*_w \mid \mu_p, \Sigma_p) - \log \mathcal{N}_{\mathfrak{so}(3)}(\log(\hat R_w^\top R^*_w) \mid 0, \Sigma_R) - \kappa_\psi \cos(\psi^* - \mu_\psi) + \log I_0(\kappa_\psi)
$$

평균 보정(bias)도 함께 학습하므로 이것이 **Vision 오차 보정**의 학습 성분이다.

### D.3 Feasible-MAP solver (embodiment + uncertainty 결합)

$$
q_t^\star = \arg\min_{q \in \mathcal{Q}_t}\;
\underbrace{\| e_p(q) \|^2_{\Sigma_p^{-1}} + \| e_R(q) \|^2_{\Sigma_R^{-1}}}_{\text{손목: Mahalanobis}}
+ \underbrace{\kappa_\psi \big(1 - \cos(\psi_R(q) - \mu_\psi)\big)}_{\text{팔꿈치: 불확실성 자동 가중}}
+ \underbrace{\| q - q_{t-1} \|^2_{W_t}}_{\text{연속성}}
$$

$$
\mathcal{Q}_t = \{ q : q_{min} \le q \le q_{max},\; |q - q_{t-1}| \le \dot q_{max}\Delta t,\; d_{coll}(q) \ge d_{min} \}
$$

- $e_p(q) = \mathrm{FK}_p(q) - (s R_{align}\mu_p + p_{vs})$
- 손목 목표가 도달 불가능할 때 해는 **불확실성이 큰 방향으로 먼저 양보**한다 (Euclidean projection과의 차이 — G2).
- $W_t$ 는 사후 정밀도의 함수: 관측이 불확실할수록 이전 상태 유지 → 가림 복귀 시 불연속 억제 (C3 흡수).
- **효율적 구현 (NERO가 SRS 유사 구조일 경우):** 손목 pose 고정 시 해 집합은 ψ로 1-D 파라미터화되고, joint limit은 ψ의 feasible 구간(arc)으로 해석적 변환 가능 → ψ에 대한 1-D 문제로 축소. **NERO가 offset 구조이면** 이 축소가 성립하지 않으므로 SQP/QP (Pinocchio + OSQP 등)로 직접 푼다. → Stage 2에서 URDF로 판정.

### D.4 왜 IK + Residual MLP로는 부족한가 (Q2에 대한 검증 가능한 가설)
1. **H-a (circular collapse):** $h \to 0$ 에서 ψ는 사실상 다봉/균등 분포. MSE로 학습한 $\Delta q$ 는 조건부 평균을 내므로 팔꿈치가 "중간"으로 끌리거나 프레임 간 튐. → ψ 오차의 P90을 $h$ 구간별로 비교.
2. **H-b (error entanglement):** residual은 vision 오차와 embodiment 오차를 하나의 $\Delta q$ 로 학습 → 학습에 없던 사용자(다른 팔 길이)에서 일반화 저하. → leave-one-subject-out 성능 비교.
3. **H-c (no feasibility guarantee):** $q_{IK} + \Delta q$ 는 제약 위반 가능; clipping 후에는 손목 오차가 비구조적으로 증가.
4. **H-d (no direction-aware yielding):** infeasible 목표에서 어느 방향을 양보할지 정보가 없음.

이 넷은 모두 **반증 가능**하며, 반증되면 C1의 동기 자체를 재검토한다.

### D.5 오차 분리 (G3) — Oracle substitution

| 오차 | 정의 | 측정 조건 |
|---|---|---|
| Vision | $d(\hat z_t, z^*_t)$ | 단일 RGB-D 추정 vs 다중 시점 기준 (로봇 불필요) |
| Embodiment | $d(z^*_t, \Phi(q^{IK}(z^*_t)))$ | **기준 좌표를 입력**으로 solver 실행 → 로봇이 재현 못하는 부분 |
| Numerical | $d(\text{solver target}, \mathrm{FK}(q^{IK}))$ | solver 수렴 잔차 (해석해면 ≈0 → 학습 보정 불필요) |
| Execution | $d(\mathrm{FK}(q_{cmd}), \mathrm{FK}(q_{meas}))$ + 지연 | 실제 로봇 관절 측정값 |

$\Phi$ = 로봇 구성 → 의도 공간 사상 (스케일 역변환 포함). 오차는 가산적이지 않으므로 각 단계를 단독으로 보고하고, 상호작용은 2×2 (vision oracle on/off × robot-limit on/off) 설계로 본다.

---

## Part E. Minimum Viable Experiment

**목표:** C1의 *전제*가 참인지 로봇 없이 4–6주 안에 판정.

### E.0 장비 (확인 필요 항목 포함)
- 테스트 센서: 단일 RGB-D (**모델 미확인 → 정밀도·fps 가정 금지**)
- 기준 측정: 동기화된 추가 카메라 2–3대 다중 시점 삼각측량 + (선택) 상완/전완/손등 ArUco 커프 → 기준 좌표 자체의 정확도를 bone-length 분산, 기지 길이 물체로 먼저 검증
- TRIDOT 장갑: 모델·데이터 형식 확인 후, 손목 방향 기준값(IMU)으로 사용 가능한지 판단

### E.1 데이터
- 피험자 4–6명 (파일럿), 1인당 약 10–15분
- 동작: 도달 그리드, **같은 손목 pose에서 팔꿈치만 돌리기 (ψ sweep)**, 팔 펴기 (h→0), 몸 앞 교차·카메라 방향 뻗기 (자기 가림), 빠른 동작
- 분할: **피험자 단위 leave-one-subject-out**, 시퀀스 단위 분리, 정규화 통계는 train fold에서만 계산

### E.2 실험과 Go/No-Go (사전 등록 제안 임계값, 근거 데이터 확보 후 조정 가능)

| 실험 | 질문 | 지표 | Go 조건 (제안) |
|---|---|---|---|
| E1 전제 확인 | 단일 시점 ψ 오차가 $h$·가림에 따라 이분산적인가? | $h$ 구간별 ψ 오차 P50/P90, 가림 여부별 | 저-$h$/가림 구간 P90이 고-$h$ 구간 대비 뚜렷이 큼 |
| E2 불확실성 품질 | 예측 분산이 실제 오차를 반영하는가? | NLL, 68/95% coverage, 예측 σ–실제 오차 Spearman ρ | coverage 목표치 ±10%p 이내, ρ > 0.3 |
| E3 시뮬레이션 비교 | Feasible-MAP가 baseline보다 나은가? | Part F 지표 전체 (NERO URDF, 기록 데이터 재생) | 손목 오차 비열화 + ψ 오차·jerk·불연속 개선 |

**E2 실패 시:** C1의 전제 붕괴 → C2 또는 C3로 전환하고 그 결과를 보고 (실패도 기록).

### E.3 Baseline (동일 입력·동일 데이터 예산·동일 제약)
B1 DLS IK + posture homing · B2 SEW-Mimic (논문 기준 재구현, 공식 코드 공개 시 교체) · B3 2610.16855 방식 AEAC + J-PARSE · B4 B1 + One Euro filter · B5 IK + residual MLP/GRU ($\Delta q$, 레이블 = 기준 좌표 입력 오프라인 solver 해) · B6 IK + learned swivel target (Polimi/Biomimetics 방식) · **P: Feasible-MAP**

### E.4 Ablation
불확실성 제거 ($\kappa_\psi$, $\Sigma$ 고정) · Mahalanobis → Euclidean 투영 · 해석적 전파 제거 (네트워크가 κ 직접 출력) · temporal term 제거 · ψ term 제거 · 학습 bias 보정 제거

### E.5 지표 정의 (주요)
- Wrist position error [mm]: $\|\mathrm{FK}_p(q_t) - (sR\,p^*_w + p_{vs})\|$, 평균/P95
- Wrist orientation error [deg]: $\|\log(R_{FK}^\top R^*)\|$
- Elbow (swivel) error [deg]: $|\mathrm{wrap}(\psi_R(q_t) - \psi^*_t)|$, $h$ 구간별
- Jerk: $\frac{1}{T}\sum \|\dddot q\|^2$ (수치 미분 전 동일 필터 적용)
- Pose discontinuity: 가림 종료 후 1 s 내 최대 $\|\Delta q\|_\infty$
- Feasibility: joint limit 위반율, self-collision 비율, solver 실패율
- Latency: solver 시간 평균/P95/P99 [ms], end-to-end는 실제 로봇 단계에서 측정

---

## Part F. ICRA / IROS / CoRL / RA-L Publication Readiness Assessment

**현재 상태: 실험 결과 0건. 아래는 "MVE가 Go일 경우"의 조건부 판단이다.**

| 항목 | ICRA | IROS | CoRL | RA-L |
|---|---|---|---|---|
| 새로운 알고리즘 원리 | C1이 성립하면 충분 가능 | 가능 | 학습 성분이 약하다는 지적 위험 | 가능 |
| 기존 연구와의 차별성 | 2610.16855·Biomimetics와의 차이를 정량 입증해야 함 | 동일 | 동일 + learning 기여 강조 필요 | 동일 + 이론적 정당화 요구 큼 |
| 실험 충분성 요구 | 실제 로봇 + 사용자 연구 | 실제 로봇 + 소규모 사용자 연구 | 실제 로봇 + 일반화 | 가장 엄격 (통계 포함) |
| Generalization | 미학습 사용자 필수 | 동일 | 다른 로봇(2종↑) 요구 가능 | 미학습 사용자·조건 |
| Reproducibility | 코드·데이터 공개 권장 | 동일 | 사실상 필수 | 권장 |
| **현재 적합도 (조건부)** | 중 | **중–상 (가장 현실적)** | 하–중 | 중 (결과 강할 때) |

**남은 reviewer risk**
1. "결국 confidence-weighted IK 아닌가" → 해석적 전파 vs 직접 가중 ablation으로 방어
2. "SEW-Mimic/2610.16855 대비 개선 폭이 작다" → 저-$h$, 가림 조건에서의 효과로 범위 한정
3. "기준 좌표가 정말 정확한가" → 기준 측정 정확도 별도 보고
4. 단일 로봇(NERO) → 시뮬레이션에서 다른 7-DoF (예: Kinova Gen3, Franka) 추가 고려
5. 손(REVO2) 통합은 본 Contribution과 독립 → 시스템 데모로만 사용

---

## Part G. Immediate Implementation Roadmap

| 주차 | 작업 | 산출물 / 완료 기준 |
|---|---|---|
| 0–1 | **하드웨어·API 확인**: pyAgxArm (공식 Python SDK), `nero_description` URDF 출처 확인, REVO2 SDK/ROS2 (리셀러 문서만 확인됨 → BrainCo 공식 자료 확보), RGB-D 카메라 모델, TRIDOT 장갑 모델·데이터 형식 | `docs/hardware_inventory.md` |
| 1–2 | Stage 2: URDF FK 검증 (실기 joint 읽기 vs FK), **SRS 여부 판정**, torso/virtual-shoulder 프레임 정의 | `02_kinematics_validation.md` |
| 2–3 | 기준 측정 rig 구축 및 정확도 검증 | 기준 좌표 오차 보고 |
| 3–5 | 데이터 수집 (4–6명) → **E1, E2** | Go/No-Go 결정 |
| 5–8 | Baseline B1–B6, Feasible-MAP 구현, 시뮬레이션 **E3**, 오차 분리 | `03_baseline_failure_analysis.md`, `04_proposed_solver.md` |
| 8– | 안전 제한 검증 후 실기 NERO, 사용자 연구 설계 | `05`, `06` |

**안전 원칙:** 실기 명령 전 (1) 시뮬레이션 재생 통과, (2) joint/속도/workspace 하드 리밋 별도 레이어, (3) 저속 모드·비상 정지 확인.

---

## 다음 단계에 필요한 정보 (사용자 확인 요청)
1. RGB-D 카메라 정확한 모델
2. TRIDOT 장갑 모델명과 SDK/출력 형식
3. NERO URDF·SDK 접근 상태 (pyAgxArm 설치 여부)
4. 다중 시점 기준 측정용 추가 카메라 확보 가능 여부
