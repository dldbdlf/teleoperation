# 실험 로드맵 — IROS / CoRL 검증 계획 (Experiment Roadmap V1)

> 작성: 2026-11-04 · **실험 계획 문서, 결과 보고서 아님** · 2026-11-13 Canonical Mapping 추가
>
> 상위 계획: [../roadmap.md](../roadmap.md) · 근거: [02 NERO 기구학](02_kinematics_validation.md) / [2026-11-03 Jetson 실험](../11.03_exp/README.md)
>
> 연구 범위를 유지한다: **Single D415 RGB-D / glove-free / marker-free online inference → occlusion-aware human belief → Feasible-MAP constrained retargeting → NERO 7-DoF → REVO2 Pro 후속**.
> D435i·AprilTag는 라벨/Ground Truth/캘리브레이션 용도로만 허용하며, 온라인 제안 방법의 입력에 쓰지 않는다. 모든 수치 목표는 **제안된 사전등록 기준**이다.

## 0. 핵심 질문과 검증 논리

**추가 설계 명세:** [Canonical Feasible-MAP Solver 및 R1–R6 프로토콜](04_proposed_solver.md). 동일한 full human belief에서는 동일 q_ref/branch를 요구하되, 상태 의존적 q_cmd와는 구분한다.

- **H1 (Observation):** self-occlusion, depth invalidity, elbow-extension(팔꿈치 offset h→0)에서 3D/ψ 오차 및 불확실성이 체계적으로 변하는가?
- **H2 (Belief):** 실제 가림을 반영한 확률적 wrist SE(3) + ψ belief가 단일자세, 기하학적 보정, KF/One-Euro, 단일 von Mises에 비해 calibration과 오류를 개선하는가?
- **H3 (Retargeting, 주 기여):** 사람의 belief를 그대로 보존한 채 **로봇** feasible set 위에서 조합 최적화하면, 단순 uncertainty-weighted IK 또는 belief→IK 순차식 대비 wrist 오차를 유지하면서 ψ/recovery/jerk/real-task 성능을 개선하는가?
- **H5 (Canonical Repeatability):** 동일한 full human belief와 동일 URDF/TF/policy에서 이전 q, 입력 순서, random seed에 관계없이 동일한 q_ref/IK branch를 반환하는가? 같은 wrist 좌표지만 서로 다른 elbow ψ 의도는 구별하는가?
- **H4 (CoRL 추가 기여):** downstream **robot-control utility**를 학습 목표로 사용한 belief가 동일한 모델·데이터 예산의 독립적 NLL 학습보다 새로운 사람·가림 조건에서 로봇 성능을 개선하는가?

**주 Contribution = H3.** H1/H2가 거짓이면 복잡한 모델 도입을 중단하거나 재설계. H4는 CoRL 확장 가설이며 관찰 결과에 따라 주장 여부 판단. Occlusion 복원 자체, confidence weighting, filtering만으로 신규성 주장하지 않음.

**개념적 분리:** 인간 상태의 사후분포 p_H(z|o)는 로봇 제한 때문에 재정의/삭제하지 않는다. 로봇 실행 가능성은 이 분포에 대한 **명령 결정 q***에 적용. 사람 ψ가 정의 불안정한 near-straight 상태에서 ψ error를 무조건 평균 내지 말고 h별 stratification, undefined mask 사용.

## 1. 현재까지 완료/미완료 근거

| 구분 | 상태 및 근거 | 다음 사용 |
|---|---|---|
| NERO SRS FK/IK/ψ | **URDF 기반 계산·테스트 기록됨**: 1000 q IK round-trip, 최대 8 branch, feasible ψ arcs | EX03/EX04에 재사용 |
| 실행가능성 분석 | **시뮬레이션 기록됨**: ready 방향 wrist feasible 약 95–98%, elbow-down ψ 허용 약 15–19% (실제 human 행동 분포 아님) | EX03 stress 재현 |
| AprilTag wrist servo | 코드 프로토타입만 기록됨, 추종 오차·실제 폐루프 성능 미측정 | GT rig 후보/시스템 비교 조건 |
| Jetson + D415/D435i | 두 카메라 ROS2 노드와 **컬러** 약 14.6/14.4 FPS 기록; Depth 품질은 별도 검증 필요 | EX01/EX02 |
| ArUco 카메라 상대변환 | 30 samples, 위치 추정 z std 44.7 mm. **정밀 GT로 검증 전 불인정** | EX01 재캘리브레이션 |
| Feasible-MAP / learned belief / user task | **아직 실험 결과 없음** | EX04~EX10 |

## 2. 일정·선후 의존성·Gate

W1 = 2026-11-02. 기존 [상위 로드맵](../roadmap.md)의 W1–W17 일정을 따르며, Gate를 통과하지 않으면 후속 실험에서 성공했다고 표기하지 않는다.

| ID | 기간(제안) | 책임 트랙 | 실험과 주요 산출물 | 통과 Gate |
|---|---|---|---|---|
| **EX00** | W1–W2 | A+B | 실험 설정/로그 계약 + 재현성/안전 audit | 자료준비 |
| **EX01** | W1–W3 | A | D415 RGB/Depth alignment + GT rig accuracy | G0 |
| **EX02** | W1–W5 | A | markerless S/E/W 3D + ψ + occlusion/depth validity | G0→G2a |
| **EX03** | W1–W3 | B | NERO branch, singularity, ψ arcs, infeasible wrist + R1/R2 deterministic fixture | G1 준비 |
| **EX04** | W2–W4 | B | synthetic oracle occlusion + strong IK baseline vs Feasible-MAP + R1–R6 | **G1 + R-CANON** |
| **EX05** | W4–W7 | A | 실제 인간 occlusion benchmark, GT mask·held-out split | **G2a** |
| **EX06** | W6–W10 | A | uncertainty calibration / unimodal↔multimodal belief | **G2b** |
| **EX07** | W8–W12 | A+B | recorded replay, coupled solver vs weighted/sequential IK + real belief R1–R6 | **G3** |
| **EX08** | W10–W14 | B | real NERO safety, latency, tracking dry-run→live | G4 사전조건 |
| **EX09** | W12–W16 | A+B | 실제 인간 teleop task study, arm→hand 확장 | **G4** |
| **EX10** | W18+ | A+B | CoRL control-aware learning·unseen user/camera/robot generalization | G5 |

실행 순서의 핵심: **EX01과 EX03/EX04를 병렬 시작**. EX06 학습 및 EX07 통합은 EX05 data quality + EX04 oracle value가 확인된 뒤. EX08 실기 명령은 safety 검증 전 수행 금지.

## 3. 실험별 구체 프로토콜

### EX00 — 공통 실험 계약 및 재현성 (우선 시작)

**목적:** 두 트랙의 입력 불일치, GT 누출, baseline별 다른 안전 설정 방지.

**최소 공통 데이터 계약**
- Observation: image_timestamp_ns, frame_id, S/E/W [m], hand orientation [SO(3) or quaternion], visibility, depth_validity, landmark confidence, measured/estimated covariance, torso transform, sequence_id, subject_id(익명화).
- HumanBelief: timestamp, wrist SE(3) mean + tangent-space 6×6 covariance, 원형 ψ distribution (초기 mean/κ, 필요 시 mode/weight), occlusion type, age_ms; 단위/좌표계/undefined-ψ mask 명시.
- CanonicalTarget: q_ref[7] [rad], canonical_psi, branch_id, mode_id, cost_gap, feasible, solver_config_hash. 동일한 full human belief/고정 robot 설정일 때 항상 동일 target; q_prev 또는 warm-start에 의해 q_ref 변동 금지.
- SolverCommand: q_cmd[7] [rad], optional qdot [rad/s], timestamp, status (TRACK/HOLD/SAFE_STOP), reason, safety_override. **실제 명령은 현재 로봇 상태 의존적이며 safety shield는 독립**.
- Dataset manifest: split 기준(피험자·원본 episode), sensor serial/intrinsics, calibration SHA, model/URDF SHA, time synchronization, GT validity, consent status, experiment_code_git_sha.
- 모든 방법에 동일한 안전/속도/제어 주기/카메라 스트림/훈련 데이터 예산.

**필수 산출물:** configs 버전, input→solver→output 예시 기록 1개, clean clone 재생 가이드, test-only split manifest, 결과 JSON schema.

**통과:** 합성 샘플 1개를 A→B 전송 후 단위/TF/타임스탬프 테스트 성공; data leakage 체크 통과.

### EX01 — D415 단독 관측 및 Ground Truth 신뢰성

**목적:** 기존 Jetson 컬러 스트림 성공과 실제 3D 정확도를 구분.

**절차:** (1) D415 단독 color/depth/camera_info를 동시 기록, FPS·drop·frame age 측정. (2) depth-to-color 정렬과 pinhole 역투영을 다양한 거리/각도의 알려진 물체로 확인. (3) depth-validity mask (0, sentinel, 범위 밖, 배경/전경 혼동) 구축. (4) D435i + 검증된 마커 rig를 **오직 GT 수집에 사용**하고 calibrated transform 반복 측정. (5) 마커 부착 offset·GT 가림·시간 동기화 오차 평가.

**반드시 구분:** 기존 D415↔D435i ArUco 변환의 z std 44.7 mm는 camera-to-camera 추정 분산이며 인간 관절 GT 오차가 아니다. 이 상태를 정밀 GT로 가정하지 않는다. D435i가 NERO 손목에서 움직이면 static TF 금지, hand-eye + robot FK에 따른 동적 TF 필요; GT rig 용도에서는 고정 배치 권장.

**지표:** valid-depth ratio, 3D known-point RMSE/P95 [mm], camera reprojection [pixel], transform repeatability [mm/deg], stream color/depth fps and timestamp skew [ms], missing frame %, GT valid ratio.

**판정 G0:** GT 정확도/반복성이 연구에서 보고하려는 알고리즘 개선폭을 구별할 만큼 충분하고, timestamp+frame convention이 검증됨. 불충분하면 재캘리브레이션 후 EX05 시행; 숫자 임의 가정 금지.

### EX02 — Markerless 3D Human Pose / Occlusion Detection (Track A)

**입력:** D415 RGB+aligned depth만. detector는 MediaPipe 등 공개 baseline부터 시작하고 모델·버전 고정. **온라인 입력에는 D435i, glove, tag 불사용**.

**방법:** shoulder/elbow/wrist 2D→aligned depth→3D; ROI median/robust depth, foreground test, temporal association, torso coordinate, wrist orientation, φ/ψ. 가려진 관절에서 배경 depth를 관절 depth로 읽지 않는다. elbow h와 ψ singular/undefined를 분리.

**지표:** 관절 위치 error [mm], wrist R error [deg], circular ψ MAE/P90 [deg], occlusion precision/recall, valid-depth false acceptance, frame-time P50/P95, FPS; visible/occluded, h buckets 별도.

**산출물:** ROS2 inference 노드, versioned rosbag replay, 2D-3D 시각화, failure clips, EX01 GT 비교표.

**판정:** 모든 프레임에 confidence가 나오는 것을 성공으로 간주하지 않음. **관측 불가와 추정값을 명확히 구분**하고 GT mask를 일관되게 적용.

### EX03 — NERO 기구학/제약 stress & solver readiness (Track B)

**입력:** 공식 NERO URDF @ 고정 SHA, 기존 share/sim/kinematics.py 재사용. 단순 uniform sampling 외 실제 가능 궤적·경계 테스트.

**경계 조건:** ψ ±π wrapping, 좁은 feasible arc, 8 IK branch 전환, near-extension(h→0), J4/J6 hard limits, wrist infeasible SE(3), velocity/acceleration limits, singularity/self-collision, unreachable goal. Pose가 infeasible할 때 ψ만 변경하는 해결책 금지; wrist target relaxation 경로 정의.

**지표:** FK→IK round-trip, missed ψ arcs rate (fine-grid / independent numeric reference), branch-jump count, optimization residual, limit/collision breach, feasible oracle regret, latency p50/p95/p99.

**산출물:** deterministic pytest/regression seeds, 경계 사례 CSV 및 시각화, branch continuity 규약, safety shield spec.

**추가 R1/R2:** 동일 full belief fixture를 N≥100회 반복·프로세스 재시작·호출 순서 역전하며 q_ref/branch 안정성을 기록한다. 서로 다른 초기 q와 접근 경로로 같은 belief에 도달했을 때도 canonical 결과가 같은지 검증한다. 실제 명령 q_cmd의 즉시 일치는 요구하지 않는다.

### EX04 — Oracle Synthetic Benchmark (G1: 가장 먼저 Contribution 검증)

**목적:** 실제 perception 문제를 배제한 채 solver 수학적 기여부터 평가. 정답 human wrist/ψ와 주입한 noise·covariance를 기록한다.

**시나리오:** S1 ψ sweep(고정 wrist), S2 near-straight elbow, S3 J6/SE(3) infeasibility, S4 elbow-only occlusion, S5 wrist dropout, S6 visibility return + sudden direction reversal.

**추가 EX04-R:** R1 동일분포 100회 결정성, R2 서로 다른 이전 궤적/초기 q에서 동일 canonical target, R3 같은 실제 자세+관측 노이즈 변화, R4 elbow/wrist occlusion 및 재인식, R5 feasible arc·joint limit·singularity 경계, R6 동일 손목 SE(3)지만 다른 elbow ψ belief. 각 실험에 q_ref/q_cmd, branch/mode ID, cost gap, config SHA, safety override를 분리 기록한다. 공간/속도·노이즈·dropout seed를 baseline별 동일하게 고정.

**계획 샘플:** 먼저 장면별 20 trajectories × 3 seeds로 디버그; 기능 안정화 후 장면별 100 trajectories × 5 seeds 목표(계산 용량에 맞춰 수정/사전기록). 0.2, 0.5, 1.0, 2.0s dropout; 실제 카메라 약 15fps를 반영한 observation update, 별도 robot command control rate. 1개 trajectory에 여러 지표가 있지만 독립 샘플로 세는 단위는 trajectory/seed.

**비교군:** B0 wrist DLS IK+posture, B1 SEW direction follow, B2 adaptive elbow gating, B3 B0+One-Euro, B4 B0+Kalman, B7 fixed/unimodal uncertainty-weighted constrained IK, P Oracle-Σ Feasible-MAP. **추가 비교:** home-canonical IK, previous-q seeded IK 및 B2/B7의 동일 canonical tie-break 적용 버전. 단순 결정성 효과와 coupled posterior 최적화 효과를 분리. 비교군 모두 동일 safety shield, q0, speed caps, workspace, invalid-input handling. Oracle-Σ는 **잠재 상한**이며 실제 learned-Σ와 성능 비교를 섞어 제안 방법의 실제 성능처럼 주장하지 않는다.

**핵심 지표:** feasible 목표에서는 wrist tracking position/orientation mean/P95와 ψ error P90; infeasible 목표에서는 best-feasible oracle regret와 ψ/wrist tradeoff; 전체 jerk / branch jumps / recovery peak speed / deadline miss / runtime P95.

**R-CANON 제안 기준:** 동일 실행 환경에서 동일 full belief를 N≥100회 넣어 q_ref 최대 관절 차이 ≤1e-6 rad, branch/mode 불일치 0을 목표로 한다(실험 결과 아님). Cross-path는 최종 q_ref 동등성을 보되 q_cmd는 안전한 수렴 후 평가한다. Near-singularity/feasible 경계에서는 global continuity 보장 금지; 실패/hold를 별도 보고.

**Gate G1 사전등록 제안:** strongest fair baseline 대비 wrist 비열화 예시 median +5 mm/+5° 이내이며, ψ P90/jerk/recovery 중 사전 선정한 1차 지표에 실질 개선(초기 목표 15–20%). 안전 위반은 동일 safety shield 하에서 평가. **이 수치는 아직 관측값 아님.** 개선 없으면 belief 모델 훈련 전에 solver 구조와 loss를 수정한다.

### EX05 — 실제 인간 Occlusion Dataset 및 E1 (G2a)

**수집 전제:** IRB 필요 여부·동의·영상 저장·얼굴 익명화 등 학교 방침 승인. 참가자 손등/상완 등 GT marker는 오프라인 기준 측정용으로만, 실제 추론 영상으로 사용되는 D415에는 마커/보조 시점 정보가 입력되지 않도록 촬영·masking protocol 명시.

**초기 수집 계획:** pilot 4–6 people × 10–15 min. 모든 참가자에서 동일한 O0–O7 시나리오와 시간 조건을 가능한 범위 균형 배치; 이어서 본 연구 규모는 pilot effect/variance와 GT usable ratio 기반 결정.

| Occlusion strata | 시험 조건 | 핵심 비교 |
|---|---|---|
| O0 | 정상 팔 동작 | 정상 baseline error |
| O1 | torso 뒤 elbow self-occlusion | ψ reliability |
| O2 | 물체 뒤 wrist/hand | wrist prediction vs safe hold |
| O3 | 거의 펴진 팔, h↓ | ψ 정의성/다봉성 |
| O4 | torso crossing / 깊이 모호성 | invalid depth 거부 |
| O5 | 빠른 동작 중 가림 | temporal belief |
| O6 | 가려진 동안 방향 반전 | overconfident prediction |
| O7 | 0.2/0.5/1/2s 소실 뒤 재관측 | recovery/discontinuity |

**분할:** subject-held-out (가능하면 train/val/test 별도 사람), 참가자가 너무 적으면 leave-one-subject-out + 별도 마지막 테스트 참가자; test는 hyperparameter tuning 금지. 동일 원본 episode의 synthetic-mask 버전은 동일 split. GT invalid/occluded-from-reference 프레임은 GT error/NLL의 primary 평가에서 제외하고 mask 통계 공개.

**E1:** h = distance(elbow, shoulder–wrist line), h=0 근처 ψ undefined. h bins·occlusion type·duration에 따른 3D/ψ MAE/P90, valid GT 비율, depth false acceptance 보고. 카메라 pose 변화·피험자 팔 길이도 stratify.

**산출물:** dataset manifest, scenario balance, sample video(동의 범위 내), calibration report, E1 charts, GT quality audit.

### EX06 — Belief / Uncertainty Learning (G2b)

**실험 순서:** (1) 기구학 보정 + CV/KF baseline, (2) 학습된 heteroscedastic Gaussian wrist pose + von Mises ψ, (3) 필요 시 mixture-von-Mises/particle posterior, (4) temporal GRU/compact transformer. 모델만 복잡하게 키우지 않는다.

**학습:** NLL + 재투영/시간적 일관성 보조 loss, GT missing mask, subject split; wrist orientation SO(3) tangent error와 ψ periodicity 처리. uncertainty와 mean bias correction을 별도로 ablation. outlier/low-h에서는 적절한 circular/undefined 처리가 핵심.

**지표:** wrist/ψ MAE/P90, NLL, 68/95% empirical coverage(명목값 대비), ECE/accuracy-confidence, σ–absolute-error Spearman ρ, occlusion duration별 uncertainty vs error. 인간 관절 label quality 부정확 시 GT label noise 조치.

**Gate G2b 제안:** calibrated coverage 목표 ±10%p, Spearman ρ>0.3, baseline보다 낮은 NLL 또는 미학습-user 개선. 숫자는 초기 **판정 목표**이고 실제 비교 후 수정 시 pre-reg 기록. 다중 가설 이득이 없으면 작은 unimodal posterior 채택.

### EX07 — Coupled Belief-Constrained Solver: 핵심 통합 (G3)

**필수 구조:** learned human belief → 결정적 canonical q_ref/branch 선택(현재 q와 독립) → 상태 의존 q_cmd tracker/safety. 동일 실제 belief 반복 시 q_ref가 달라지면 실패로 처리. 이전 q를 canonical solver의 목적함수나 seed 선택에 주입하지 않는다.

**동일 기록 데이터 입력으로 공정 비교**:
- B5 AKC-style geometry correction + common IK, B6 residual MLP/GRU 또는 learned ψ, B7 calibrated unimodal confidence-weighted IK, **B8 multimodal belief→ordinary constrained IK (sequential)**, P proposed coupled belief-constrained Feasible-MAP.
- Human belief posterior p_H(z|o)는 **NERO feasible 여부로 변경하지 않음**. Pose inference accuracy(인간 추정)와 robot optimization outcome(로봇 행동)을 별도 기록.
- Joint limits·collision·speed/acceleration·input age threshold는 전 비교군 동일. Wrist infeasible에는 task-space pose relaxation 포함하는 조건과 미포함 조건을 구분.
- Ablation A1 no visibility, A2 fixed Σ, A3 single ψ hypothesis, A4 sequential vs coupled, A5 no temporal/branch tracking **(실행 계층에서만)**, A6 no pose relaxation, A7 no bias learning; **한 번에 하나만 제거**.
- **추가 ablation:** canonical tie-break off, q_prev를 reference solver에 직접 주입, home prior off, deterministic baseline + same safety, mixture-belief→canonical IK, tracker hysteresis off(별도 safety 유지).

**지표:** EX04 oracle와 동일 + 실제 EX05 sequences의 P95 tracking/jerk/branch jumps, stratified per occlusion and near-limit, solver P50/P95/P99, CPU/GPU 사용량. **추가 R1–R6 지표:** q_ref max-norm diff, branch/mode mismatch, cross-path target equality, noise-to-q P95, q_ref 대비 q_cmd/실측 관절 수렴 차이. 몇 프레임이 아닌 **episode·subject cluster**로 CI 산출. 5-seed(가림/초기화/샘플) 재현을 목표.

**Gate G3:** strongest B7/B8 대비 단순 smoothing 이상의 이득이 관측되는가? 동일 safety, success/failure 모두 기록. 효과 없으면 학습 규모 증가 전에 목적함수/branch coupling 재설계.

### EX08 — Jetson End-to-End + 실제 NERO 저속 안전검증

**순서:** simulation-only → command generator dry-run(실기 명령 차단) → 저장 입력 재생 → 조작자 승인된 저속 단관절 → 저속 팔 추종 → 온라인 D415 live. 한 단계라도 실패하면 실제 실행 중단.

**필수:** physical E-stop, heartbeat/watchdog, timeout-to-hold/stop, velocity/acceleration caps, workspace limit, collision monitor, timestamp stale detection, ramped reacquisition, robot joint feedback sanity; 한 개의 별도 safety shield에서 강제. 실제 limit 및 pyAgxArm/ros2_control command 주기 측정. REVO2 Pro 통합은 팔이 안정화된 후.

**지표:** color+depth FPS, RGB-D→Pose P95, belief P95, solver P95, command publish P95, **sensor timestamp→actual actuation P95/P99**, watchdog response, wrist tracking FK measured, failed frames/unsafe commands. canonical q_ref와 실제 q_cmd/q_measured, tracking_status/override, branch_id를 분리 기록하여 동일 목표의 재수렴 평가. 50/100 Hz launch 설정을 실제 처리율로 주장하지 않음.

**Go 조건:** 기록된 시뮬레이션 궤적에서 limit violations 0, emergency stop/timeout 확인, 사람 없이 작업공간 위험요소 제거, 감독자 승인. 임의 prediction 동안 위험한 움직임이 있으면 중단.

### EX09 — Real Human Teleoperation Study (G4 / IROS 주 결과)

**목적:** 인간의 실제 조작 성공률과 가림 중 안정성 개선 증명. 성공률은 **모방학습 policy가 아닌 인간 teleoperator의 성공 비율**이다.

**과제 구성:** 3–5 task pilot → 본 평가 가능한 경우 6–10 task(목표 10). 예: 목표 위치 도달, 방향 맞춤, 장애물 접근, 좁은 공간 이동, 정렬, 가림 뒤 복귀, pick-and-place, 물체 이동, 배치, 복합 조작. 손가락이 필요한 task는 REVO2 Pro 통합 성공 후에만 포함, 아니면 arm-only task로 구분. 모든 방법에 동일 조작 대상·robot workspace·환경.

**사전 설계:** 8–12명 pilot→본 실험 표본수는 pilot 효과 크기/분산으로 계산. learned-model train에 포함되지 않은 참가자; 참가자별 3–4 방법, 2+ attempts, 방법/과제 순서 counterbalance, 동일 사전 연습 시간, 무작위화, task success 사전 정의 및 evaluator/영상 기록. 사용자가 10 task를 목표로 하지만 근거 없는 task 확대보다 각 조건의 반복과 paired analysis를 우선한다.

**Primary outcome:** per-task/per-person Task Success Rate(SR), completion time(실패 timeout 포함), 가림 지속시간별 SR/hold fraction. 서로 다른 접근 경로로 동일 human belief에 도달하는 trial을 포함하여 canonical target 재현성과 실기 수렴 후 관절 자세를 별도로 비교. Secondary: wrist/ψ errors, jerk, recovery latency/max joint jump, near-collision, safety intervention rate, preference/effort(선택). SR 도전 목표 +10 percentage points는 **검증되지 않은 희망치**.

**통계:** subject 단위 paired or hierarchical model + 95% CI, trial/episode cluster bootstrap, task-stratified 효과, multiple comparisons 해석. 막대그래프에 단순 frame 수를 독립 n으로 사용 금지. 모든 실패·중단·실제 actuator latency 포함.

**Gate G4:** 제안법이 현실 과제에서 성능/안정성 이득을 재현하고 안전 감시 통과. 통계가 부족하면 pilot 성과로만 보고하며 강한 SR claim 금지.

### EX10 — CoRL 학습 기여 확장 (IROS 검증 이후)

**새 가설:** pose estimation NLL만 최소화하는 posterior보다 **downstream robot-control utility를 고려하여 학습한 posterior**가 constrained solver의 조작 성공·안정성·OOD 일반화를 개선한다.

**학습 설계:** 동일 architecture/data budget에서 supervised NLL vs NLL+control-aware objective; differential rollout/QP training이 불안정하면 simulation teacher distillation. 사람의 진짜 ψ posterior를 로봇 제약으로 오염시키지 않고, 별도 robot action/decision loss로 학습 분리. model capacity/data-size ablation.

**일반화:** 새로운 피험자, 카메라 거리/측면, 배경/가림 물체, 조명, 제어 주기/지연, 다른 7-DoF URDF (예: 공개 Franka/Kinova URDF sim; 실기 NERO만 사용해도 검증 가능) 및 일부 동작 분포 변화. OOD별 absolute error뿐 아니라 calibration & task utility 보고.

**학습 주장 자격:** 고정 belief + solver와 end-to-end control-aware training의 차이가 통계·학습 효율/일반화로 확인될 때. 부정 결과라면 IROS-style solver 기여 유지, 학습 claim 삭제.

### R-CANON 평가 및 리포팅 원칙 (2026-11-13 추가)

- R1 반복성은 **동일 full posterior + 고정 URDF/TF/모델·정책**을 조건으로 한다. 손목 XYZ가 같아도 ψ 의도가 다르면 다른 q_ref가 올바를 수 있다.
- R1–R6 결과에 `q_ref`, `q_cmd`, `branch_id`, `mode_id`, `cost_gap`, `safety_override`, `solver_config_hash`, `repeatability_max_norm_rad`을 기록한다.
- 수치적 결정성, 작은 입력 변화에 대한 강건성, 로봇 실제 종료 관절각 일관성을 **별도 지표**로 보고한다. global 연속성이나 모든 상태에서 즉시 동일 명령은 보장하지 않는다.
- oracle belief 기반 EX04 성능을 실제 learned belief 성능으로 과장하지 않고, EX07과 EX09로 증분 검증한다.

## 4. 사전등록 공통 평가 기준

| 그룹 | Primary / secondary measurement | 필수 slice |
|---|---|---|
| Vision | 3D S/E/W mm, wrist orientation deg, ψ circular MAE/P90, NLL/coverage | visible vs elbow/wrist occlusion, h bins, camera distance |
| Retargeting | wrist 3D mm & SO(3) deg, feasible ψ deviation, infeasible oracle regret, jerk/branch jumps, **canonical repeatability / cross-path consistency** | wrist feasible 여부, J6 근접, near-straight, near-tie |
| Recovery | drop length, max |Δq| first second, peak velocity, time-to-stable, false continue, hold fraction | elbow-only vs wrist-loss, 0.2/0.5/1/2s |
| Real Task | operator success rate, completion time, intervention, end-to-end latency P95/P99 | task, user, method, occlusion severity |
| Safety | violations & misses, robot physical E-stop response | all methods identical independent shield |

**가림이 없을 때도 이득이 있는지 vs 가림에서만 의미 있는지** 별도 보고한다. primary outcome을 사전에 명시하고 탐색적 결과와 구분한다.

## 5. 공정 비교·데이터 누출 방지

1. Oracle-Σ는 이상적 upper bound: Learned-Σ의 **실제 제안 결과**와 혼동 금지.
2. 모든 baseline에 동일한 safety shield/속도/충돌 제약; same view, same sample budget, same control update, same warm start 및 timeout.
3. GT 생성에 사용된 D435i, AprilTag 위치, subject identity가 온라인 입력 feature에 섞이지 않게 파이프라인·리플레이 audit.
4. 테스트 사람, 원본 시퀀스, 동일 sequence의 masked augmentation, camera calibration/test tuning은 분리.
5. 로봇 ψ가 물리적으로 불가능한 human ψ와 달라졌다는 사실만으로 '오류'로 보고하지 않고 **best feasible regret**로 비교.
6. human loss·robot loss·solver numerical residual·execution error를 oracle substitution으로 분리. 서로 단순 가산하지 않는다.
7. OOD 개선, 학습 기여, user SR 등의 결론은 독립 참가자·episode 반복/CI와 함께 보고.

## 6. 결과 파일·리포트 규격 (추후 생성할 경로; 지금 실행됐다는 뜻 아님)

공유 경로 제안:

    share/experiments/
      configs/{EX00..EX10}/
      code/  # 독립적인 evaluator, benchmark runner, common safety
      splits/
      manifests/
      results/<experiment_id>/<run_id>/
        config.yaml
        dataset_manifest.json
        metrics.json
        per_episode.csv
        failures.csv
        git_sha.txt
        environment.txt
        plots/
      reports/<experiment_id>.md

각 결과에는 반드시 **실험 ID, 날짜, 장비·FW·OS/ROS 버전, URDF/model git SHA, seed, 원본 데이터 manifest, split ID, method, safety limits, FPS/제어주기, 실패 사유, 재현 명령, 측정된 값**을 포함.

실험 상태는 NOT_STARTED → RUNNING → PASS/PARTIAL/FAIL을 사용. 빈 표를 성과로 표시하지 않는다. Git에는 스크립트·집계 수치·동의된 비식별 결과만 커밋; 원본 참가자 동영상은 안전한 비공개 저장소에 보관한다.

## 7. 다음 2주간 가장 먼저 시작할 일 (각각 병렬)

**Track A (Vision / Dataset / Belief) — EX00, EX01, EX02**:
1. Jetson에서 D415 단독 color/depth/camera_info rosbag 1개; synchronized timestamp, measured FPS/latency table.
2. Depth↔Color alignment 검증 및 3D point backprojection 오차 확인.
3. S/E/W/hand landmarks + depth validity + occlusion flag 출력; 팔꿈치가 가려지면 background depth를 reject하는 테스트.
4. D435i 고정 GT rig calibration 반복과 known-length QA, 독립 GT-valid mask.
5. EX01 결과표와 EX02 preliminary error plots; 사람 데이터 수집 전 IRB·동의 확인.

**Track B (Kinematics / Solver / Robot) — EX00, EX03, EX04**:
1. 현재 SRS IK/ψ tests 회귀; near-straight/ψ wrap/narrow arc/8 branch/J6 stress 추가.
2. 동일 seed에서 synthetic S/E/W + ψ trajectories, injected noise, occlusion, true Σ 생성.
3. B0–B4와 B7 minimum baselines → oracle Feasible-MAP 구현, 동일 safety 조건 비교.
4. Wrist feasible/infeasible 나눠 ψ P90, wrist error, jerk, recovery, runtime 출력.
5. 결과가 나온 이후 G1 review, belief 학습 고도화 여부 결정.

**첫 공동 리뷰 기준:** EX00 contract, EX01 calibration report, EX03 stress unit tests, EX04 same-protocol baseline preliminary; 두 트랙 결과가 아직 없으면 논문 기여 완료로 주장하지 않는다.

## 8. Gate 실패 시 행동과 학회 선택

| 실패 | 처리 |
|---|---|
| G0 GT 부정확 | rig·검증 방식 수정, 실제 ψ 오차 주장은 보류. Oracle simulation 독립 진행 가능 |
| G1 Oracle solver 기여 없음 | confidence-weighted baseline과 구조적 차이 분석, branch/occlusion objective 개선한 뒤 재실험 |
| G2 belief 미보정·단일모드로 충분 | calibrated heuristic/KF 채택하고 과도한 multimodal network claim 삭제 |
| G3 sequential belief→IK와 동등 | coupled optimization의 명확한 제어 이점 재검토; 막연한 모델 확대 금지 |
| G4 실기 task 효과 불명확 | 사용자·과제·GT/안전/지연 원인분석, 주장 범위 축소 |
| CoRL learning 개선 없음 | 학습기여 주장 금지. IROS 중심의 실기/solver 결과 강화 |

**참고:** 본 문서는 실행할 프로토콜이며 아직 성능 개선을 확인했다는 뜻이 아니다. 실험 결과에 따라 변경 시 변경 이유·기존 결과·새 기준을 기록하고, 공동 결정은 [../decision.md](../decision.md)에 남긴다.
