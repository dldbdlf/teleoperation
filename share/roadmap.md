# 연구 로드맵 V2 — Glove-Free Occlusion-Aware Feasible-MAP Retargeting

> 갱신 제안일: 2026-11-04 · **검토용 초안** · W1=2026-11-02 주 · 대상: IROS 2027 / CoRL 2027
>
> 기존 연구 방향 유지: 단일 RGB-D(D415) → 사람 팔 관측·불확실성 → NERO 7-DoF Feasible-MAP → 실기, REVO2 Pro 후속 통합. 
> **실시간 추론에는 장갑/마커/2번째 카메라를 사용하지 않는다.** D435i/마커/모션캡처는 학습·평가 시 Ground Truth 취득용으로만 사용 가능하다.
> 일정·Go/No-Go·Contribution 가설은 팀 리뷰 후 [decision.md](decision.md)에 확정 기록한다. 이번 변경은 의사결정 확정이나 실험 성공 주장이 아니다.

## 0. 목표 및 연구 질문

**연구 질문:** 단일 RGB-D에서 가려진 인간 팔의 동작이 다의적으로 추정될 때, 인간 동작의 *확률적 belief*를 로봇의 실제 실행 가능 영역, 연속성, 안전 제약에 연결하면 고정 가중치·단순 필터·단일자세 IK보다 실제 원격조작이 개선되는가?

논문 중심은 **새로운 실시간 Retargeting solver**이다. Occlusion-Aware Belief는 핵심 입력·학습 기여, occlusion recovery는 폐루프 제어 효용을 검증하는 부분이다. 단순 RGB-D pose 복원, confidence-weighted IK, 시계열 smoothing, 모듈 연결을 각각 독립적인 주요 신규성이라고 주장하지 않는다.

### 목표 venue와 실제 마감

- **IROS 2027 우선 트랙:** IEEE RAS 공고 기준 논문 제출 마감 **2027-03-01**. 이에 맞춰 W16(2027-02-15)까지 핵심 그림·실험 확정, W17(2027-02-22)에 원고·비디오 검수한다. 최종 시간대·세부 규정은 공식 CFP에서 재확인.
- **CoRL 2027 확장 트랙:** 2027 CFP 일정 미확정. 동일 파이프라인을 유지하면서 **학습 기반 posterior의 실질 기여, robot-aware learning loss, 미학습 조건의 일반화**를 강화한다. CoRL 2026 CFP는 확률적 불확실성·학습+계획·실기 relevance를 강조한다.
- 동시 투고 가능 여부·중복 투고 정책은 각 CFP를 확인한다. 같은 성과를 동시 중복 제출하는 계획은 세우지 않는다.

참고: https://www.ieee-ras.org/event/2027-ieee-rsj-international-conference-on-intelligent-robots-and-systems-iros-70525/ , https://2026.corl.org/contributions/call-for-papers

## 1. 검증할 Contribution (가설)

| ID | 기여 가설 | 반드시 필요한 차별화 실험 | 실패하면 |
|---|---|---|---|
| **C1: Occlusion-Aware Motion Belief** | occlusion·depth validity·신체 기구학·시간 이력을 통해 손목 SE(3) 및 원형 ψ의 **보정된 확률분포**를 학습/유지한다. 필요하면 mixture-von-Mises 등 다중 가설. | 단일 pose, KF/One-Euro, AKC-style bone constraint, confidence-only, 단일 von Mises 대비 NLL·coverage·가림 구간 오차·미학습 피험자 비교 | 단순 기하+filter가 충분하면 복잡한 network 버리고 더 작은 belief 사용 |
| **C2: Feasibility-Constrained Belief Retargeting (주 기여)** | 인간 posterior를 임의로 로봇 관절제약에 맞춰 바꾸지 **않고**, 가능한 인간 의도 분포를 유지한 채 feasible robot trajectory를 계산한다. IK branch·ψ arcs·속도·충돌·연속성을 함께 고려. | 동일한 안전계층에서 SEW IK, adaptive elbow weighting, uncertainty-weighted IK, IK+GRU, sequential belief→IK 대비 wrist·ψ·jerk·recovery·task SR | 단순 weighting과 동등하면 최적화 구조/목적함수 재검토 |
| **C3: Occlusion-Recovery Control (C2의 속성)** | 팔꿈치만 가림/손목 가림/장기 소실/재관측에 따라 서로 다른 제어 신뢰도 및 안전 응답 | dropout 길이별 command jump, recovery time, unsafe extrapolation, task completion | 별도 이득이 없으면 독립 contribution으로 주장하지 않음 |

**주의:** 로봇에서 실현 불가능한 인간 ψ를 인간의 실제 자세 posterior에서 제거하는 것은 개념적 오류이다. 인간 belief p_H(z|observations)와 로봇 의사결정/행동 q*는 분리하고, robot constraints는 후자에만 적용한다.

### 방법 정의: 두 단계가 아니라 학습–기하–제어가 정합된 구조

1. **Observation:** o_t=(RGB-D S/E/W/hand landmarks, confidence, depth validity, image timestamp, past window), glove-free. RGB-D 상에서 보이지 않는 관절은 배경 깊이를 그대로 관절 깊이로 채택하지 않는다.
2. **Human belief:** p_φ(T_w, ψ, v | o_1:t). 초기 MVE는 wrist Gaussian / orientation tangent covariance + von Mises(ψ); 명확한 다봉성이 관측될 경우 mixture 또는 particle representation으로 확장. 사람 상완·전완 길이 제약과 일관된 분포, 2D reprojection consistency 검토.
3. **Robot feasibility:** 공식 NERO URDF에서 ψ feasible arc와 최대 8개 branch, joint limits, qdot/qddot, collision, singularity margin. 손목 pose 자체가 infeasible하면 ψ-only solver로 해결 불가하므로 task-space pose relaxation 포함.
4. **Control:** posterior 하의 expected task loss + 안전/속도 제약 + short-horizon continuity/recovery를 최적화. 다중 모드에서 평균 자세로 붕괴하거나 branch 점프하지 않도록 explicit hypothesis / temporal hysteresis 검증. 확신 없는 손목 장기 소실은 정지·hold로 종료해야 하며 무한 예측 금지.
5. **CoRL 확장:** supervised NLL만이 아니라 posterior calibration과 robot-control utility를 연결하는 *차별화된 학습 목적함수* 검증. 예: posterior를 고정한 solver의 rollout loss(ψ deviation, continuity, recovery)와 확률 calibration을 함께 평가/학습. 미분 가능한 constrained layer가 불안정하면 black-box/teacher-distillation 대안. 일반화가 개선되지 않으면 학습 기여로 과장 금지.

## 2. 현재 근거/미완료 분리 (2026-11-04)

**완료·기록됨**
- [x] NERO URDF SRS, POE FK 및 해석적 IK (1000 random q round-trip), ψ feasible arcs.
- [x] joint-limit wrist feasibility 분석: ready 방향 근처 wrist pose 가능 약 95–98%; elbow-down |ψ|≤30° 가능 약 15–19% (가정된 pose 분포이며 인간 실측 분포 **아님**).
- [x] Windows/WSL AprilTag 손목 추종 코드 프로토타입 (정량 성능 측정 **아님**).
- [x] Jetson AGX Orin + ROS 2 Humble + D415/D435i 동시 스트림 (~14.6/14.4 FPS color) 및 ArUco extrinsic 추정 보고.

**아직 미검증**
- [ ] D415 단독 markerless human 3D pose, 손목 SE(3), ψ 및 visibility/depth validity.
- [ ] 캘리브레이션 GT 정확도; 기존 ArUco 카메라간 상대변환 z 표준편차 44.7mm는 재검증 필요.
- [ ] Feasible-MAP solver, learned belief, baselines 및 공정 비교.
- [ ] 안전 레이어 일원화 후 NERO 실제 폐루프 실험.
- [ ] 실기 REVO2 Pro 손가락 제어 성능. 팔 Retargeting 메인 claim과 분리.

기존 관련 문서: [02_kinematics_validation.md](docs/02_kinematics_validation.md), [11.03_exp/README.md](11.03_exp/README.md), [01_literature_novelty_audit.md](docs/01_literature_novelty_audit.md).

## 3. 일정 및 단계별 deliverable

| 기간 | 단계·담당 제안 | 작업 | 완료 산출물 |
|---|---|---|---|
| **W1–W2 (11/02–11/15)** | P0/기구학 Track B | 단일 D415 입력 확정, D435i는 GT 전용. URDF↔실기 command path/limit 확인, ψ singularity/branch 경계 테스트, 안전 인터페이스, IRB 확인, 관련 문헌 재검증 | hardware_inventory, frame conventions, URDF/unit tests, 분업 리뷰 |
| **W2–W4 (11/09–11/29)** | P1 Oracle Solver Track B | 가짜 human trajectory+oracle covariance, ψ sweep/arm stretch/J6 infeasible/occlusion/reappearance. SEW, DLS, adaptive weighting, filter vs oracle Feasible-MAP. **G1** | oracle benchmark CSV/figures, runtime, baseline-failure report |
| **W1–W5 (11/02–12/06) 병렬** | V Vision/GT Track A | Jetson D415 RGB+depth, keypoints→3D, depth validity, torso frame, occlusion flags, timestamps/rosbag. D435i + fiducials **GT 수집 전용**, 독립 정확도 검증 | markerless inference graph, GT calibration audit, synchronized logs |
| **W4–W7 (11/23–12/20)** | P2 Occlusion Benchmark Track A | 4–6명 파일럿, 6 scenario, occlusion length별, GT-quality mask, subject-held-out splits, real ψ error vs h/visibility, noisefit. **G2a** | benchmark-v1, scripts, E1, dataset card + split manifest |
| **W6–W10 (12/07–01/10)** | P3 Belief Learning Track A | 기하 filter→unimodal probabilistic→multimodal 필요성 순서로 확장. landmark covariance calibration, NLL+reprojection/temporal, GT uncertainty, unseen-person. **G2b** | posterior model, calibration/reliability plots, ablations |
| **W8–W12 (12/21–01/24)** | P4 Integrated Solver Track B | p_H(z|o) 입력 + feasible set + branch continuity + short-horizon; wrist infeasible task-space relaxation; recorded replay, 2×2 component ablation. **G3** | common benchmark, solver code, 5-seed sim results, computation P50/P95/P99 |
| **W10–W14 (01/04–02/07)** | P5 Real NERO Track B | 저속 dry-run→recorded playback→live D415. Unified safety limits, watchdog, E-stop, dynamic TF check, execution latency. REVO2 Pro는 팔 baseline 통과 후 부가 demo | safety audit, NERO closed-loop plots/video, real-world trial logs |
| **W12–W16 (01/18–02/21)** | P6 Human Study A+B | 초기 3–5 task pilot 후 가능한 범위 6–10 task, 8–12명, counterbalanced 3–4 methods, 2+ repeats; 미학습 사용자, occlusion 유형/시간 stratification. **G4** | task SR/time/ψ/jerk/recovery, per-user paired CI, negative results |
| **W14–W17 (02/01–02/28)** | P7 IROS Paper A+B | 논문 도식·실험표·비디오·failure analysis, draft→internal review→artifact freeze. 2027-03-01 마감 전 최종 검수 | IROS 2027 manuscript, reproducibility supplement, video |
| **W18+ (03/01~; CoRL 선택)** | P8 CoRL deepening | robot-aware trainable belief, cross-camera/cross-user/cross-robot URDF sim, 2nd dataset, out-of-distribution occlusion, learning baselines·scaling | 학습 목적함수 기여, 일반화 curve, 학습 데이터 효율, CoRL 원고 (공식 CFP 확인) |

Track A: Vision/Data/Learning, Track B: Kinematics/Solver/Robot. 제안 담당은 [기존 roadmap 역할표]에 따르되 [decision.md](decision.md)에서 합의 확인. 시험 셋 공유 및 baseline 정의는 공동 리뷰.

## 4. Occlusion benchmark 프로토콜 (사전 등록)

**실기 추론 입력**: D415 **단일 카메라**의 RGB-D와 이전 관측만, no gloves, no marker, no second-view inference. GT에 쓰는 D435i/마커 라벨은 train/eval separation; 운용 시 사용하지 않는다.

- 상황 O0 정상 관측, O1 elbow self-occlusion, O2 wrist/hand occlusion, O3 arm fully extended/near singular, O4 torso crossing/depth confusion, O5 fast motion during loss, O6 reversal during occlusion, O7 reappearance.
- 지속 시간 버킷 예시 0.2/0.5/1/2초 이상 및 연속 구간: 프레임레이트가 15fps 안팎임을 고려하고 유효 프레임 수 기록. 장기 손목 소실은 '맞히는 것' 대신 안전 hold/stop 평가.
- 파일럿 4–6명 × 10–15분, 이후 학습·검증·시험 사용자 별도. Train/Test에는 동일 사람이 섞이지 않도록 subject-level split; 동일 영상의 synthetic mask 변형을 다른 split에 넣지 않음. 결과 신뢰 구간은 frame 독립 가정이 아니라 participant/episode cluster bootstrap.
- GT: 외부 camera marker/rig는 **참값 생성에만 사용**, marker bone attachment offset, timestamp alignment, reprojection/rigid ruler error를 검증. GT가 보이지 않는 프레임은 valid-GT mask, 추정 정답을 주 결과에서 제외.
- 수집 윤리: IRB/동의·영상 저장/익명화 정책 확인 전 사람 데이터 수집 시작하지 않음.
- motion distribution: synthetic uniform wrist poses로 도출한 95–98%와 실제 인간 빈도는 분리 보고.
- 오차 요인 oracle substitution: vision-only, robot embodiment-only, numerical solver, real execution을 별도 산출; 오차는 단순 합으로 취급하지 않음.

### Baselines (같은 입력/안전 제약/재계획 주기)

B0 Wrist IK + posture home; B1 SEW direction retargeting; B2 adaptive elbow offset/weight; B3 IK + One-Euro; B4 IK + Kalman/constant velocity; B5 AKC-style bone-length geometric occlusion correction + common solver; B6 learned residual MLP/GRU 또는 learned ψ; B7 unimodal uncertainty-weighted constrained IK; B8 **mixture-belief → ordinary IK**(순차식); P **proposed belief-constrained Feasible-MAP**. 논문이 공개한 코드가 있으면 재사용, 없으면 재구현 가정 명기.

### 필수 ablations (각각 1개만 달라야 함)

A0 full; A1 no occlusion detection; A2 fixed/uncalibrated Σ; A3 single ψ mode (mixture 제거); A4 no robot-feasible coupling (belief + ordinary IK); A5 no temporal/branch continuity; A6 no pose relaxation for infeasible wrist; A7 no learned bias; CoRL-only A8 no downstream-control loss. 모든 조건 안전 장벽 동일. 더 강한 간단한 baseline이 있으면 이를 우선 포함.

### 공개 가능한 핵심 지표

- **Perception**: S/E/W 3D position error [mm], wrist orientation [deg], circular ψ MAE/P90 by elbow offset h and occlusion, 68/95% coverage, NLL, ECE, occlusion recall, missed-valid-depth rate.
- **Solver**: wrist position/orientation target error, robot ψ deviation when human request *feasible*; infeasible에선 best-feasible oracle regret, joint jerk, mode/branch switch count, deadline miss, collision/joint/velocity violations, solver failure, P50/P95/P99 ms.
- **Recovery**: max joint jump / peak joint speed during first 1s after reacquisition, delay to stable wrist tracking, false extrapolation during wrist dropout, intervention/hold fraction.
- **Human task**: per-task/per-user success, completion time (timeout included), safety intervention, number of trials. Compare same tasks/workspace/scales/training time, counterbalanced order, 95% paired confidence intervals, full failures/videos.
- **Realtime**: D415 input FPS/depth availability, inference p95, robot command p95, full camera-to-actuation p95. Hardware limits rather than nominal ROS timer settings.

## 5. Milestone Gates (목표치는 제안값; 성공 기록 아님)

| Gate | 시점 | 사전등록 통과 기준/질문 | No-Go 조치 |
|---|---|---|---|
| **G0 Hardware & GT** | W3 | D415 단독 pipeline 안정, 정확한 timestamp/TF; GT repeatability가 목표 retargeting gain보다 충분히 작고 independent ruler/marker 검증 통과 | GT rig 재설계; 정확성 근거 없는 3D claim 금지 |
| **G1 Oracle value** | W4 | 동일 safety에서 oracle Feasible-MAP가 strongest baseline 대비 wrist non-inferiority(예: median +5mm/5° 이내)와 ψ P90/jerk/recovery 실질 개선(초기 지향 ≥15–20%) | weighting만으로 충분한지 원인 분해; 계획된 solver 결합 수정 |
| **G2a Data hypothesis** | W7 | self-occlusion/low-h에서 실제 ψ/3D 오차가 구간별로 달라짐; GT mask와 subject split 정확 | Occlusion 문제를 wrist-only 또는 임계 상황 중심으로 좁힘 |
| **G2b Calibrated belief** | W10 | coverage 68/95%의 목표 대비 ±10%p 내(제안), NLL/precision 개선, error–σ Spearman>0.3(제안), multi-mode가 필요할 때만 이를 유지 | 작고 안정적인 filter로 fallback, learned component의 claim 재검토 |
| **G3 Solver benefit** | W12 | fixed safety + same inputs로 강한 B7/B8 대비 occlusion/near-limit subgroup에서 개선; 5 seed simulation + clustered CI, avg/P95/P99 real-time 가능 | 단순 가중 IK와 동등하면 핵심 알고리즘 재설계 후 venue 조정 |
| **G4 Real manipulation** | W15–W16 | 위험한 명령 0건, watchdog·E-stop 검증, 미학습 조작자 실험에서 유의미한 task SR/continuity/recovery 이득. +10 percentage points SR은 **도전 목표**, 결과나 게재조건 아님 | IROS 본문 주장 축소, CoRL 확장 실험에 자원 집중 |
| **G5 Venue decision** | W16 | IROS: 새 solver + solid real eval + convincing ablation. CoRL: learned component 필수·generalization/out-of-distribution/control-aware learning 효과 명확 | 강한 쪽 1차 투고, 무리한 CoRL learning claim 금지 |

### 성공 정의에서 지켜야 할 것

- 관절 violation 0만으로 방법이 우수하다고 주장하지 않는다. **모든 baseline에 동일한 별도 safety shield** 적용.
- multimodal belief 자체가 기여가 되려면 unimodal·filter와의 의미 있는 차이를 증명한다. 모드 평균 붕괴, human belief를 robot constraints로 잘못 왜곡하는 오류를 방지.
- Occlusion 중 손목 정보가 없을 때 끝없이 예측하지 않는다. confidence/time-to-stop 정책과 false-continue 빈도를 측정한다.
- 통계적으로 독립적인 표본은 비디오 프레임 수가 아니라 subject·trial; 반복 측정 계층적 통계로 보고한다.
- GT가 부정확하면 정답 정확도에 대한 논문 결론을 보류한다. 안전/지연/실패 결과는 숨기지 않는다.

## 6. 구현 구획 및 공통 API

공유 코드 제안: `share/{vision,belief,solver,benchmark,robot,safety,configs,results}` (아직 생성하지 않음).

- Vision → Belief: `Observation(timestamp, frame_id, S/E/W xyz, hand R, visibility flags, depth-valid flags, confidence, 3D covariance, torso_frame)`
- Belief → Solver: `HumanBelief(t, wrist_mean_SE3, wrist_cov_SE3, psi_modes={means, concentrations, weights}, occlusion_state, age_ms, calibration_status)`
- Solver → Safety → Robot: `RobotCommand(q_target, qdot_target?, target_timestamp, confidence, hold_reason)`, command limits independent of learned model.
- 모든 모듈은 `base_link`, `camera_color_optical_frame`, `torso`, `shoulder` 좌표계를 정의하고 units [m], [rad], timestamp freshness 명시.
- D435i가 NERO wrist에 부착되어 움직이는 경우 D415↔D435i transform은 **static TF로 고정하지 않음**. 본 연구 실험에서는 GT 수집을 위해 D435i 고정 배치 권장.
- 재현성: config+seed+git SHA+dataset manifest+GT quality flags+split+results.json+videos를 실험마다 저장. 원본 사람 영상과 개인식별 정보는 Git에 커밋 금지.
- 공개 전 독립 run-from-clean-clone, reproducibility checklist, license/인간 데이터 동의 재확인.

## 7. 우선 실행 업무 (두 사람 병렬)

**Track A — Vision / GT / Belief (먼저 실행):**
1. D415 단독 3D S/E/W+hand R ROS 2 node, aligned-depth quality와 occlusion flag, 15fps 가까운 실제 처리량 보고.
2. D435i+marker GT rig를 separate calibration script로 재측정, 불확실한 GT 프레임 mask, timestamp alignment.
3. O0–O7 최소 파일럿 시퀀스 후 E1 (h·가림별 ψ 분산) 및 E2(uncertainty calibration)부터.
4. 소형 posterior baseline (geometric constant-velocity + KF), 필요성 증명 후 temporal model 학습.

**Track B — Kinematics / Solver / Robot (먼저 실행):**
1. 현재 해석적 IK test 유지, singular/branch stress + wrist-infeasible target relaxation + collision/velocity limits.
2. oracle covariance / multimodal ψ simulator 생성; same safety baseline B0–B4/B7/B8 구현 및 G1 데이터.
3. Feasible-MAP + branch continuity + dropout hold/reacquisition; P95 latency profiler.
4. dry-run→recorded replay→저속 실기 순서; safety shield를 메인 제어 명령 밖에서 별도 유지.

**첫 병합 조건:** 합의된 `HumanBelief` 인터페이스, GT validity mask, reproducible occlusion sequence, 동일 baseline safety profile, G1 수치. 두 트랙 보고를 동일 GitHub PR 안에서 동료 검토한다.

## 8. 리스크와 전략

| 리스크 | 대응 |
|---|---|
| "confidence-weighted IK에 불과" | 동일 입력·안전 제약 아래 B7/B8과 공동 목적/branch/recovery 비교; 차별성 실패 시 목표와 수식 수정 |
| 기존 AKC와 중복 | 3D depth correction만으로 기여 주장 금지, robot control-level outcome 검증 |
| 2026 human-like redundancy 방식과 중복 | 동일 elbow-offset gating baseline 및 near-straight stress 포함 |
| 실제 카메라 오류 / 15Hz / GT 저품질 | frame-age/latency 분리, GT rig audit, camera-to-action p95, 안전 정지; 두 카메라 inference 주장 금지 |
| 2027-03-01 IROS 마감 | W10~W12 결과가 강하지 않으면 실험 범위/학회 선택 조정; 안전한 본 실험 시간 우선 |
| CoRL learning contribution 부족 | downstream-control-aware training, subject/camera/robot generalization, learned-vs-heuristic ablation 및 학습 데이터 효율 |
| 빠른 관절 움직임·실기 안전 | 별도 safety shield, work envelope, fail-safe stop, E-stop 실험 승인 없이 실기 금지 |
| 참가자/IRB/개인정보 | IRB/동의 범위 먼저 확인, 개인 정보·얼굴 영상 비공개 원칙, 공개 dataset은 별도 승인 |

**본 문서는 실험 계획 초안이다.** 미달 성과를 성공으로 포장하지 않고, 연구 주제를 바꾸지 않는 범위에서 모델 규모·핵심 식별 실험을 조정한다.
