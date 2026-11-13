# 04. Proposed Solver — Repeatable Canonical Feasible-MAP Retargeting (Design Draft)

> 작성: 2026-11-13 · **설계/검증 계획 (구현 또는 성능 입증 아님)** · 상위 문서: [연구 로드맵](../roadmap.md), [EX00–EX10](experiment_roadmap.md), [기구학 검증](02_kinematics_validation.md)
>
> 변경 목적: **동일한 인간 자세 분포가 들어와도 NERO 7-DoF의 IK 다중해/시드/이동 경로 때문에 기준 관절각이 달라지는 문제를 명시적으로 방지**한다. 기존 occlusion-aware human belief, feasibility-constrained solver, 실기 safety shield의 역할은 유지한다.

## 0. 반드시 구분할 세 종류의 동일성

1. **손목 위치 동일**: 사람 손목 3D 좌표만 같음. 팔꿈치 위치, 손목 방향, 인간 의도는 서로 다를 수 있으므로 **로봇 관절각 동일을 강제하지 않는다**.
2. **Human belief 동일**: 동일한 좌표계·캘리브레이션·정규화 및 로봇 설정에서 입력 p_H(T_w, ψ, v | o_1:t)의 전체 표현(모드·가중치·공분산·undefined mask 포함)이 동일함. 이 경우 **canonical target q_ref는 동일**해야 한다.
3. **실제 명령 동일**: 각 프레임의 q_cmd는 q_measured, 시간, 속도 제한, 충돌·watchdog 상태에 따라 달라질 수 있다. **q_ref와 q_cmd를 혼동하지 않는다**. 서로 다른 출발점의 즉시 q_cmd 일치는 요구할 수 없으며, 안전한 조건에서 수렴 후 최종 자세의 일관성을 검증한다.

**정식 요구사항 R-CANON** (고정된 model/URDF/TF/limits/policy/version, 유효하고 가능한 목표 영역 내):
- 동일 belief p_H이면 canonical solver의 출력은 동일 q_ref와 동일 mode/branch ID.
- 동일한 관측의 반복 호출은 수치적으로 재현 가능해야 하며 입력 순서·warm start·난수 seed·이전 q 때문에 결과가 바뀌어서는 안 된다.
- 거의 같은 belief는 특이점·feasible 집합 위상 변화·mode 동점 영역을 제외하고 **작은 q_ref 변화**를 목표로 한다. 모든 작업공간에서 연속·유일성 보장 불가: 예외를 사전에 정의하고 실패/전환으로 기록한다.
- 관절 명령 q_cmd에는 독립 safety shield 적용. Safety에서 명령을 수정하더라도 q_ref를 몰래 바꾸거나 사람 posterior를 재정의하지 않는다.

## 1. 두 층의 인터페이스: 기준 매핑과 실행 궤적

### (A) Canonical reference solver (상태 독립적인 기준 관절 구성)

`@
q_ref, branch_id, mode_id, quality = CanonicalFeasibleMAP(human_belief, robot_model, fixed_calibration, fixed_policy)
`@

- 입력: wrist position + SO(3) orientation distribution, ψ distribution (필요 시 mixture-von-Mises), validity, timestamp quality, canonical torso/robot frames.
- 주어진 full belief와 고정된 robot configuration에서 **deterministic optimization/branch selection** 수행. p_H는 로봇 제약으로 재학습하거나 잘라내지 않는다.
- 인간 의도에 맞춘 기대 task loss, wrist/ψ 불확실성, robot joint limits, reachability, collision, singularity 및 기준 자세 prior를 사용한다.
- q_prev나 current q로 q_ref 후보를 정렬하지 않는다. 동일 비용/허용 오차의 후보에는 **명시적 전역 정렬 키**(cost tolerance bucket → preset branch priority → home distance → 정규화된 q lexicographic)를 적용한다.
- 초기 해 탐색은 NERO URDF의 **feasible ψ arc + 최대 8개 analytic IK branch**를 우선 이용하되, 좁은 arc/수치 특이점/SE(3) infeasible은 deterministic fallback (고정 시드·고정 restart 목록·고정 tol)로 처리한다.
- 무작위 posterior sample 하나를 선택하는 방식 금지. Mixture는 mode를 열거/통합하여 objective에 반영하고 최종 canonical output은 하나의 관절해를 deterministic하게 선택.
- 완전히 팔을 펴 ψ가 정의 불안정한 구간에서 관측 ψ를 가짜 확정값으로 만들지 않는다(ψ undefined/weak term 처리).
- feasible human wrist pose 자체가 존재하지 않으면 task-space pose relaxation을 최적화하되 결과가 인간 posterior의 보정 GT인 것처럼 표시하지 않는다.

개념적 형태:

\[
q_{\mathrm{ref}} = \operatorname{DetTieBreak}\left(
 \underset{q\in \mathcal{F}_{\mathrm{static}}}{\arg\min}
 \left\{\mathbb{E}_{z\sim p_H}[\ell_{\mathrm{wrist}}(q,z)+\ell_{\psi}(q,z)]
 +\lambda_h \|q-q_{\mathrm{home}}\|^2+\lambda_s\ell_{\mathrm{sing}}(q)\right\}\right)
\]

`DetTieBreak`는 global optimum 존재나 near-tie 부근 연속성을 자동으로 보장하지 않는다. 유한 후보 열거의 coverage, 최적성 gap, branch boundary 예외를 별도 측정한다. q_home는 **기준 자세의 의미를 가진 고정값**이고 q_prev가 아니다.

### (B) Trajectory tracking and safety (상태 의존적인 명령)

`@
q_cmd, status, reason = TrackSafe(q_ref, q_measured, qdot_measured, dt, obstacle_state, input_age)
`@

- q_cmd는 joint velocity/acceleration/jerk limits, collision, feasibility, sensor age, recovery ramp, watchdog, E-stop에 따라 현재 상태 기반으로 달라질 수 있다.
- canonical target이 branch boundary를 통과하거나 안전상 위험할 때 **속도를 제한하거나, Hold/Stop하거나, 명시된 안전 경로로 전환**한다. 이 단계의 branch hysteresis는 q_cmd에만 적용하며 canonical 기준 q_ref 자체를 바꾸지 않는다.
- 장기 손목 dropout 시 canonical 기준을 무리하게 갱신하지 않으며 안전 Hold/Stop을 우선한다. 가림 전후 q_ref의 변동과 실행 q_cmd의 변동을 **별도 기록**한다.

## 2. 난점과 설계 결정

| 위험 | 방지 규약 | 기록/ablation |
|---|---|---|
| 손목만 같은데 팔꿈치 의도가 다름 | wrist xyz-only consistency를 **요구하지 않음**; full p_H 기준 | wrist-only vs full-pose 반복실험 |
| 7-DoF의 다수 ψ / branch | feasible arc·모드 후보 열거, 고정 기준 비용과 tie-break | selected ψ, branch ID, 후보 수, best-vs-second cost gap |
| warm start·random restart에 따른 비결정성 | canonical 모듈에는 q_prev 금지; 고정된 탐색 순서와 solver tolerance | 같은 입력 100회 / 프로세스 재시작 / reversed input order |
| 가림 후 재인식/작은 RGB-D 노이즈 | posterior calibration, 필요 시 mixture·신뢰도 기반 Hold; near-tie 경계 별도 분석 | canonical Δq / command jump / time-to-settle |
| canonical map의 위상적/수치적 불연속 | 전 공간 smoothness 주장 금지, 해당 구간 flag + tracking safe transition | singularity, branch boundary, infeasible boundary |
| q_ref는 같지만 로봇 시작점이 다름 | q_cmd는 상태 의존; 충분한 안전한 수렴 시간 후 최종 q 비교 | path-start consistency / convergence fraction |
| canonical home prior가 인간 팔꿈치 의도 희생 | human ψ fidelity와 repeatability의 trade-off 측정 | λ_home sweep, ψ P90, feasible oracle regret |
| 다른 method가 canonical 규칙만 없어서 손해 | baseline에도 동일한 canonical tie-break/safety variants 제공 | fair baseline vs proposed coupled objective |

**주의:** q_ref에 현재 로봇의 q_t-1을 넣으면 동일한 p_H에서 서로 다른 q_ref를 내는 것을 허용하므로 R-CANON과 충돌한다. 기존 roadmap의 'short-horizon continuity + branch hysteresis'는 **실행 계층**으로 옮기고, belief–feasibility coupled objective는 기준 매핑 계층에 남긴다.

## 3. 공통 API / 로그 계약 제안

- `HumanBelief`: posterior representation, wrist SE(3), ψ modes/weights/κ, covariances, occlusion_state, undefined_psi, frame_id, calibration_id, model_version, timestamps.
- `CanonicalTarget`: q_ref[7] rad, canonical_psi rad or undefined, branch_id, mode_id, feasible, pose_relaxed, objective_terms, cost_gap, solver_status, deterministic_config_hash, canonical_model_version.
- `RobotCommand`: q_cmd[7], timestamp, tracking_status (TRACK/HOLD/SAFE_STOP), hold_reason, q_measured, safety_override, applied_speed_limit.
- 모든 좌표계/단위, pose mapping scale, robot model SHA, tolerance, optimizer configuration, mode ordering을 재현 가능한 설정에 포함.
- canonical q_ref에 대한 반복성 측정에서 GT tag/glove/D435i를 runtime feature로 사용 금지(offline GT만 허용).

## 4. 추가 실험 프로토콜: EX04-R / EX07-R

| ID | 입력/조건 | 1차 지표 | 기대 확인 / 예외 |
|---|---|---|---|
| R1 Same-Distribution Determinism | 동일 oracle belief와 고정 config를 N≥100회, 순서·프로세스 재시작을 달리해 반복 | q_ref max-norm diff [rad], branch/mode mismatch | 동일 환경에서 q_ref 오차 ≤1e-6 rad 목표; 불일치는 실패 기록 |
| R2 Cross-Path Target Consistency | 동일 최종 human belief에 서로 다른 과거 동작 경로와 q_start로 도달 | q_ref diff, 수렴 후 q_measured diff, convergence rate | canonical q_ref 동일; q_cmd는 즉시 동일할 필요 없음 |
| R3 Noise Robustness | 같은 실제 인간 pose에 서로 다른 RGB-D noise/uncertainty/seed 관측 | canonical Δq P50/P95/P99, ψ deviation, task wrist error | noise level별 곡선. 무조건 동일 q를 강제하지 않음 |
| R4 Occlusion/Reappearance | EX04 S4/S5/S6에 elbow/wrist dropout, 방향 반전, 재인식 | branch flips, max command jump, recovery time, unsafe extrapolation | hold 비율 포함, 장기 wrist-loss 안전 제약 |
| R5 Feasibility / Boundary | J6 limit, 좁은 ψ arc, unreachable wrist, near singularity | target repeatability, oracle regret, failure/hold, cost gap | 안전 가능한 경우만 수렴; 전역 연속성 불가능 영역 flag |
| R6 Same-Wrist/Different-Elbow | 같은 6D wrist이지만 서로 다른 ψ posterior | mode/branch 선택, ψ fidelity | **동일 q 강제 금지**; 사람 팔꿈치 의도 구별 여부 검증 |

**공정 비교**: 기존 B0~B8 및 P에 대해 동일 observed belief/URDF/safety/timeout/warmstart budget 사용. 추가 baseline은 (a) home-posture + deterministic IK, (b) naive previous-q seeded IK, (c) confidence-weighted IK + 동일 canonical tie-break, (d) sequential mixture-belief→canonical IK, (e) proposed canonical coupled Feasible-MAP. Oracle-Σ는 upper bound로 구분. 학습된 posterior 실험은 EX06/EX07 이후.

**Ablation**: canonical tie-break 제거, q_prev를 reference solver 안에 직접 사용(비일관 baseline), single ψ mode, home prior 제거, belief-feasible coupling 제거, tracking hysteresis 제거(단 safety shield 동일), q_ref/q_cmd 분리 제거(명시적 비권장 비교).

**제안 Gate (새 목표, 측정 결과 아님)**:
- R1: 동일 소프트웨어 환경에서 동일 입력 N=100 반복 시 max-norm q_ref diff ≤ 1e-6 rad, selected branch/mode mismatches = 0 (수치 tolerance와 deterministic execution backend 고정).
- R2: 서로 다른 접근 경로/초기 q에 대해 **canonical q_ref 동일**. 실제 robot endpoint 수렴은 사전 정의한 joint tolerance/시간/안전 조건에서만 판단.
- R3~R5: 기존 B7/B8와 canonicalized baseline 대비 같은 wrist accuracy/safety 하에서 q_ref instability, branch flips, recovery/jerk 개선. 효과 크기는 파일럿 후 사전 등록하며 임의 성공 수치를 만들지 않음.
- 새로운 regularizer만으로 수치 개선을 얻더라도 실제 feasible ψ 보존과 real task SR이 희생되면 주 기여로 주장하지 않음.

## 5. 작업 순서 및 현재 상태

1. **EX03 / EX04 (우선)**: deterministic candidate enumeration / tie-break / R1–R2 oracle test → 같은 입력에서 q_ref가 달라지는 root-cause 확인.
2. **EX04 확장**: R3–R6 noise, occlusion, near-limit stress + canonicalized strong baselines + trade-off curves.
3. **EX06 / EX07**: learned human posterior + coupled solver, posterior calibration / solver gap 분리 + R1–R6 재검증.
4. **EX08 / EX09**: q_ref / q_cmd 별도 로그, 실제 경로 의존성·수렴·Task SR·안전 검증.
5. **CoRL 확장**: control-aware learned posterior가 repeatability만 높이는 게 아니라 ψ fidelity, OOD 조작 성능을 개선하는지 추가.

**2026-11-13 상태**: 본 문서는 **신규 solver 요구사항과 실험 계획**이다. canonical solver 구현 완료, R1–R6 통과, NERO 실기 성공을 의미하지 않는다.
