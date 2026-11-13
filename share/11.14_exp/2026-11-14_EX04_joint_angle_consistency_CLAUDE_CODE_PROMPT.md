# 2026-11-14 — Claude Code 실행 프롬프트: EX04 관절각 재현성·개선 실험

> **이 문서는 실행 지시용 프롬프트이며 실험 결과가 아니다.** 작성일: 2026-11-14 (KST)
> 관련 기록: [2026-11-14 로그](2026_11_14_log.txt) · [Canonical Solver 요구사항](../docs/04_proposed_solver.md) · [EX00–EX10 계획](../docs/experiment_roadmap.md)
> 대상: Jetson AGX Orin의 기존 `~/teleoperation_repo` 로컬 작업 환경. 실기 NERO 명령 금지, **시뮬레이션/오프라인 평가만 수행**.

## Claude Code에 그대로 붙여 넣을 프롬프트

당신은 `dldbdlf/teleoperation` 프로젝트의 로봇 Retargeting Solver 실험 담당 연구 엔지니어이다. **코드 수정 → 테스트 → EX04 데이터로 정량 실험 → 실패 원인 분석 → 보고서/코드 GitHub 커밋 및 push**까지 수행하라. 보고 수치를 꾸미지 말고 기존 결과와 새 결과를 명확히 구분하라. 실기 로봇 제어는 절대 실행하지 말라.

### 0. 프로젝트 고정 조건과 현재 상태

- 단일 RealSense D415 RGB-D 온라인 입력; D435i/AprilTag/장갑은 학습·평가 GT 전용. 이번 EX04는 합성 oracle belief만 사용.
- 대상 NERO 7-DoF 공식 URDF, 15Hz 관측 / 100Hz 제어, 동일한 `--psi-map mirror`, speed cap `pi rad/s`, wrist hold `0.5s`, output interpolation ON. 비교군 모두 동일 안전 계층, 동일 데이터, 동일 초기 조건을 사용.
- 2026-11-14 기록: 생성 899/900 중 **894개 trajectory 공통 평가셋으로 B0–B8 평가 완료**. S6/seed2/index14 생성 오류 1건. 평가에 사용되지 않은 생성 파일 5개의 이유를 manifest/필터링 기준에서 확인하고 명시하되 **현재 비교의 894개 공통집합은 바꾸지 말 것**.
- 11/14 로그상 제안 P (Oracle-Sigma Feasible-MAP 초기 버전)는 B7과 사실상 동일한 비용함수를 사용하되 `lam_t=lam0*max(1,(sigma_pos/0.015)^2)`로 이전 q에 대한 연속성 가중치만 가변. S5, 2초 손목 가림, seed0 idx0 **단일 시퀀스**에서 B7 recovery jump 40.85° → P 12.25°, jerk 6053 → 5460. **전체 894개 P 비교·G1은 이 기록만으로 통과 판단 불가**.
- 원격 main의 `share/sim/baselines.py`에는 현재 `FeasibleMAP` 클래스가 없고 `run_baselines.py`의 METHODS에는 P가 없으며 BLAS 관련 코드도 반영되지 않았을 수 있다. **로컬 미커밋 작업·별도 브랜치·로그·기존 출력물을 먼저 확인하라. 로컬 변경을 강제 pull/reset/checkout으로 덮어쓰지 말라.** GitHub main과 로컬 diff를 저장하고, 이미 있는 P/로그를 보존·재사용하며 필요한 수정만 수행.

### 1. 가장 먼저 해야 할 작업: 결과/코드 재현성 감사

1. `git status -sb`, `git log --oneline -n 10`, 현재 브랜치, 코드 diff, 테스트 상태, 기존 `outputs/baselines_P/per_traj.csv`, `summary.md`, `~/run_baselines.log`, synthetic manifest, 894개 공통 파일 목록을 확인해 표로 기록.
2. P 전체 실행이 이미 끝났다면 다시 894개를 무조건 계산하지 말고 **완료된 실제 CSV/요약을 우선 분석**. 미완료라면 resume을 통해 빠진 단위만 실행하라. 방법별 비교는 동일 894개 파일/seed에 한정.
3. 8-worker Jetson의 BLAS 과도 스레드 사용을 방지: Python 시작 시 `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1`을 **numpy import 전에** 적용하고 실제 threadpool 설정을 확인. 기존 설정값이 1보다 크면 `setdefault`만으로 덮이지 않을 수 있으므로 실행 환경도 검증.
4. 오래 걸리는 실행은 **20 trajectory마다 진행률/경과/ETA를 flush 출력하고 CSV/manifest를 atomic write + resume 가능**하게 하라. 진행 중 중단되어도 이미 계산한 항목은 보존. 데이터·시드·설정 변경 시 캐시를 섞지 말 것.

### 2. 가장 중요한 문제: 동일 Human Belief → 동일 기준 관절각

기존 P의 `||q-q_prev||^2` 항을 canonical reference solver에 직접 쓰면, 동일한 belief에서도 진입 경로에 따라 서로 다른 q가 나올 수 있다. 이를 해결하되 P의 좋은 occlusion recovery 특성을 잃지 않도록 **기준 자세와 실행 추적을 분리**한다.

- 새 `CanonicalTarget`: `q_ref[7]`, `psi_ref`, `branch_id`, `mode_id`, `feasible`, `cost_gap`, `solver_config_hash`.
- `q_ref=f(p_H,URDF,fixed_TF,fixed_policy)`: 현재 `q_prev`, random seed, 호출 순서, warm-start에 **독립**. 동일 full posterior/동일 robot model과 설정에서 동일 q_ref/branch. robot feasible ψ arcs, 최대 8개 IK branch 열거, 고정 home prior, 일관된 비용/후보 정렬/tie-break 사용. 모든 후보의 feasibility를 검증. 수치 local minima 또는 후보 누락도 별도 기록.
- 실제 제어는 `q_cmd=TrackSafe(q_ref,q_measured,dt,occlusion_state,age)`로 분리. 동일 `q_ref`여도 서로 다른 `q_measured`에서는 속도 제한/hold/충돌 안전 때문에 `q_cmd`가 달라도 된다.
- 사람의 posterior `p_H`를 로봇 관절 한계로 잘라 인간 정답을 바꾸지 말 것. **동일 wrist XYZ나 SE(3)만으로 동일 관절을 강제하지 말 것**: 인간 팔꿈치 ψ 의도가 다르면 다른 q_ref가 맞을 수 있다.
- 팔이 펴진 상태(ψ undefined), 좁은 feasible ψ arc, joint J6 limit, 손목 자체 infeasible, near singularity와 cost tie에서는 유일성과 전역 연속성이 불가능할 수 있다. 이를 flag/hold/safe transition으로 처리하고 '모든 입력에서 연속'이라고 주장하지 말 것.

### 3. 반드시 비교할 방법(공정 비교)

- **B7 / B8** 원본.
- **P-old**: 11/14 로그의 기존 adaptive-`lam_t` 방식. 그 자체로도 가치가 있으므로 제거하거나 새 방법과 혼동하지 말 것.
- **B7-canonical**: B7의 belief objective + 고정 home/결정적 IK-branch tie-break. 단순 결정성으로 얻는 효과의 강한 baseline.
- **P-canonical**: 명시적 canonical `q_ref` + 별도 stateful tracking/occlusion-adaptive smoothness. 제안 방법과 canonicalized baseline 간 차이를 계산.
- **Ablation**: (a) no canonical tie-break, (b) q_prev 포함 reference, (c) fixed vs adaptive continuity, (d) no branch-aware tracking, (e) ψ term off, (f) home prior sweep. 각각 한 번에 한 조건만 바꿀 것. 안전 shield 및 관측은 모두 동일.
- 새 방법의 난도 때문에 B0–B6 결과를 모두 재실행할 필요는 없다. 이미 동일 데이터/설정으로 저장된 결과를 재사용하되 공정성이 달라지면 별도로 재평가.

### 4. 실행할 시험 R1–R6 (기존 EX04 894개 공통 데이터 + 소형 결정성 fixture)

| ID | 시험 | 핵심 측정 |
|---|---|---|
| R1 | 완전히 동일한 full belief + 고정 URDF/TF/policy 100회 호출, 호출 순서 역전/다른 프로세스에서 반복 | `max||Δq_ref||∞` [rad], branch/mode mismatch. **제안 pass** ≤1e-6 rad, mismatch 0 |
| R2 | 다른 q_start / 이동 궤적에서 동일 belief 종점으로 접근 | canonical target 일치, 실제 q_cmd의 제약 준수 및 수렴 후 q 차이 별도 |
| R3 | 같은 GT 자세에 서로 다른 RGB-D 노이즈/불확실성 관측 주입 | q_ref P50/P95/P99 편차, wrist/ψ 오차 및 noise-to-q sensitivity |
| R4 | S4/S5/S6 가림(0.2/0.5/1/2초) 및 재관측·방향반전 | requested recovery jump, jerk RMS, branch switch, hold fraction, recovery time |
| R5 | S2/S3, J6 limit, near-straight, feasible arc 경계/목표 infeasible | feasibility violation, best-feasible regret, fallbacks, latency |
| R6 | 동일 6D 손목 목표, 서로 다른 사람 팔꿈치 ψ posterior | ψ 의도 구별 및 q_ref 보존. **같은 관절각 강제 금지** |

동일 분포에 대한 결정성(R1)과 **노이즈가 섞인 다른 분포에 대한 강건성(R3)**을 동일한 수치로 혼동하지 말라.

### 5. 반드시 집계·보고할 수치

- 조건/시나리오/가림 시간별, 방법별: wrist position P50/P95 [mm], wrist orientation P50/P95 [deg], feasible ψ error P90 [deg], infeasible regret, jerk RMS, recovery jump [deg], branch flips, hold rate, solver runtime P50/P95/P99 [ms], 안전 위반 및 solver fail count.
- R1–R6의 canonical `q_ref` 지표와 실행 `q_cmd` 지표를 **분리**. 평균만이 아니라 worst-case, seed/trajectory 분포, 실패 사례 5개 이상을 보고.
- 결과 통계의 독립 단위는 trajectory/seed이며 100Hz 프레임을 독립 표본처럼 세지 말 것. 가능한 범위에서 trajectory별 paired 비교 및 95% bootstrap CI.
- G1 기준은 기존 roadmap에 따른다: strongest fair baseline 대비 wrist median 비열화 예시 +5mm/+5° 이내, ψ P90/jerk/recovery에서 유의미한 개선(초기 목표 15–20%). **조건별/전체 성과를 분리하고 미달이면 PARTIAL/FAIL**로 보고.
- 기존 `run_baselines.py` harness가 실제로 collision을 검사하는지 코드로 확인. collision 구현이 없다면 '충돌 제약 검증 완료'라고 쓰지 말고 **joint-limit / speed-cap까지만 검증되었다고 표시**.
- 최적화 기반 B7/B8의 높은 latency(11/14 로그: P95 대략 100–900ms)를 줄일 수 있는지도 검증. **최적화 반복수/멀티스타트 축소, analytic ψ-arc candidate search, 캐싱**을 후보로 하되 결과 정확도와 solver P95/P99의 trade-off를 비교. 실제 Jetson 측정치와 다른 환경 측정치는 혼합하지 말 것.

### 6. 결과물 저장/검증/GitHub 반영

- 코드: `share/sim/` 하위에 Canonical Solver/tracker, 재현 가능한 R1–R6 시험, 공통 harness 확장(또는 분리 실행기)을 작성. 기존 B0–B8 동작 변경 시 변경 이유와 regression을 기록.
- 중간 저장/최종 결과: `outputs/`(gitignore)을 유지하고 GitHub에는 **작은 요약 CSV·Markdown·재현 스크립트·config·시드·git SHA**만 커밋. 원본 894개 npz와 큰 생성 output, 사람 영상, 비밀정보는 GitHub에 올리지 말 것.
- 리포트: `share/11.14_exp/EX04_joint_consistency_results.md`에 실행 환경, 실제 데이터 수, baseline vs new 표, R1–R6 PASS/PARTIAL/FAIL, G1 판정, 시간·정확도 trade-off, 실패 원인, 다음 단계 명시.
- `python -m pytest -q share/sim/tests` 등 기존/신규 테스트 및 단일 trajectory smoke test 통과 후 전체 공통 데이터셋 평가. 가능하면 run-from-clean-clone 테스트도 수행.
- GitHub push 전 `git diff` 검토, 테스트/민감정보 확인, 기존 미커밋 작업 보존. 관련 변경만 commit하고 main으로 병합할 때 충돌 및 최신 커밋 확인. **실제로 push 성공이 확인되지 않으면 완료라고 말하지 말 것.**
- 종료 보고를 한국어로: ① 실제 수행한 개수 ② 가장 중요한 수치 ③ B7/B8/P-old/P-canonical 비교 ④ R1–R6/G1 상태 ⑤ 남은 blocker ⑥ GitHub commit/PR 링크. **실험 중이라면 현재 완료분만 사실로 보고**.

**최우선 순서:** (A) 현재 로컬 P/894개 결과 및 브랜치 보존·감사 → (B) R1/R2 canonical correctness → (C) P-old 포함 S5 작은 재현 → (D) R3–R6 전 894개 비교 → (E) latency/실패 개선 → (F) 한국어 보고서 + GitHub 업로드.
