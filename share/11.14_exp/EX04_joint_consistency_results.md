# EX04 관절각 재현성·개선 실험 결과

> 작성: 2026-11-14 (KST) · 실행 지시: [Claude Code 실행 프롬프트](2026-11-14_EX04_joint_angle_consistency_CLAUDE_CODE_PROMPT.md) · 설계: [04 Canonical Solver](../docs/04_proposed_solver.md)
> **이 문서가 보고하는 것은 시뮬레이션/오프라인 평가뿐이다. 실기 로봇은 전혀 구동하지 않았다.**
> 재현: 아래 "재현 명령어" 절. 수치는 `outputs/`(gitignore)에서 계산되었고, 이 보고서가 인용하는 작은 파생 수치는 [`canonical_summary.csv`](canonical_summary.csv), [`canonical_branch_ambiguity.json`](canonical_branch_ambiguity.json)에 커밋되어 있다.

## 0. 요약 (TL;DR)

- **G1 판정: PARTIAL/FAIL.** canonical 방법(B7-canonical, P-canonical)은 occlusion이 없을 때는 기존 B7/P보다 **더 정확하고 훨씬 빠르지만**, occlusion 시나리오(S4/S5/S6, 714개 trajectory)에서 손목 median 오차가 기존 대비 평균 **+29.6 mm** 나빠진다 — 로드맵 G1 허용치(+5 mm)를 크게 초과한다. recovery jump도 평균 +87.7°로 악화된다.
- **원인은 명확하다**: canonical 기준(q_ref)은 R-CANON을 만족하기 위해 q_prev를 전혀 쓰지 않는데, 이 때문에 belief가 occlusion 중 충분히 이동하면 IK branch가 통째로 바뀐다(샘플 300프레임 중 87%가 같은 psi에서 2개 이상의 feasible branch를 가짐 — tie-break가 거의 항상 실제로 작동한다는 뜻). 기존 B7/P는 q_prev에 가까운 branch를 선택해 이 문제를 원천적으로 피하지만, 그 대가로 R1/R2(재현성)를 만족하지 못한다.
- **ablation (c) (고정 vs 적응형 tracking rate)는 canonical 내부에서도 재현된다**: P-canonical은 B7-canonical 대비 가장 심한 가림(S5, 2초)에서 recovery jump를 141°→97°(−31%)로, jerk를 8545→7941(−7%)로 줄인다 — 기존(non-canonical) P vs B7과 거의 같은 비율의 개선.
- R1/R2/R6는 **통과** (아래 fixture 테스트), 단 R1은 "동일 belief → 동일 q_ref"에 대한 보장이고 q_cmd(추적)의 재현성은 애초에 요구되지 않는다.
- **범위 제한**: 04_proposed_solver.md의 전체 결합 최적화(psi·branch 전체 그리드에 걸친 home-distance regularizer)는 이 Jetson에서 프레임당 ~1.2초가 걸려 894개 트라젝토리 스윕이 불가능했다. 대신 "mu_psi에 가장 가까운 feasible psi를 먼저 찾고, 그 psi에서의 branch만 home 거리로 tie-break"하는 축소판을 구현했다 — 이는 더 좁은 주장이며 이하 모든 결과는 이 축소판 기준이다.

## 1. 재현성 감사 (작업 시작 시점)

| 항목 | 상태 |
|---|---|
| 로컬 브랜치 | `feat/ex00-common-contract`에서 분기한 신규 브랜치 `feat/ex04-canonical-solver` 생성 (기존 브랜치 오염 방지) |
| 작업 시작 시 미커밋 변경 | `share/sim/{baselines,run_baselines,synthetic_human}.py` 3개 — 11/13-14 세션에서 만든 BLAS 스레드 수정, resumable 생성/체크포인트, 제안 방법 P(비-canonical) 구현. **보존하고 첫 커밋으로 분리** (`bbf5741`) |
| origin/main 대비 | 로컬이 뒤처져 있었음(5개 커밋: `04_proposed_solver.md`, decision.md 갱신, roadmap R1-R6 추가, 이번 실행 프롬프트, 11/14 로그 — 전부 문서). `git merge origin/main`으로 병합 (`ce4062c`), 충돌 없음 |
| 기존 B0-B8+P 894개 결과 | `outputs/baselines_P/per_traj.csv`, `summary.md` — **보존, 재사용**. 새로 실행한 것은 B7-canonical/P-canonical 2개 방법뿐 (`outputs/baselines_canonical/`) |
| EX04 synthetic 데이터 | `outputs/synthetic/manifest.json`: 900 jobs 중 894 성공, 1 failed(S6/seed2/idx14, 별도의 inhomogeneous-array 버그, 미조사). **894개 공통 평가셋 변경 없음** |
| 디스크의 "평가에 안 쓰인" 5개 파일 | `S6_drop2000ms/{seed2_idx14,16,17,18,19}.npz` — manifest에도 failed_jobs에도 없음. 11-13 로그에 기록된 "구버전 프로세스가 같은 디렉토리에 동시에 쓰던" 사고의 잔존 파일로 추정(결정론적 seed로 재계산해도 idx14는 현재 코드에서 매번 같은 RuntimeError로 실패하므로, 이 파일들은 그 사고 당시 다른 프로세스가 써넣은 것). 삭제하지 않았고 894개 비교셋에는 포함되지 않는다. |
| BLAS 스레드 설정 | `OPENBLAS_NUM_THREADS=1` 등을 `os.environ.setdefault`로 설정 — 이미 1보다 큰 외부 설정이 있었는지 실행 시점에 `echo $OPENBLAS_NUM_THREADS`로 확인(비어 있음, 즉 setdefault가 실제로 적용됨) |
| collision 검사 | `grep collision share/sim/*.py` → **0건**. run_baselines.py의 안전 계층은 **joint limit + per-joint speed cap + stale-input hold만 검증한다. Collision 제약은 구현되어 있지 않다** (이 문서를 포함해 이전 로그의 "안전 계층 동일"이라는 표현은 이 세 가지로 한정해서 읽어야 한다). |

## 2. Canonical Solver 구현 (범위 제한 명시)

`share/sim/canonical.py`:

- `canonical_feasible_map(m, T, mu_psi, q_home)` — **q_prev/난수/호출 순서에 의존하지 않는 순수 함수**. mu_psi에 가장 가까운 feasible psi를 먼저 찾고(직접 확인 → 각도 거리순 그리드), 그 psi에서의 모든 IK branch를 `q_home`(고정, `k.READY`) 거리로 tie-break한다. branch 열거는 `kinematics.Nero.ik()`(기존의 analytic, warm-start 없는 최대 8-branch 해석해)를 그대로 사용 — 이 자체가 이미 결정적이라 새로 만들 필요가 없었다.
- `track_safe(q_track, q_ref, dt, tau)` — 상태(occlusion 신뢰도, q_prev)가 들어가는 **유일한 곳**. 1차 저역 필터, `kinematics.wrap()`으로 ±π 경계를 감아서 처리한다.
- **실제로 구현 중 발견·수정한 버그 2개**: (1) 초기 버전은 `q_track`을 매 스텝 wrap하지 않아 branch가 ±π 경계 근처에서 전환될 때 2π의 배수로 drift — 단일 트라젝토리에서 wrist 오차가 300mm까지 치솟는 것으로 발견. (2) canonical target이 infeasible(= `pose_relaxed`)일 때 즉시 `q_home`으로 점프하도록 했던 초기 버전을 "그 프레임은 hold"로 수정 (04_proposed_solver.md Part 1B의 권고와 일치).

**범위 축소 (명시)**: 04_proposed_solver.md Part D.3/1A의 완전한 공식은 전체 psi 도메인에 걸쳐 wrist/psi loss + home-distance를 **공동 최적화**한다. 이 전체 그리드(180개 psi × branch) 스캔을 측정한 결과 **프레임당 최대 ~1.2초**(S3, infeasible-heavy 시나리오)가 걸려 894개 트라젝토리 스윕이 비현실적이었다(단순 외삽으로 수십 시간). 따라서 "mu_psi에 가장 가까운 feasible psi를 찾고 그 psi 내에서만 home-tie-break"로 좁혔다 — home prior는 전역 regularizer가 아니라 **좁은 의미의 분기 tie-break**다. 이는 (f) home-prior 가중치 sweep 실험이 원래 설계대로는 적용되지 않는 이유이기도 하다(연속적인 가중치가 없음).

성능상 그리드 해상도도 2°→10°로 낮췄다(infeasible-everywhere 프레임에서 1.17s→0.23s/frame, 측정값). 이 정밀도 손실은 이미 "완화된" 경로에만 영향을 준다.

## 3. R1/R2/R6 — fixture 기반 결과 (`tests/test_canonical.py`, 8개 전부 PASS)

| 시험 | 방법 | 결과 |
|---|---|---|
| R1 반복 호출 | 동일 belief에 `canonical_feasible_map` 100회 반복 | **PASS**, `max‖Δq_ref‖∞ = 0`(완전 동일값 — 반복 solver가 전혀 없는 순수 analytic+그리드 탐색이므로 1e-6 rad보다 훨씬 강한 보장) |
| R1 호출 순서 | 6개의 다른 belief를 정순/역순으로 평가 | **PASS**, 순서 무관 |
| R1 프로세스 재시작 | 같은 belief를 별도 `subprocess`에서 재계산 | **PASS**, 동일 |
| R2 경로 독립성 (q_start) | 서로 다른 q0에서 `reset()`한 두 Method 인스턴스에 동일 frame으로 `update()` | **PASS**, `q_ref`/`branch_id` 동일. q_track(추적 상태)는 의도적으로 다름 — q_ref만 canonical이면 되므로 |
| R2 경로 독립성 (호출 이력) | 5개의 무관한 이전 frame을 거친 뒤 동일 최종 belief에 도달 | **PASS**, `q_ref` 동일 |
| R6 동일 wrist·다른 ψ | 같은 6D wrist, feasible arc 안의 서로 다른 두 psi | **PASS**, `q_ref`가 **다름**(강제로 동일시키지 않음), 각 psi에 가장 가까운 psi_ref를 각각 선택 |
| pose-relaxed fallback | 도달 불가능한 wrist 위치 | **PASS**, `feasible=False`, `q_ref=q_home`으로 명확히 표시 |

R1/R2는 설계상 **q_ref에 대한 명제**이며, 이 보고서가 다루는 "canonical이 회복 구간에서 더 나쁘다"는 **q_cmd(추적) 수준의 트레이드오프**다. 둘은 서로 모순되지 않는다 — q_ref는 완벽하게 재현 가능하지만, 그 재현 가능한 q_ref로 **가는 길**이 매 가림마다 달라질 수 있다는 것이 이번 실험의 핵심 발견이다.

R3(노이즈 강건성)은 **이번에 수행하지 않았다** — 별도의 노이즈 레벨 스윕이 필요하며 시간 예산상 R4(가림, 전체 894개 데이터로 수행됨)를 우선했다. 아래에서 PENDING으로 명시한다.

## 4. R4/R5 — 전체 894개 트라젝토리 비교

B0-B8, P는 기존 `outputs/baselines_P/per_traj.csv`를 재사용(재실행 안 함). B7-canonical, P-canonical만 새로 실행(`outputs/baselines_canonical/`), 동일 894개 공통 파일·동일 `--psi-map mirror --qd-max pi --hold-s 0.5`.

### 4.1 Occlusion 유무로 나눈 평균 변화 (P-old → P-canonical, 894개 중 finite pair만)

| 구간 | n | 손목 mm Δ | jerk Δ | ψ P90 Δ | recovery jump Δ |
|---|---|---|---|---|---|
| **Occlusion 없음** (S1/S2/S3) | 180 | **−3.1** (개선) | +1680 (악화) | **−31.9°** (큰 개선) | (해당없음) |
| **Occlusion 있음** (S4/S5/S6) | 714 | **+29.6** (악화, G1 허용치 5mm 초과) | +2537 (악화) | +5.2° (소폭 악화, 유의성 약함) | **+87.7°** (큰 악화) |

전체 894개 평균(부트스트랩 95% CI, `canonical_summary.csv`): 손목 mm +23.0 [+19.9, +26.7], ψ P90 −1.9 [−4.7, +0.8] (유의하지 않음), jerk +2364 [+2255, +2481], recovery jump +87.7 [+80.4, +94.9], **런타임 P95 −107.6 ms [−112.3, −103.0]** (즉 B7/P의 ~140-150ms에서 canonical은 ~40-60ms로, S3 등 일부 시나리오를 빼면 약 3배 빨라짐).

### 4.2 Occlusion 중 가장 심한 조건(S5, 2초 가림)에서의 ablation (c): 고정 vs 적응형 tracking

| 방법 | 손목 mm | ψ° | jerk | recovery jump° |
|---|---|---|---|---|
| B7-canonical (고정 tau) | 88 | 22.8 | 8545 | 141 |
| P-canonical (적응형 tau) | 86 | 21.8 | 7941 | **97 (−31%)** |

비-canonical 쪽(P-old vs B7-old, 11/14 앞선 실험)에서 본 "적응형이 고정보다 recovery jump를 32-48% 줄인다"는 결과가 **canonical 틀 안에서도 거의 같은 비율로 재현**된다. 즉 "적응형 tracking rate가 낫다"는 결론은 canonical 여부와 독립적으로 성립하는, 더 견고한 발견이다.

### 4.3 가장 나빴던/좋았던 사례 5개 이상 (recovery_jump_deg, P-old → P-canonical delta)

| 파일 | P-old | P-canonical | Δ |
|---|---|---|---|
| S6_drop200ms/s2_004 | 1.6° | 335.7° | **+334.1°** |
| S6_drop200ms/s1_012 | 2.6° | 334.8° | +332.3° |
| S5_drop200ms/s0_013 | 8.9° | 337.2° | +328.3° |
| S5_drop2000ms/s2_005 | 22.2° | 334.4° | +312.2° |
| S4_drop200ms/s1_001 | 5.7° | 315.1° | +309.5° |
| S6_drop2000ms/s0_011 (반대 방향) | 256.9° | 88.2° | **−168.7°** |
| S6_drop2000ms/s1_002 (반대 방향) | 146.3° | 2.1° | −144.2° |

**714개 occlusion 트라젝토리 중 149개(20.9%)만 canonical이 non-canonical보다 같거나 나음** — 나머지 79%는 canonical이 더 나쁘다. 위 "+330° 급" 사례들은 branch가 통째로 바뀌는 경우로, home-tie-break가 고른 branch가 이전 상태와 전혀 다른 분기일 때 발생한다(300프레임 샘플 중 87%가 해당 psi에서 2개 이상 feasible branch를 가짐 → tie-break가 거의 항상 "실제로" 결정을 내리고 있고, occlusion으로 belief가 충분히 움직이면 그 결정이 바뀔 수 있다는 뜻). 반대로 "non-canonical이 훨씬 나빴던" 사례들은 L-BFGS-B(q_prev에서 시작하는 지역 최적화)가 나쁜 지역해에 갇힌 경우로 보인다 — canonical의 analytic 탐색은 이런 실패 모드가 없다.

### 4.4 Feasibility / S3 (R5)

S3(의도적으로 많은 목표가 infeasible한 시나리오, `psi_feasible_pct` 22%)에서 canonical은 ψ P90 43°로 non-canonical B7-old의 131°보다 크게 낫고(§4.1의 비-occlusion 개선과 일치), **계산 시간은 196.7ms로 non-canonical(900ms대, 11/14 앞선 로그)보다 훨씬 빠르다** — 다만 이 196ms 자체도 대부분 `run_baselines.py`의 기존 `targets()`(평가용 ground-truth 계산, canonical과 무관)가 S3에서 트라젝토리당 최대 ~98초 걸리는 **기존(변경하지 않은) 코드의 문제**이며, 이 보고서의 범위 밖이다. canonical 방법 자체의 S3 실행 시간은 20.9s(794 non-S3 평균 수십 ms보다 느리지만 L-BFGS-B 기반 B7(12.6s)보다 약간 느릴 뿐 — targets()와 혼동하지 않도록 §4.1의 runtime_p95_ms는 run() 루프만의 수치다).

## 5. Ablation 커버리지 (솔직한 범위 보고)

| Ablation | 수행 여부 | 결과/근거 |
|---|---|---|
| (a) canonical tie-break 제거 | **부분**: 전체 재실행은 안 함. 대신 300-frame 샘플로 "tie-break가 실제로 결정을 내리는 비율"을 측정 | **87.0%** 프레임이 선택된 psi에서 2개 이상 feasible branch를 가짐 (11/300은 pose_relaxed). tie-break를 없애면(=임의 선택) R1이 거의 항상 깨질 것 — 수치로 뒷받침됨, 직접 "제거한 버전"을 894개로 돌리지는 않음 |
| (b) q_prev를 reference solver에 직접 사용 | **기존 결과로 커버**: 이것이 정확히 P-old/B7-old(비-canonical)의 구조. §4의 모든 비교가 이 ablation이기도 함 | 위 §4 전체 |
| (c) 고정 vs 적응형 continuity | **완료**, 894개 전체 | §4.2, −31%(recovery jump, 가장 심한 가림) |
| (d) branch-aware tracking 없음 | **구현 안 됨** — 현재 `track_safe`는 branch 전환을 특별히 다루지 않는다(그래서 §4.3의 +330° 급 사례가 발생). 이것이 사실상 기본값(ablation (d) 조건 자체)이다 | 가장 중요한 다음 단계로 제안 (§7) |
| (e) ψ term off | **수행 안 함** (시간 예산). 이 경우 항상 "home에 가장 가까운 branch"만 고르게 되어 04_proposed_solver.md가 제시한 baseline (a) "home-posture + deterministic IK"와 동등해짐 — 별도 실행 없이 등가성만 확인 | PENDING |
| (f) home prior 가중치 sweep | **적용 불가** (§2 범위 축소: home은 연속 가중치가 아니라 tie-break라 "가중치"가 없음) | 설계를 확장해야 가능 |

## 6. G1 판정

로드맵 기준: *"strongest fair baseline 대비 wrist 비열화 median +5mm/+5° 이내이며, ψ P90/jerk/recovery 중 1차 지표 15-20% 개선."*

- **P-canonical vs strongest non-canonical baseline (P-old/B7-old) 전체 894개**: **FAIL**. Occlusion 시나리오에서 손목 median이 평균 +29.6mm 나빠짐(허용 +5mm의 약 6배), recovery jump가 +87.7°(개선이 아니라 악화) 나빠짐. jerk도 악화. **G1을 통과하지 못한다.**
- **비-occlusion 구간만(S1-S3)**: 손목 비열화 없음(오히려 −3.1mm 개선), ψ P90 −31.9°(큰 개선), 런타임 대폭 개선 — 이 부분 지표만 보면 **PASS**였을 것.
- **Canonical 내부 비교(P-canonical vs B7-canonical, ablation c)**: 가장 심한 가림에서 recovery jump −31%, jerk −7%, 손목 비열화 없음 — 이 좁은 질문("고정 대비 적응형이 낫나")에는 **PASS**.
- **종합**: "제안 방법이 가장 강한 기존 baseline을 이긴다"는 원래 G1 질문에는 PARTIAL/FAIL. "canonical 제약 안에서 적응형 tracking이 고정보다 낫다"는 더 좁은 질문에는 PASS. 둘을 섞어서 보고하지 않는다(04_proposed_solver.md §2 "다른 method가 canonical 규칙만 없어서 손해"를 그대로 관찰한 결과이기도 하다 — 단, 이번엔 canonical 쪽이 불리했다).

## 7. 남은 과제 / 다음 단계 (제안, 순위별)

1. **branch-aware tracking (ablation d) 구현**: branch_id가 바뀌는 프레임을 감지해 그 전환에서만 속도를 추가로 제한하거나 hold하는 로직을 `track_safe`에 추가. §4.3의 +330° 사례들을 직접 겨냥한 수정이며, 이번 실험에서 드러난 가장 명확한 다음 스텝.
2. **R3 (노이즈 강건성)** 수행: 지금은 생략됨.
3. **(e) psi term off, (f) home prior sweep**: (e)는 코드 한 줄로 가능(별도 실행 필요), (f)는 설계를 "전역 tie-break"에서 "연속 regularizer"로 확장해야 함(비용 재측정 필요).
4. S6_drop2000ms/seed2/idx14 생성 버그(별도, 미조사) 및 디스크상 5개 미사용 파일 정리(삭제 여부는 팀 결정).
5. `run_baselines.py`의 기존 `targets()` S3 비용(트라젝토리당 최대 ~98s, canonical과 무관)도 전체 894개 반복 실험의 실질적 병목이므로 최적화 후보.

## 재현 명령어

```bash
cd share/sim
# 새 방법만 재실행 (B0-B8/P는 outputs/baselines_P/에 이미 있음, 재사용)
python3 run_baselines.py outputs/synthetic --methods B7c Pc --psi-map mirror --out outputs/baselines_canonical
# 페어 비교 + ablation(a) 진단 (이 문서의 canonical_summary.csv를 재생성)
python3 analyze_canonical.py --ablation-sample 300
# R1/R2/R6 unit test
python3 -m pytest -q tests/test_canonical.py
# 전체 회귀 테스트 (42개, 이번에 통과 확인)
python3 -m pytest -q tests/
```

환경: Jetson AGX Orin, numpy 1.26.4 (OpenBLAS), `OPENBLAS_NUM_THREADS=OMP_NUM_THREADS=MKL_NUM_THREADS=1`, Python 멀티프로세싱 8 workers. git SHA: 이 디렉토리의 `git rev-parse HEAD` (브랜치 `feat/ex04-canonical-solver`).
