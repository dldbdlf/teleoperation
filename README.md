# NERO Learned Retargeting

Vision 기반 Human-to-Robot 원격조작을 위한 학습형 오차 보정 Retargeting Solver 연구 저장소.

- Robot arm: AGILEX NERO (7-DoF)
- Robot hand: BrainCo REVO2 Pro (11 DoF, 6 active) — 후속 단계
- Input: RGB-D 기반 3D human pose (Shoulder / Elbow / Wrist / Hand landmarks)

> 현재 진행: **NERO URDF 기반 FK/해석적 IK·ψ feasible arc 시뮬레이션 검증 및 Jetson/RealSense 환경 구축 기록**. Feasible-MAP 성능 비교, occlusion belief 학습, NERO 실기 task 성능은 **아직 미검증**이다.

## 연구 단계

| Stage | 문서 | 상태 |
|---|---|---|
| 1. Literature Review & Novelty Audit | [share/docs/01_literature_novelty_audit.md](share/docs/01_literature_novelty_audit.md) | 초안 |
| 2. Coordinate & Kinematics Validation | [share/docs/02_kinematics_validation.md](share/docs/02_kinematics_validation.md) | 시뮬레이션 부분 검증, 실기 미검증 |
| 3. Baseline Failure Analysis | `share/docs/03_baseline_failure_analysis.md` | 예정 |
| 4. Proposed Solver Development | [share/docs/04_proposed_solver.md](share/docs/04_proposed_solver.md) | **Canonical mapping 설계 초안 (미구현)** |
| 5. Benchmark & Ablation | `share/docs/05_benchmark_results.md` | 예정 |
| 6. Real-Robot Validation | `share/docs/06_real_robot_evaluation.md` | 예정 |
| 7. Publication Assessment | `share/docs/07_publication_assessment.md` | 예정 |

**실행 문서:** [전체 연구 로드맵 V2](share/roadmap.md) · [상세 실험 로드맵 (EX00–EX10)](share/docs/experiment_roadmap.md) · [Jetson 센서 구축 기록](share/11.03_exp/README.md)

## 현재 주 후보 알고리즘

**Feasible-MAP Retargeting** (임시 명칭): 단일 시점 vision의 관측별 불확실성을 손목 SE(3)와 swivel angle 공간으로 전파하고, 로봇 실행 가능 집합 위에서 제약 MAP 추정으로 관절각을 구한다. 자세한 내용은 Stage 1 문서 Part C–D 참고.

**2026-11-13 추가 요구사항 — Repeatable Canonical Mapping:** 같은 *전체 human belief*와 고정 URDF·좌표계·캘리브레이션·정책 설정이면 **동일한 기준 관절각 `q_ref`와 IK branch를 결정적으로 출력**해야 한다. 동일 손목 위치만으로 같은 인간 의도를 가정하지 않는다. 실제 명령 `q_cmd`는 현재 관절 상태와 safety 조건에 따라 달라질 수 있으므로 별도 계층에서 생성한다. R1–R6 반복성·경로 독립성·노이즈·가림 검증 계획은 [제안 solver 설계 문서](share/docs/04_proposed_solver.md)를 참고. **구현/성능 검증 전**.

## 협업

- 규칙: [CONTRIBUTING.md](CONTRIBUTING.md)
- 결정 기록: [share/decision.md](share/decision.md) (중요한 결정은 이 파일에만 기록)
- 로드맵: [share/roadmap.md](share/roadmap.md) · [실험 계획](share/docs/experiment_roadmap.md)

## 폴더 구조

| 폴더 | 용도 |
|---|---|
| [yuil/](yuil/) | yuil 개인 작업 |
| [headam/](headam/) | headam 개인 작업 |
| [share/](share/) | 공동 결과물 (단계별 문서, 결정 기록, 공동 코드) |

## 원칙

- 공식 URDF / SDK / 기구학 파라미터 없이 임의 값을 사용하지 않는다.
- 실기 명령 전 시뮬레이션과 안전 제한을 검증한다.
- Baseline을 약하게 구현하지 않는다.
- 측정되지 않은 수치를 보고하지 않는다. 실패 결과도 기록한다.
- 실험마다 설정·시드·데이터 분할·코드 버전을 기록한다.
