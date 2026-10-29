# NERO Learned Retargeting

Vision 기반 Human-to-Robot 원격조작을 위한 학습형 오차 보정 Retargeting Solver 연구 저장소.

- Robot arm: AGILEX NERO (7-DoF)
- Robot hand: BrainCo REVO2 Pro (11 DoF, 6 active) — 후속 단계
- Input: RGB-D 기반 3D human pose (Shoulder / Elbow / Wrist / Hand landmarks)

> 현재 단계: **Stage 1 (Literature & Novelty Audit)**. 실험 결과는 아직 없으며, 문서의 신규성 판단은 모두 가설이다.

## 연구 단계

| Stage | 문서 | 상태 |
|---|---|---|
| 1. Literature Review & Novelty Audit | [docs/01_literature_novelty_audit.md](docs/01_literature_novelty_audit.md) | 초안 |
| 2. Coordinate & Kinematics Validation | `docs/02_kinematics_validation.md` | 예정 |
| 3. Baseline Failure Analysis | `docs/03_baseline_failure_analysis.md` | 예정 |
| 4. Proposed Solver Development | `docs/04_proposed_solver.md` | 예정 |
| 5. Benchmark & Ablation | `docs/05_benchmark_results.md` | 예정 |
| 6. Real-Robot Validation | `docs/06_real_robot_evaluation.md` | 예정 |
| 7. Publication Assessment | `docs/07_publication_assessment.md` | 예정 |

## 현재 주 후보 알고리즘

**Feasible-MAP Retargeting** (임시 명칭): 단일 시점 vision의 관측별 불확실성을 손목 SE(3)와 swivel angle 공간으로 전파하고, 로봇 실행 가능 집합 위에서 제약 MAP 추정으로 관절각을 구한다. 자세한 내용은 Stage 1 문서 Part C–D 참고.

## 협업

- 규칙: [CONTRIBUTING.md](CONTRIBUTING.md)
- 결정 기록: [docs/decisions.md](docs/decisions.md)

## 원칙

- 공식 URDF / SDK / 기구학 파라미터 없이 임의 값을 사용하지 않는다.
- 실기 명령 전 시뮬레이션과 안전 제한을 검증한다.
- Baseline을 약하게 구현하지 않는다.
- 측정되지 않은 수치를 보고하지 않는다. 실패 결과도 기록한다.
- 실험마다 설정·시드·데이터 분할·코드 버전을 기록한다.
