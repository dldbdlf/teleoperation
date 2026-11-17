# 2026-11-17 — NERO E1 Solver 정확도 유지 최적화

## 작업 상태

- GitHub main에 최적화 구현, 회귀 테스트, Checkpoint/Resume 실행 옵션 반영 완료
- 독립적인 로컬 CPU 축소 검증에서 이전 E1 대비 속도 개선 확인
- **Jetson AGX Orin 실측 및 기존 894개 trajectory 전체 재평가는 아직 수행하지 않았음**
- 기존 E1/P/Pc/E2/E3 결과와 체크포인트를 변경하지 않음

## 적용한 두 최적화

**A. 프레임별 IK 공통 연산 1회 수행** (fast_ik_bank.py / FastIKPose)

- 목표 T가 고정된 동안 손목 위치, J4 회전 분기, 공통 팔 기하를 반복해서 계산하지 않음
- 4×4 강체변환 역은 회전 전치와 상대 위치 계산으로 대체
- 관절 한계와 원래 FK pose 오차 검증은 그대로 유지

**B. E1의 정확한 ψ 가지치기** (fast_ik_bank.py / exact_canonical_e1)

E0 기준해와 E1 추종해의 비용은 다음과 같음.

L(ψ) = 0.5 * [wrap(ψ-μ)/σ]^2
C_ref(q,ψ) = L(ψ) + 0.055 * ||q-READY||²
C_track(q,ψ) = C_ref(q,ψ) + 0.85 * ||q-q_measured||²

모든 추가 항은 비음수. 따라서 미평가 ψ의 L(ψ)가 현재 두 최선 비용
모두보다 크면 그 ψ에서 어떠한 IK 해가 나오더라도 두 최적해를 바꿀 수 없음.
이를 수학적 하한 인증으로 사용해 평가하지 않는 ψ를 안전하게 제거.
동점 가능성에는 여유값을 두며 원래 후보의 순서로 tie-break.

주의: 모든 후보 확률에 의존하는 E2/E3에는 이 가지치기를 적용하지 않음.
E1에 대해서만 명시적인 fast_e1 옵션을 활성화한 경우 동작.

## 로컬 CPU 측정 (Jetson 속도로 오해하지 말 것)

NERO SRS 분석식 동일한 로컬 기구학, 각 uncertainty 25개 pose, BLAS threads=1.

| σψ | 기존 평균 | 최적화 평균 | 속도 향상 |
| ---: | ---: | ---: | ---: |
| 5° | 59.4ms | 4.4ms | 13.47× |
| 10° | 60.2ms | 7.3ms | 8.22× |
| 20° | 56.7ms | 9.7ms | 5.87× |
| 35° | 58.0ms | 13.8ms | 4.21× |
| 65° | 62.7ms | 22.3ms | 2.81× |
| 120° | 59.5ms | 33.8ms | 1.76× |

정확성 추가 검증: 16 random poses × 3 ψ grids(10/15/20deg) ×
7 observation noise levels = 336회. q_ref/q_des 최대 차이
1.55e-14rad, 관절 목표 불일치 0.
평균 ψ 그룹 평가 비율 39.7%.
FK가 불가능한 자세에서는 ψ 그룹을 전부 확인한 뒤 원래 fallback.

## Jetson에서 실행 (원본 결과 보존)

GitHub main의 새 코드를 Jetson 로컬에 안전하게 병합한다.
로컬 수정사항이 남아있다면 백업/커밋 후 병합.
기존 checkpoints 및 legacy run_all 폴더를 덮어쓰지 말 것.

~~~bash
cd /path/to/teleoperation
git status -sb
git fetch origin main
git merge origin/main

cd share/sim
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1

# 테스트: 한 건이라도 실패하면 894 실행 금지
python3 -m pytest -q experiments/prob_joint/tests/test_fast_e1.py
python3 -m pytest -q tests/test_canonical.py experiments/prob_joint/tests/test_canonical_e0e3.py

# 실제 894개 cohort에서 먼저 60프레임 microbenchmark
python3 experiments/prob_joint/benchmark_fast_e1.py \
  --cohort outputs/ex04_prob_894/cohort_data \
  --samples 60 --grid 15 \
  --out outputs/ex04_prob_894/fast_e1_pilot.json

# 894개 E1 실험: 별도 DB/CSV 사용, 자동 Checkpoint/Resume/ETA
python3 -u experiments/prob_joint/run_ckpt.py fast15 --fast-e1 \
  --cohort outputs/ex04_prob_894/cohort_data \
  --out outputs/ex04_prob_894/run_e1_fast15 --workers 8

# 진행률 확인
python3 experiments/prob_joint/run_ckpt.py --status \
  --out outputs/ex04_prob_894/run_e1_fast15

# 중단 시 위 fast15 명령을 똑같이 재실행하면 Resume
# 성공 후 품질 지표 전체 비교
python3 experiments/prob_joint/compare_fast_e1.py \
  --legacy outputs/ex04_prob_894/run_all/per_traj.csv \
  --fast outputs/ex04_prob_894/run_e1_fast15/per_traj.csv \
  --grid 15 --expected 894 \
  --out outputs/ex04_prob_894/e1_fast15_comparison.json
~~~

검증 결과 모든 non-timing 품질 지표가 같아야 성공.
최적화 여부는 config_hash에 들어가 기존 체크포인트와 자동 분리됨.

## 평가상 주의사항

- 최적화된 E1의 canonical q_ref는 입력 belief의 함수이며 이전 q에 의존하지 않음
- 후보 전체를 평가하지 않았으므로 기존 E1의 전체 bank entropy/candidate-count
  통계는 fast 모드에서 NOT MEASURED 처리(확률 보정 지표로 재사용 금지)
- 실제 Jetson에서 후보 생성 포함 15Hz deadline/P95/P99를 새로 측정해야 함
- 가림 시 관절 불확실성이 크면 가지치기가 줄고 지연이 다시 커질 수 있음
- Sim에서는 여전히 실제 solver 지연이 control에 반영되지 않을 수 있으므로
  실제 E2E에는 지연 인식 replay, 안전 검증, D415 데이터 평가가 필요
- 실제 로봇 제어는 금지
