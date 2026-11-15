# EX04 894개 전체 재검증 — Jetson 실행 프로토콜 (2026-11-15)

## 상태
**준비 완료 / 전체 실험 미실행.** 이 GitHub 대화 환경은 Jetson 원격 셸이나 로컬 894개 \`npz\`를 직접 읽을 수 없습니다. 따라서 894개 E0–E3 결과가 나왔다고 말하면 안 됩니다.

현재 GitHub \`main\`에는 P/Pc가 없습니다. 2026-11-14 로컬 보고서의 \`feat/ex04-canonical-solver\` 브랜치에만 있다고 기록돼 있으므로 **먼저 Jetson 미푸시 변경을 보존하고 새 스크립트를 병합**해야 합니다. \`git reset --hard\` 금지.

## 장비
- **정확도/관절 오차/branch switching:** 원본 NPZ와 같은 코드/설정이 있으면 워크스테이션에서도 가능. RTX A6000 GPU는 해석적 IK 코드에 필수 아님; CPU 스레드·메모리·버전 고정이 더 중요.
- **Jetson 지연·15Hz deadline/100Hz 안전 제어:** 최종 대상인 Jetson AGX Orin에서 측정해야 함. \`runtime_p95_ms\`만으로 센서→액추에이터 E2E 지연을 대체할 수 없음.
- 실기 NERO 로봇 명령 절대 금지. 여기는 EX04 오프라인 시뮬레이션만.

## 실행 전 필수 조건
1. Jetson에서 P/Pc 구현·원본 \`outputs/synthetic\`·원본 \`outputs/baselines_full2/per_traj.csv\` 존재.
2. 실험 저장소의 \`share/sim/run_baselines.py\`가 P, B7c, Pc를 실제로 등록해야 함(별명으로 대체 금지).
3. GitHub \`main\`의 \`share/sim/experiments/prob_joint/\` 아래 신규 4개 스크립트를 해당 로컬 브랜치에 반영.
4. \`git status -sb\`, \`git log -1\`, \`python -V\`, numpy/scipy 버전, Jetson 모드 및 CPU governor 기록.
5. **원본 894개 공통집합은 기존 CSV로 추출.** Synthetic manifest는 899개일 수 있으므로 무조건 manifest 전체 재실행 금지.

## 1) 고정 894개 평가셋 생성

Jetson 로컬 저장소 루트에서 (위의 병합이 완료된 뒤):

\`\`\`bash
cd ~/teleoperation_repo
python3 share/sim/experiments/prob_joint/prepare_894.py \
  --data share/sim/outputs/synthetic \
  --reference-csv share/sim/outputs/baselines_full2/per_traj.csv \
  --out share/sim/outputs/ex04_prob_894 \
  --expected 894
\`\`\`

이 명령은 평가를 돌리지 않고 정확히 894개를 점검한 뒤 symlink 기반 cohort manifest를 생성합니다. missing/중복/개수 불일치/원래 Pc 코드 없음이면 **중단**합니다.

## 2) 회귀 테스트 → 894개 비교

\`\`\`bash
cd share/sim
python3 -m pytest -q tests
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
export EX04_PROB_PSI_GRID_DEG=15
python3 -u experiments/prob_joint/run_894_methods.py \
  outputs/ex04_prob_894/cohort_data \
  --psi-map mirror --qd-max 3.141592653589793 --hold-s 0.5 \
  --methods B7 B8 P B7c Pc E0 E1 E2 E3 \
  --out outputs/ex04_prob_894/full_15deg \
  2>&1 | tee outputs/ex04_prob_894/run_15deg.log
\`\`\`

원래 Jetson 로컬 \`run_baselines.py\`의 20개 trajectory checkpoint/원자적 CSV 저장/resume 기능을 그대로 사용해야 합니다. 원격 main의 구버전에는 이 기능이 없으니 **로컬 버전을 보존**해야 합니다. 종료 코드 검증 시 \`set -o pipefail\` 권장.

## 3) 결과 집계 / 재현성 감사

\`\`\`bash
cd ~/teleoperation_repo
python3 share/sim/experiments/prob_joint/analyze_894.py \
  share/sim/outputs/ex04_prob_894/full_15deg/per_traj.csv \
  --old-csv share/sim/outputs/baselines_full2/per_traj.csv \
  --cohort share/sim/outputs/ex04_prob_894/cohort_894.json \
  --out share/sim/outputs/ex04_prob_894/analysis_15deg.md
\`\`\`

확인할 것:
- **894 trajectory × 9 methods = 8,046 rows** (선택한 9개 method 기준), 모두 같은 (group,file) key.
- B7/B8/P/Pc 재실행 결과는 기존과 수치 비교; 런타임은 CPU 부하로 달라도 정상.
- S4/S5/S6 (0.2/0.5/1/2초), S1/S2/S3 조건별 손목 오차/ψ/jump/jerk.
- E1 vs E0, E2 vs E1, E3 vs E1, E1/E2/E3 vs 기존 P/Pc의 trajectory paired 95% CI.
- 동일 입력 100회 R1 및 R2 다른 경로 검증은 별도 canonical fixture 테스트 필요. 현 runner의 \`q_ref\` 별도 dump 없음. **이 파일만으로 R1–R6 PASS 금지.**
- branch ID/모드 전환율, collision, 실제 가속도 안전 등이 CSV에 없으면 \`NOT MEASURED\`로 보고.

## 4) Ψ 후보 grid ablation

먼저 \`15°\` 결과를 확인한 뒤 공통 894개와 E0–E3만 선택해서 \`10°\`, \`20°\`도 별도로 실행. 결과 출력을 다른 경로에 저장. 그리드 변경 시 \`run_894_methods.py\`를 동일 안전 계층으로 사용.

예:
\`\`\`bash
cd ~/teleoperation_repo/share/sim
export EX04_PROB_PSI_GRID_DEG=10
python3 -u experiments/prob_joint/run_894_methods.py \
  outputs/ex04_prob_894/cohort_data --psi-map mirror \
  --methods E0 E1 E2 E3 --out outputs/ex04_prob_894/full_10deg
export EX04_PROB_PSI_GRID_DEG=20
python3 -u experiments/prob_joint/run_894_methods.py \
  outputs/ex04_prob_894/cohort_data --psi-map mirror \
  --methods E0 E1 E2 E3 --out outputs/ex04_prob_894/full_20deg
\`\`\`

## 5) 보고서 및 배포
- \`share/11.15_exp/prob_joint_894_result.txt\`: 실제 894개 결과/개선/악화/CI/실패 원인/기준 G1 판정.
- Jetson 모델, 커널/JetPack, Python/numpy/scipy, CPU 스레드, CPU 부하, commit SHA, dataset manifest sha를 반드시 기록.
- GitHub에는 요약 CSV/Markdown/스크립트만 commit. 894개 \`npz\` 및 거대한 원시 로그는 별도 보관.
- **측정이 끝나기 전 PASS 또는 full revalidation complete라고 주장 금지.**

## 연구적으로 필요한 판단
- E1이 좋아지고 E2 추가 이득이 없으면 확률 모델을 기여로 내세우지 말 것.
- E2가 의미 있게 개선된다면 사람 ψ 정확도, wrist 정확도 및 deadline을 모두 유지하는지 확인.
- E3 시간적 확률 추적이 branch flip을 감소시켜도 R4 복귀 지연/ψ 오차를 악화시키면 실패/트레이드오프로 보고.
- 결론은 \`result.txt\` 기존 P/Pc 성능과 새 결과를 동일 기준으로 비교해서 내릴 것.
