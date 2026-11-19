# B9 IKFlow on NERO — 학습·평가 결과

> 작성: 2026-11-19 · 실행: headam 로컬 PC (RTX 3090 Ti, WSL2) · **시뮬레이션/오프라인 평가만. 실기 NERO 구동 없음.**
> 논문: Ames, Morgan, Konidaris, *IKFlow: Generating Diverse Inverse Kinematics Solutions*, RA-L 2022 ([arXiv 2111.08933](https://arxiv.org/abs/2111.08933)) · 코드: [jstmn/ikflow](https://github.com/jstmn/ikflow) `2f4636e`, [jrl](https://github.com/jstmn/jrl) `2ba7c39`

## 요약

- IKFlow(조건부 normalizing flow)를 논문 Appendix A 설정으로 NERO 7-DoF에 **48시간, 195만 step** 학습했다.
- 최종 모델 정확도: latent scale 0.25에서 위치 평균 **6.8 mm**(중앙값 4.1), 회전 평균 **5.3°**(중앙값 1.9°). 위치는 논문 보고 범위(0.4–7.7 mm) 안이고, 회전은 논문(0.15–2.8°)보다 크다.
- EX04 하네스(B0–B8과 같은 조건)에서는 **B7/B8을 이기지 못했다.**
  - B9(IKFlow 샘플 + 해석적 IK 투영)는 B7/B8과 비슷하거나 약간 나쁘다.
  - B9r(raw)은 손목 오차가 B7/B8 수준까지 내려왔지만 ψ P90은 2–3배 크다.
  - jerk는 B7/B8보다 20–60% 크다.
- **결론:** 입력이 목표 플랜지 자세뿐이고 사람 팔꿈치·가림 정보가 없어서, 우리 핵심 문제(가림 중 ψ 결정)는 풀지 못한다. 또 랜덤 샘플링이라 R-CANON(같은 belief → 같은 `q_ref`)을 만족하지 않는다. **비교군 참고용으로만 유지**하는 것을 제안한다.

## 모델 입출력

| | 크기 | 내용 |
|---|---|---|
| 조건 | 7+1 | link7(플랜지) 자세 `x,y,z,qw,qx,qy,qz` (base_link 좌표계) + SoftFlow 노이즈 크기(추론 시 0) |
| 잠재 z | 7 | N(0, scale²I). 해의 다양성(ψ 위치)을 결정한다 |
| 출력 | 7 | J1–J7 [rad], 관절 한계로 clamp |

하네스에서는 belief 손목(`b_W`, `b_R`)을 `tool_pose()`로 플랜지 자세로 바꿔 넣는다. 사람 ψ는 **모델 입력이 아니며**, 샘플을 고른 뒤 선택 비용에만 쓴다.

## 학습 설정 (논문 Appendix A 기준)

| 항목 | 논문 | 이번 실행 |
|---|---|---|
| 데이터 | 2.5M (Panda 공개 모델은 25M, 비자기충돌 필터) | 2.5M, 균일 관절 샘플 → FK. **자기충돌 필터 없음** (URDF에 충돌 형상 없음) |
| 네트워크 | Glow coupling, 3×1024 LeakyReLU, softflow 1e-3 | 같음, coupling 12층, latent 7 (공개 Panda 모델 `panda__full__lp191_5.25m`과 같은 값) |
| 최적화 | Ranger, batch 128, lr 5e-4, ×0.979 / 39000 batch | 같음 |
| 학습량 | "수렴까지" (공개 Panda 모델 5.25M step) | **1.95M step / 48 h** 상한 |

NERO 관절 범위 합은 27.7 rad이다. 논문 Fig. 8에 따르면 1 cm 아래로 내려가는 데 1–5M batch가 필요한 구간이다.

## 결과 1 — 정적 정확도 (무작위 도달 가능 자세, `results/static_*.json`)

| step | 포즈 수 | 위치 평균 mm (s=0.25) | 회전 평균 ° (s=0.25) | 1 cm·2° 이내 해 % (s=1) | ψ 가능 구간 커버 raw % | 투영 후 커버 % | 256개 샘플 시간 P95 ms |
|---|---|---|---|---|---|---|---|
| 10k | 50 | 67.4 | – | – | – | – | – |
| 100k | 200 | 23.0 | 12.3 | 7.6 | 25.7 | 95.2 | 20.9 |
| 120k | 200 | 19.4 | 10.7 | 13.3 | 37.8 | 95.2 | 21.8 |
| **1.95M** | 200 | **6.8** | **5.3** | **55.0** | **71.2** | 94.3 | 40.5 |

- 시간은 학습이 GPU를 같이 쓰던 상태와 아닌 상태가 섞여 있어서, step 간 직접 비교는 할 수 없다.
- 10k 행은 초기 평가 스크립트로 50개 포즈만 측정했고, 나머지 지표는 남아 있지 않다.

## 결과 2 — EX04 하네스 (`results/harness_1950k_vs_B0-B8.md`)

**조건**
- S1–S6, 시나리오당 궤적 20개, ψ 매핑 mirror.
- 15 Hz 관측, 100 Hz 제어, 관절 속도 ≤ π rad/s, 손목 hold 0.5 s.
- B0–B8 행은 `outputs/baselines_mirror_v2`(PR #14와 같은 코드)의 결과를 재사용했다.

**방법**
- **B9:** z를 64개 뽑는다. 비용 |ψ − ψ_belief| + 0.1‖q − q_prev‖ 순으로 상위 8개를 골라 각 샘플의 ψ에서 해석적 IK로 투영하고, 비용이 가장 작은 해를 쓴다. latent scale은 1.0이다.
- **B9r:** 투영 없이 latent scale 0.25로 뽑은 샘플을 그대로 쓴다.

| 시나리오 | B8 손목 mm / ψ P90° | B9 (1.95M) | B9r 120k → 1.95M | B9 jerk / B8 jerk |
|---|---|---|---|---|
| S1 ψ 한 바퀴 | 27 / 18 | 33 / 31 | 32 / 79 → **27 / 73** | 13244 / 7578 |
| S3 손목 불가능 | 28 / 19 | 30 / 20 | 43 / 47 → 32 / 34 | 6958 / 7152 |
| S4 1 s 가림 | 27 / 18 | 27 / 20 | 52 / 62 → 28 / 38 | 10668 / 8818 |
| S5 1 s 가림 | 28 / 22 | 28 / 24 | 43 / 52 → 33 / 48 | 10300 / 8504 |
| S6 1 s 가림 | 26 / 27 | 29 / 32 | 57 / 86 → 32 / 66 | 12154 / 8224 |

- B9는 120k와 1.95M에서 거의 같다. 투영 단계가 모델 오차를 지우기 때문이다.
- B9r는 학습이 길어질수록 손목 오차가 크게 줄었다. 하지만 프레임마다 새로 샘플을 뽑아서 ψ가 흔들린다.
- 계산 시간 P95: B9 약 41 ms, B9r 약 20 ms (GPU, 64 샘플).

## 한계

- 데이터에 자기충돌 필터가 없다. 학습량도 공개 Panda 모델의 37% 수준이다.
- 하네스는 시나리오당 20개 궤적, seed 2개로 돌렸다. 팀의 894개 공통셋(`share/11.15_exp/`)으로는 아직 돌리지 않았다.
- **하네스 API 호환:** `eval_ikflow.py harness`는 PR #14(`40eac1d`)의 `run_baselines.targets/run/metrics` 시그니처로 작성했다. 그 뒤 main에서 반환값과 인자가 바뀌었기 때문에, 현재 main에서 harness 모드를 돌리려면 이 부분을 맞춰야 한다. `static` 모드는 영향이 없다.
- 학습 가중치(200 MB)는 커밋하지 않았다. 필요하면 별도로 공유한다.

## 파일

| 파일 | 내용 |
|---|---|
| `nero_kin.urdf` | `share/sim/assets/nero_description.urdf`에서 만든 기구학 전용 사본. visual/collision을 제거하고 공백을 정규화했다(jrl 파서가 이중 공백에서 무한 루프). klampt용 1 mm 더미 충돌구를 넣었다 |
| `nero_robot.py` | jrl에 `nero` 로봇 등록 (flange = link7). FK가 `kinematics.py`와 1e-7 m 이내로 일치하는 것을 확인했다 |
| `run_script.py` | IKFlow 스크립트를 NERO가 등록된 상태로 실행한다. `IKF_MAX_TIME`이 있으면 시간 상한을 걸고, 유한값 검사를 거친 스냅샷을 주기적으로 저장한다 |
| `eval_ikflow.py` | `static`(정확도·ψ 커버리지·시간), `harness`(B9/B9r를 EX04 하네스로 실행) |
| `compare.py` | B9 결과를 B0–B8 `per_traj.csv`와 합쳐 요약 표 생성 |
| `results/` | 위 표의 원본 JSON·CSV·Markdown |

## 재현

IKFlow 저장소에는 라이선스가 없어서(`LICENSE`가 `#TODO`) 코드를 이 저장소에 복사하지 않았다. 별도로 clone해서 쓴다.

```bash
git clone https://github.com/jstmn/ikflow && cd ikflow && git checkout 2f4636e
# 로컬 패치 1줄: ikflow/config.py 에 `device = DEVICE` 추가 (training/lt_data.py가 이 이름을 import)
uv venv -p 3.10 && uv pip install torch==2.5.1 freia==0.2 numpy==1.24.4 pytorch-lightning wandb klampt==0.9.2 \
    scipy "setuptools<70" jparse-robotics OneEuroFilter
pip install git+https://github.com/jstmn/jrl@2ba7c39

E=<repo>/share/sim/experiments/ikflow
python $E/run_script.py scripts/build_dataset.py --robot_name=nero --training_set_size=2500000
# 이 데이터는 자기충돌 필터가 없지만, train.py는 __tag0=non-self-colliding 접미사가 붙은 파일을 찾는다
# → ~/.cache/ikflow/datasets/nero/ 의 각 파일에 해당 이름으로 심볼릭 링크를 걸었다
IKF_MAX_TIME=02:00:00:00 IKF_OUT=$E/paper.pkl IKF_KEEP_EVERY=100000 python -u $E/run_script.py scripts/train.py \
    --robot_name=nero --nb_nodes=12 --dim_latent_space=7 --coeff_fn_internal_size=1024 --coeff_fn_config=3 \
    --batch_size=128 --learning_rate=0.0005 --optimizer=ranger --gamma=0.979 --step_lr_every=39000 \
    --disable_wandb --disable_progress_bar --log_every 100000 --eval_every 19531 --checkpoint_every 100000000

python $E/eval_ikflow.py $E/paper.pkl static --n 200 --dim-latent 7 --out out_static
python $E/eval_ikflow.py $E/paper.pkl harness --dim-latent 7 --out out_harness   # PR #14 하네스 API
python $E/compare.py <share/sim>/outputs/baselines_mirror_v2/per_traj.csv out_harness/per_traj.csv
```
