# 2026-11-05 실험 리포트 — Gazebo Fortress Dry-Run 및 EX04 Oracle Benchmark 착수

> 실험일: **2026-11-05** · 장비: **NVIDIA Jetson AGX Orin** · OS: Ubuntu 22.04.5 LTS · JetPack 6.x (L4T R36.4.7) · ROS 2 Humble
>
> 프로젝트: [NERO Learned Retargeting](https://github.com/dldbdlf/teleoperation)
> 로컬 저장소: \`~/teleoperation_repo\` · 참고: [실험 로드맵](../docs/experiment_roadmap.md)
>
> **증거 수준:** 아래 실행·수치는 작업자가 전달한 2026-11-05 전체 작업 로그를 정리한 내용이며, 본 문서를 게시하는 과정에서 현장 장비를 재실행하거나 실제 코드·rosbag·원시 로그를 독립적으로 검증하지 않았다. **Gazebo / benchmark 코드의 원격 업로드는 아직 확인되지 않았다.**

## 0. 실험 배경과 범위

NERO 실물 팔 및 REVO2 Pro 핸드를 약 한 달간 사용할 수 없는 상황이다. 실기 없이 **IK 결과 → ROS 2 명령 → Gazebo 관절 움직임 → 관절 상태 피드백**을 검증할 수 있는 dry-run 기반 인프라를 먼저 구축했다.

같은 날 EX04의 비교 실험 골격을 최소 구현했으나 **제안 Oracle Feasible-MAP(P)은 미구현**이며 **G1 판정 불가**다.

이 문서는 **2026-11-05 기록만** 포함한다. 다른 로컬 기능 브랜치의 Gazebo 코드나 EX04 코드는 이 보고서 게시와 함께 푸시/병합하지 않는다.

## 1. Gazebo Fortress 환경 및 NERO 모델 등록

### 사용 소프트웨어
- Jetson AGX Orin: Ubuntu 22.04.5, ROS 2 Humble, JetPack 6.x / L4T R36.4.7
- **Gazebo Fortress**: \`ign gazebo 6.18.0\`, \`ros-humble-ros-gz\` / \`ros-gz-sim\`
- NERO 모델 메시(STL/DAE): \`agilexrobotics/agx_arm_urdf\`에서 다운로드; 느린 \`git clone\` 대신 tarball 사용
- \`agx_arm_description\` ROS 2 package 생성: 메시 및 URDF 설치, \`package://\` URI와 \`GZ_SIM_RESOURCE_PATH\`의 \`model://\` 해석 처리

### 검증/제약
- 최초 스폰 성공: \`ign model --list\`에서 \`nero\` 등록 확인.
- Gazebo GUI는 실제 STL/DAE 메시 로드 시 **\`Ogre2MeshFactory → GL3PlusVaoManager::createVao\` assertion** 후 abort.
- \`LIBGL_ALWAYS_SOFTWARE=1\`도 동일한 crash. Jetson 환경의 OGRE-Next/렌더링 호환 문제로 *추정*되며 정확한 원인은 별도 검증이 필요.
- **결정: GUI를 필수 요건에서 제외하고 \`ign gazebo -s\` headless physics server로 진행.**
- \`nohup/disown\` 백그라운드 세션이 불안정해, 실행 도구의 \`run_in_background:true\`로 운용 후 안정화되었다는 작업 기록.

> 확인 범위는 **해당 Jetson 환경**에서의 관측이다. Gazebo Classic(gazebo11)이 모든 arm64에서 불가능하다는 보편적 결론은 내리지 않는다.

## 2. Gazebo 관절 제어 및 PID 튜닝

URDF에 \`JointStatePublisher\` 및 일곱 개의 \`JointPositionController\` (\`ign-gazebo6\`) 플러그인을 추가했다.

- \`joint1 = 0.5 rad\` 명령 → \`axis1.position = 0.5009...\` 관측으로 실제 simulated joint movement 확인.
- PID 수정:
  - \`p_gain\`: 50 → 200
  - \`i_gain\`: 0.1 → 2
  - \`i_max\`: 1 → 10
- 튜닝 전 기록된 최대 관절 오차: **0.095 rad**
- 튜닝 후 기록된 최대 관절 오차: **약 0.022–0.025 rad**

**해석:** 명령과 관절 상태 간의 추종 오차가 감소했다. 정확한 부하/속도/응답시간 조건은 로그에 제공되지 않았으므로 일반적인 제어기 성능으로 확대 해석하지 않는다.

## 3. 해석적 IK → Gazebo 관절 명령 통합 시험

사용 코드: \`share/sim/kinematics.py\`의 SRS 기반 FK/해석적 IK/SEW ψ.

검증 순서:

1. 테스트 관절각 \`q_test\` 선택
2. \`T = FK(q_test)\`
3. \`q_IK = IK(T, ψ(q_test))\` 복원 및 오차 확인
4. \`q_IK\`를 Gazebo joint controller에 전달
5. Gazebo joint state를 읽어 \`q_IK\`와 비교

### 보고된 결과

| 항목 | 수치 | 의미 |
|---|---:|---|
| FK→IK 내부 오차 | 화면 표시 **0.0000 mm** | 반올림된 표시값. 실제 기구학 수치정확도는 원시 오차 필요 |
| 튜닝 후 IK→Gazebo 최대 관절 오차 | **0.0245 rad** | 제어 명령과 Gazebo 관절 상태 차이 |
| 상대적으로 큰 오차 | **J5–J7** | 손목 관절 컨트롤러 추가 튜닝 후보 |

**중요한 구분:** FK/IK 내부 오차와 Gazebo 구동 오차는 완전히 다른 실험 지표다. 후자는 PID/동역학/측정시점 등 영향을 포함한다.

## 4. ROS 2 노드/메시 구성 및 End-to-End 시험

신규 ROS 2 패키지 3개를 로컬 \`~/ros2_ws/src\`에 구성한 후 저장소 로컬 브랜치로 복사했다고 기록했다.

| 패키지 | 빌드 타입 | 주요 파일 / 역할 |
|---|---|---|
| \`nero_interfaces\` | \`ament_cmake\` | \`RobotCommand.msg\`: q_target, qdot_target, confidence, hold_reason |
| \`nero_sim_bridge\` | \`ament_python\` | \`command_router.py\`, \`ik_demo.py\`, vendored \`kinematics.py\`, \`nero_sim.launch.py\`, \`bridge.yaml\` |
| \`agx_arm_description\` | \`ament_cmake\` | URDF + 관절제어 플러그인, 메시 제외 |

### 문제: ROS 2 토픽 이름과 Gazebo 토픽 이름 불일치

Fortress \`JointPositionController\`가 사용하는 Gazebo 토픽 \`/model/nero/joint/joint1/0/cmd_pos\`의 독립 경로 세그먼트 \`0\`은 ROS 2의 \`rclpy.validate_topic_name\`에서 거부되었다.

\`<sub_topic>\` 방식은 사용 환경의 Fortress 플러그인에서 동작하지 않아, **\`ros_gz_bridge\`의 \`parameter_bridge\` YAML 매핑**으로 해결했다.

- ROS 2: \`/nero/joint/jointN/cmd_pos\`
- Gazebo: \`/model/nero/joint/jointN/0/cmd_pos\`

**검증된 것으로 보고된 end-to-end 경로**

\`\`\`text
ik_demo (FK/IK)
    -> RobotCommand
    -> command_router
    -> ros_gz_bridge
    -> Gazebo JointPositionController
    -> Gazebo joint state
    -> automatic command/state error check
\`\`\`

최종 자동 비교 결과: **\`max|err| = 0.0259 rad\`**, 실행 중 오류 없이 동작.

이는 **simulated joint execution 검증**이며 **실제 NERO 하드웨어의 제어 정확도나 안전성을 검증한 결과가 아니다**.

## 5. EX04 Oracle Synthetic Benchmark — 최소 착수

[실험 로드맵 EX04](../docs/experiment_roadmap.md) 기준으로 **먼저 공통 하네스와 단순 Baseline**을 구현했다.

### 로컬 구현 파일

- \`share/benchmark/scenarios.py\` — **S1 ψ Sweep (손목 고정)**만 구현
- \`share/benchmark/baselines.py\` — **B0 Wrist IK + Posture Home**, **B1 SEW Direction Follow**만 구현
- \`share/benchmark/metrics.py\` — ψ tracking error, branch-jump count, jerk proxy
- \`share/benchmark/run_ex04.py\` — 시험 실행

### 보고된 초기 결과

조건: \`READY\` pose, 사용자 기록의 feasible ψ arc 약 **140°/360°**, 결정론적 ψ sweep, noise/covariance 주입 없음.

| 방법 | ψ 평균 오차 | ψ P90 | Branch jumps | Jerk proxy |
|---|---:|---:|---:|---:|
| **B0** Wrist IK + Posture Home | **90.00°** | **162.00°** | **0** | 로그에 값 없음 |
| **B1** SEW Direction Follow | **13.65°** | **52.10°** | **2** | **0.4018** |

**Sanity check:** 균등 ψ sweep에서 ψ를 추종하지 않는 B0의 평균 원형 오차는 이론적으로 90°이므로 관측된 90°와 일치한다. 다만 **이 단일 확인만으로 하네스 전체가 올바르다고 확정할 수는 없다.** wrap boundary, infeasible sample labeling, branch-jump threshold, jerk 정의 등을 별도 unit test로 검증해야 한다.

**정합성 확인 필요:** 기존 [02 기구학 보고서](../docs/02_kinematics_validation.md)는 ready pose에 대해 feasible ψ 총 길이를 **220°**로 기록한다. 이날 로그의 **약 140°**가 같은 정의/동일 target/동일 limits인지 확인해야 한다. 140°는 220°의 여집합 길이일 수 있지만 **검증 전 원인을 단정하지 않는다**.

**주의:** B0/B1 결과는 **Baseline 하네스 디버그 수치**다. 제안 방법(P) 성능이나 IROS/CoRL 기여에 대한 결론으로 사용할 수 없다. jerk proxy는 실제 \`rad/s³\` jerk 지표와 구분할 것.

## 6. 아직 완료되지 않은 것 — G1 판정 불가

- [ ] S2–S6 scenario 미구현 (near-extension, J6/wrist infeasibility, elbow occlusion, wrist dropout, reacquisition/direction reversal).
- [ ] B2 adaptive elbow weighting, B3 One-Euro, B4 Kalman, B7 uncertainty-weighted constrained IK 미구현.
- [ ] **제안 Oracle Feasible-MAP solver P 목적함수·해법·branch continuity·covariance 주입 규약 미설계/미구현.**
- [ ] Synthetic noise와 **정답 covariance Σ** 미주입; seed 한 개만 사용.
- [ ] EX04 multi-seed, 고정된 공통 safety 제약, P50/P95 latency 및 G1 통계 미측정.
- [ ] 본 보고서 시점에서 실제 NERO 팔/REVO2 Pro 하드웨어 없음.
- [ ] IRB/기관 심의 여부 확인 미착수; EX05 인간 데이터 수집 전 해결 필요.

### 즉시 실험 우선순위

1. **공동 설계:** feasible human belief와 robot-feasible action을 분리한 P 목적함수, IK branch continuity와 불가능한 wrist target relaxation, velocity/collision 안전 제한 정의.
2. **S2 near-extension 및 S3 J6/SE(3) infeasible**를 우선 구현 → 현재 모델의 강한 failure case를 먼저 확인.
3. **S4 elbow-only occlusion, S5 wrist loss, S6 reacquisition/reversal** 추가 → time-varying uncertainty의 필요성 확인.
4. B2/B3/B4/B7를 공통 데이터와 동일 safety, 동일 제어 주기에서 구현.
5. 정답 covariance, dropout duration, multi-seed, primary metric 사전등록 후 **P vs strongest baseline**을 실행하고 G1 판정.

## 7. Git 작업 현황 (2026-11-05 작업자가 보고한 로컬 상태)

| 로컬 브랜치 | 포함 내용 | 2026-11-05 보고 당시 |
|---|---|---|
| \`feat/gazebo-sim-bridge\` | Gazebo dry-run 및 ROS 2 bridge (\`share/robot/\`) | 로컬 커밋 완료, GitHub push 실패 |
| \`feat/ex04-oracle-benchmark-baselines\` | EX04 하네스 (\`share/benchmark/\`) | 로컬 커밋 완료, GitHub push 실패 |
| \`docs/11.05-exp-summary\` | 이날 요약 (\`share/11.05_exp/README.md\`) | 로컬 커밋 완료, GitHub push 실패 |

HTTPS 토큰·SSH key 미설정으로 로컬에서 \`git push\` 실패 (\`could not read Username\`, \`Permission denied (publickey)\`).

**리포트의 게시와 로컬 코드 푸시는 다르다.** 원격에 이 README를 별도 커밋한다고 해서 위 3개 로컬 브랜치가 원격에 업로드된 것은 아니다. 로컬 커밋들을 원격에 올리려면 해당 시스템에서 별도로 인증하고 푸시하거나, PR별 변경 내용을 연결된 GitHub 인터페이스로 업로드해야 한다.

## 8. 결론 및 상태

- **완료로 보고된 내용:** Fortress headless NERO 스폰, 7 joint controller, ROS2→Gazebo bridge→joint feedback dry-run, IK command path와 최대 관절 오차 수치, EX04 S1/B0/B1 하네스.
- **보류:** Headless Gazebo 사용은 현재 환경의 우회책이며 렌더링 문제의 근본 해결이 아님.
- **미완료:** P(Oracle Feasible-MAP), S2–S6, B2–B4/B7, covariance/noise, multi-seed, G1, 실제 NERO/REVO2 평가.
- **연구 결정:** 임의의 Oracle P를 성급하게 구현하지 않고 목적함수/분기 연속성/공분산 주입을 공동 설계한 후 본 실험에 착수.

> **11월 5일 실험 기록이며 논문 결과 아님.** 측정값·환경·실패 사례는 추후 원본 실행 로그/코드와 재대조해야 한다.
