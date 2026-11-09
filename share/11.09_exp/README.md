# 2026-11-09 Teleoperation 당일 실험 예정 로드맵

> **날짜:** 2026-11-09 (월, KST) · **상태:** PLANNED / NOT STARTED (실험 완료 보고서 아님)
>
> **환경:** Jetson AGX Orin / Ubuntu 22.04.5 / ROS 2 Humble, Intel RealSense **D415 1대**
>
> **로봇:** AGILEX NERO 7-DoF (현재 장비 부재 구간에서는 Gazebo Fortress headless), BrainCo REVO2 Pro (후속)
>
> **연구:** [전체 로드맵](../roadmap.md) · [EX00–EX10 실험 프로토콜](../docs/experiment_roadmap.md) · [11/05 Gazebo+EX04 기록](../11.05_exp/README.md)
>
> **중요:** Glove / AprilTag는 **Ground Truth(GT) 수집 및 캘리브레이션 용도만**. 제안 **온라인 추론·제어 입력은 D415 RGB-D만**. D435i/두 번째 카메라는 오늘 필수 구성에 포함하지 않는다. 장갑의 정확한 제조사·모델·SDK·출력 채널은 아직 확인되지 않았으므로 기능을 단정하지 않는다.

## 0. 당일 목표와 연구 범위

### 오늘의 최종 성공 시나리오

```text
                     고정 RealSense D415 (단일 RGB-D)
                                    |
                    +---------------+---------------+
                    |                               |
    [온라인 추론/제어 경로]                   [GT / 계측 경로]
     RGB + aligned Depth                     AprilTag 검출
               |                          + glove sensor stream
       Markerless S/E/W                     (GT / 학습·평가 전용)
               |                                  |
     Wrist goal + human ψ                동기화·관절 중심 보정
               |                                  |
      Existing NERO IK                  GT_S / GT_E / GT_W / GT_ψ
               |                           + GT_finger(지원 시)
         RobotCommand                              |
               |                           GT validity flags
      ROS2 -> Gazebo Fortress                      |
               |                                  |
       Joint state feedback -----------------> 오차·시간·실패 분석
```

- **오늘 필수(MVP):** (1) 카메라 고정·기록, (2) 장갑 데이터의 실제 출력 확인, (3) Tag 6D pose 검출 및 카메라 좌표계 정합, (4) 공통 timestamps로 저장, (5) 기존 IK를 통해 **녹화된 pose / live pose → Gazebo command → joint feedback** 1개 시퀀스 재생.
- **오늘 권장(여유 시):** 팔꿈치 가림 인식, ψ singular 상태 표시, 최초 GT-vs-markerless 오차 도표, 정지/복귀 로직.
- **오늘 제외:** 제안 논문의 Oracle Feasible-MAP(P) 성능 검증, 새 multimodal network 학습, 10-task 사용자 연구, REVO2 Pro 실기 완성. **기존 IK E2E 통합을 연구 기여 완료로 주장하지 않는다.**
- **장비 전제:** 실제 NERO/REVO2 Pro 복귀 전에는 Gazebo 전용. `share/robot/` / `share/benchmark/` 코드가 원격 `main`에 존재한다고 가정하지 않고, Jetson 로컬 `~/teleoperation_repo`와 `~/ros2_ws/src`를 먼저 확인한다.

## 1. 오늘 수행 순서 (의존성 기준)

| 우선 | 작업 ID | 수행 내용 | 선행조건 | 오늘 산출물 |
|---|---|---|---|---|
| **필수** | D0 환경·형상 동결 | 설치 소프트웨어, rosbag/시간 기준, 로컬 Gazebo/EX04 코드 상태, 장갑 모델/SDK 확인 | 없음 | `hardware_inventory.md`, `run_manifest.json` |
| **필수** | D1 D415 고정 | 카메라 설치·정렬·노출/깊이 유효성/캘리브레이션 기록 | D0 | `camera_setup.yaml`, sample RGB-D |
| **필수** | D2 장갑 SDK·출력 검증 | 관절값/주기/단위/시스템 타임스탬프/캘리브레이션 확인 | D0 | `glove_capability.md`, glove log |
| **필수** | D3 AprilTag GT rig | 몸통·상완·전완·손등 Tag 검출, ID/크기/관절 오프셋 보정 | D1 | `tag_layout.yaml`, extrinsic/offset QC |
| **필수** | D4 센서 동기화 & GT 생성 | D415·Tag·glove time alignment, GT-valid mask 생성 | D1–D3 | `aligned_gt.parquet` 또는 CSV |
| **필수** | D5 Markerless Pose | 태그·장갑 입력 **미사용**. D415 3D S/E/W, validity, ψ 추정 | D1 | `vision_estimates.csv` |
| **필수** | D6 기존 Solver + Gazebo | human→NERO 좌표계 변환, 기존 analytic IK, safe router, headless joint_states | D5 + 로컬 bridge | `robot_commands.csv`, `joint_states.csv` |
| **권장** | D7 비교·예외 처리 | GT 품질/pose 오차/관절 추종·가림·복귀 그래프 및 실패 분석 | D4–D6 | `day_report.md`, figures |

**병렬 실행:** Track A = D1/D2/D3/D4/D5 (센서·GT·비전), Track B = D0 중 bridge 확인 + D6 (Solver·Gazebo). 공유 인터페이스는 D0에서 먼저 정한다.

## 2. D0 — 장비/기존 코드 점검 (가장 먼저)

1. Jetson에서 `~/teleoperation_repo` 및 `~/ros2_ws/src` 확인. 보고된 로컬 브랜치 `feat/gazebo-sim-bridge`, `feat/ex04-oracle-benchmark-baselines`에 실제 코드가 남아 있는지 확인한다. 다른 브랜치 파일을 무심코 덮어쓰거나 `main`에 직접 push하지 않는다.
2. `ros2 topic list`, RealSense D415 connected serial/model, camera_info, color/depth topics 확인. 다른 카메라 input 비활성.
3. Gazebo Fortress headless `ign gazebo -s`, `nero_sim_bridge`, `nero_interfaces`, `agx_arm_description`의 실제 빌드·실행 경로 확인. GUI OGRE 오류는 오늘의 필수 실험 범위 밖.
4. 장갑 제조사/모델/버전/연결방식(USB, BLE, Wi-Fi 등)/SDK 유무를 실제 장비 및 매뉴얼로 확인. **장갑이 손목 글로벌 위치·방향을 제공한다고 가정하지 않는다**.
5. `run_id`, experiment date, camera serial, ROS distribution, software git SHA, URDF SHA, glove firmware, calibration version, dataset consent status, frame_id, unit convention, sensor and host timestamps 등 메타데이터를 확정.

**완료 조건:** SDK와 센서 데이터 접근경로 확인, 두 트랙 간 키 이름/단위/TF 계약 합의. 실패해도 원인을 문서화.

## 3. D1 — D415 고정 시점 선택·캘리브레이션

**목표:** D415 단일 시점으로 어깨부터 손끝까지 시야에 들어오며 움직임 중 관측 품질이 유지되는 위치 선택.

1. D415를 삼각대/고정 브래킷에 설치하고 렌즈 높이, 설치 위치, 광학축 방향, 작업자 서 있는 위치·작업 영역을 기록한다. 정면과 약한 측면 위치를 간단히 비교하되, **최종 시험 중 카메라 위치를 이동하지 않는다**.
2. RGB, depth, aligned-depth, camera_info 데이터를 함께 기록한다. 2026-11-03 기록의 색상 약 14.6 FPS는 기존 **동시 카메라** 설정 수치이며 D415 단독 RGB-D 성능으로 전용하지 않는다.
3. Intrinsics(fx/fy/cx/cy), distortion, 3D 역투영, color-depth alignment 확인. 알려진 길이의 물체를 두세 위치에서 측정해 거리·시점별 오차와 오차의 spread를 기록한다.
4. RGB 마커 인식과 RGB-D pose 추정에 동일 camera optical frame을 사용한다. 고정 world frame (실험 공간)과 torso frame (작업자 움직임)의 TF를 혼용하지 않는다.
5. 고정 시점의 재현성을 위해 tripod/photo 또는 배치 스케치, 거리 측정, exposure preset, depth preset, USB connection 기록.

**지표:** color/depth measured FPS, frame drop, timestamp skew, valid-depth %, marker reprojection RMS, known-length 3D error [mm], camera-to-world transform repeatability [mm/deg].

**G0 준비 조건:** 재관측/재측정 시 일정한 TF와 일관된 depth; 정밀 GT 정확도를 알기 전에는 GT 오차를 0으로 취급하지 않는다.

## 4. D2 — Glove 데이터 확인/정합 (GT 전용)

**주의:** 정확한 제품 모델이 제공되지 않았으므로 SDK 명령·ROS 패키지·실제 측정 관절 수를 임의로 기재하지 않는다. 먼저 장비에서 확인한다.

1. 제조사 가이드대로 power/pairing/driver/SDK 실행 후 raw stream을 덤프한다. 가능한 출력: 손가락 굴곡 센서, 추정 손가락 관절각, quaternion/IMU, calibration quality, validity, device timestamp. **실제로 제공하는 항목만 manifest에 등록**.
2. 장갑 없는 손가락 가만히/펴기/주먹/개별 손가락 움직임을 기록하고 원점 drift, 단위(deg/rad), sample-rate, clipping, finger mapping을 확인한다.
3. 장갑을 착용한 상태에서 hand landmark 검출 오류 가능성을 별도로 표시한다. **장갑 착용 영상은 marker-free / glove-free 실사용 성능을 직접 대표하지 않는다.**
4. 장갑 데이터는 REVO2 Pro 손가락 GT 또는 정합 비교용으로 보존. 장갑이 6-DoF hand pose를 제공하더라도 **world alignment / drift calibration** 없이는 wrist SE(3) GT라고 부르지 않는다.
5. 장갑에서 출력하지 않는 shoulder/elbow position은 **장갑 GT로 채우지 않는다**.

**산출물:** `glove_capability.md` (지원 채널·단위·장치 시간), raw glove log, calibration metadata, 전송 지연 및 missing sample 비율.

## 5. D3 — AprilTag를 이용한 관절 위치 GT

### 기본 배치안 (단일 D415 관측)

| Tag ID(예시; 실제 중복 없이 설정) | 부착 위치 | 직접 측정 | 필요한 보정 |
|---|---|---|---|
| T0 | 몸통/가슴에 견고하게 고정 | rigid torso tag 6D | 어깨 관절 중심까지의 체형별 오프셋 |
| T1 | 상완의 단단한 지지부 | upper-arm tag 6D | 상완 좌표계 및 어깨·팔꿈치 관절 중심 오프셋 |
| T2 | 전완의 단단한 지지부 | forearm tag 6D | 전완 좌표계 및 팔꿈치·손목 관절 중심 오프셋 |
| T3 | 손등/단단한 손목 지지부 | dorsal-hand tag 6D | 손목/hand pose 오프셋, 글러브 간섭 여부 |

**주의:** AprilTag의 translation은 **Tag 중심 위치**이지 관절 중심 GT가 아니다. 고정 부착 위치, Tag 크기/인쇄 크기, index, orientation, 물리 오프셋을 개별 기록한다.

**캘리브레이션 절차**
1. 고정된 알려진 길이·각도 rig로 Tag pose의 반복성/거리·회전 오차 평가, `solvePnP` + measured intrinsics와 reprojection check.
2. 팔 길이/관절축을 정적인 한 자세로만 단정하지 않고, 완만한 굴신 동작의 기능적 캘리브레이션 및 관절 중심 모델을 사용한다. 피부·의복의 움직임(soft-tissue artifact)과 Tag slipping을 점검한다.
3. 두 인접 Tag에서 계산한 같은 팔꿈치 중심(upper-arm 기반, forearm 기반)이 맞는지 비교해 residual·분산을 보고한다. 큰 불일치가 있으면 GT-valid=false.
4. Shoulder S, Elbow E, Wrist W, ψ를 카메라 또는 기준 torso frame으로 복원한다. ψ 정의가 불안정한 팔 완전 펴기 구간은 `psi_defined=false` 처리한다.
5. Tag 수가 부족하거나 팔 움직임 중 계속 가려진다면 visible GT만 제공한다. 신뢰 없는 오프셋을 사용해 모든 프레임에 정답을 강제로 채우지 않는다.

**최소 품질 지표:** Tag detection ratio, per-Tag corner reprojection error [pixel], tag pose repeatability, joint-center residual [mm], S/E/W GT validity %, ψ-defined %, calibration drift.

## 6. D4 — 동기화 및 GT 파일 생성

D415 frame/source timestamp와 glove의 device/source timestamp는 같은 clock domain이 아닐 수 있다. 모든 센서에 **source time, host receive time, aligned time**을 별도 저장하고 sensor delay/jitter를 측정한다.

- 같은 Jetson 수신이라도 장치별 latency offset이 존재한다. 기능적 이벤트(손 움직임/가속도 변화)로 상대 시간 오프셋을 추정할 수 있으나, wrist marker/event 품질이 낮으면 offset을 임의로 정하지 않는다.
- RGB/Depth/Tag 동기화: 같은 D415 프레임에서 Tag GT pose·markerless Pose가 대응되도록 한다.
- glove 동기화: 타임스탬프 보정 후 nearest-neighbor / 신뢰 구간 내 interpolation으로 정합. 허용 시간 오차는 measured frame period 및 실제 속도에 근거해 정한다.
- 저장 schema 예시 (메타데이터는 필드별 units와 calibration version을 추가):

```yaml
frame_id: D415_color_optical_frame
t_rgb_ns: ...
t_depth_ns: ...
t_glove_source_ns: ...     # 지원 시, 없으면 null
t_glove_host_ns: ...
t_aligned_ns: ...
rgbd_frame_valid: true
gt:
  shoulder_xyz_m: [null, null, null]
  elbow_xyz_m: [null, null, null]
  wrist_xyz_m: [null, null, null]
  psi_rad: null
  finger_angles_rad: null  # 장갑 지원 채널만
  valid_shoulder: false
  valid_elbow: false
  valid_wrist: false
  valid_psi: false
  glove_valid: false
  tag_ids_visible: []
  gt_quality_flags: []
prediction:
  shoulder_xyz_m: [null, null, null]
  elbow_xyz_m: [null, null, null]
  wrist_xyz_m: [null, null, null]
  psi_rad: null
  visible: []
  depth_valid: []
robot:
  q_cmd_rad: [null, null, null, null, null, null, null]
  q_sim_rad: [null, null, null, null, null, null, null]
  solver_status: NOT_RUN
```

**표기:** 위 값들은 예시 형식이며 측정된 수치가 아니다.

## 7. 핵심 과제: 단일 카메라 GT와 Occlusion 한계

**카메라 한 대로 online teleop을 하는 것과 카메라 한 대에서 완전한 GT를 만드는 것은 서로 다르다.**

- **Visible-GT:** D415에서 Tag들이 보이고 calibration valid인 프레임 → Tag 기반 S/E/W/ψ 비교가 가능(측정 불확실성을 함께 보고).
- **Artificial-occlusion evaluation:** 실제 Tag GT가 보이는 기록에서 online pose estimator의 관측 또는 입력 keypoint를 숨기고 loss/recovery 측정. 실제 물체에 의한 occlusion과 같다고 단정하지 않는다.
- **Natural-occlusion evaluation:** 실제 몸통/물체가 팔과 해당 Tag를 가리면 D415로 그 관절 GT를 직접 측정할 수 없다. 해당 구간은 `gt_valid=false`로 기록, pose GT 정확도 지표의 primary 분석에서 제외. 안전 중단/가림 검출/복귀 동작 등 ground-truth-free 측정은 별도로 평가 가능.
- **장갑 한계:** 손가락 flexion GT가 있어도 별도 절대 위치 추적 기능이 없다면 가려진 shoulder/elbow/wrist 3D GT를 대체하지 못한다.
- **Tag/장갑 데이터 누출:** Tag가 보이는 이미지로 학습/평가하면 마커가 visual shortcut이 될 수 있다. Instrumented calibration/GT 세션과 **Tag·장갑 없는 독립 online 실험 세션**을 분리한다. Tag mask/inpainting 실험은 별도 검증용이며 marker-free 자연 영상의 완전한 대체로 주장하지 않는다.
- 추후 **natural occlusion에서 연속적인 정밀 관절 GT**가 필수라면 관측 독립적인 추가 측정 장비가 필요할 수 있다. 그러나 오늘 프로토콜은 **한 대 D415 유지**를 우선하고 GT 누락을 숨기지 않는다.

## 8. D5 — Markerless 3D Human Pose (실제 입력 경로)

- D415 RGB를 2D human landmark detector에 입력하고 aligned depth로 S/E/W를 3D 역투영. torso/shoulder 기준 좌표계와 팔 길이 정규화 정의.
- 보이지 않는 joint pixel에서 임의 배경 depth를 가져오지 않도록 robust ROI/depth consistency/temporal bounds 및 `depth_valid=false` 적용.
- Human wrist orientation이 신뢰 불가능하면 **위치-only IK 모드**와 orientation-enabled 모드를 분리해 기록한다. 2D S/E/W만으로 손목 full 6-DoF orientation GT가 자동 생성되지 않는다.
- ψ 계산은 S/E/W의 기하 정의와 ±π wrapping, elbow offset h, near-straight undefined 처리를 갖추어야 한다.
- Tag detector나 glove 센서 값을 online pose estimator·solver input에 넣지 않는다. **제안 시스템은 독립된 RGB-D 입력만 사용**.
- 표시용 overlay: RGB + observed keypoints + depth_valid + estimated 3D skeleton + GT validity + elapsed time. GT overlay가 online model의 feature로 다시 유입되지 않도록 데이터 흐름 분리.

**지표:** 3D S/E/W MAE/P90 (GT valid subset), wrist pose validity, ψ circular MAE/P90 (`psi_defined` subset), depth false acceptance, frame FPS, frame-age/latency.

## 9. D6 — Human → NERO Solver → Gazebo E2E

**오늘 적용 Solver:** 이미 검증된 NERO SRS 해석적 IK를 사용하는 **baseline B0/B1**. **제안 Feasible-MAP P는 아직 미구현**이므로 오늘 완료라고 쓰지 않는다.

1. Camera `D415_color_optical_frame` → torso frame → NERO base frame. 작업자 신체 길이를 NERO 0.310m/0.270m 상완/전완에 매핑; 원점/축/손목 목표 방향의 기준자세를 캘리브레이션한다.
2. S/E/W로 human ψ를 계산, 로봇 wrist goal 생성. Wrist pose가 없거나 손 방향이 불확실하면 `position-only` 또는 최근의 안전한 orientation hold를 명시적으로 기록한다.
3. 이전 `q_prev`에 가까운 feasible IK branch 선택; joint/velocity/acceleration limit, workspace, stale-frame timeout, HOLD/SAFE_STOP을 **solver 외부 안전 layer**에서 일관되게 적용.
4. 로컬의 `nero_interfaces/RobotCommand.msg`·`nero_sim_bridge`·Gazebo Fortress headless가 존재하고 빌드되는지 확인. `share/sim/kinematics.py`와 vendored copy 간 파라미터/버전 불일치를 점검.
5. `robot q_target` → command_router → ros_gz bridge → joint controller → `q_sim`. 명령→관절 feedback 시간 지연과 관절 추종 오차를 별도 측정.
6. **실패 조건:** invalid/old wrist observation, IK no-solution, 과도한 branch jump, threshold 초과 관절 속도, timestamp loss → hold/stop. 사람이 다시 보일 때는 목표 관절로 즉시 점프하지 않고 제한된 전환.
7. 첫 단계는 **recorded replay** → 다음으로 live. 실물 NERO 명령은 오늘 범위에서 제외.

**현재 근거:** 11/05 로그의 bridge 단일 동작 오류 없는 실행 및 최대 관절 오차 **0.0259rad**는 이전 보고값. 새 관절 추종 수치와 혼동하지 않는다.

## 10. D7 — 당일 테스트 행렬(제안, 아직 실행 안 함)

연구 피험자 데이터 수집/연구 목적 영상 녹화는 소속기관의 IRB·동의·저장 정책 확인 후 실시한다. 승인 전에는 **체커보드/물체·합성 관절 궤적** 등 사람을 녹화하지 않는 시스템 점검만 실행한다.

| Test | 입력 상황 | 계측/기대되는 검증 | 지표/성공 조건(개념) |
|---|---|---|---|
| **T0** | 고정 rig·길이 알려진 물체 | RGB-D/Tag 위치 정확도 | 재측정 error/GT-quality flags |
| **T1** | 정지 자세 (승인 후) | 장갑/Tag drift·sync·관절 offset | GT valid ratio, source-to-aligned lag |
| **T2** | 손목 좌우/앞뒤 이동 (승인 후) | 인간 wrist → Robot IK → Gazebo | target vs q_sim error / latency |
| **T3** | 손목 고정, 팔꿈치 회전 (승인 후) | ψ 보존·IK branch | ψ error / branch jumps |
| **T4** | 팔 거의 완전히 펴기 (승인 후) | ψ singularity/undefined | undefined flags / safe response |
| **T5** | 팔꿈치가 몸통 뒤로 이동 (승인 후) | self-occlusion + GT-valid mask | observed vs missing, recovery |
| **T6** | 정답이 보이는 recorded sequence에 가림 시뮬레이션 | known-GT loss/recovery simulation | dropout duration별 quality |
| **T7** | 손가락 개폐 (승인 후) | glove channel mapping/QC | sensor angle/drift; robotic hand command는 제외 |

**반복:** T0는 서로 다른 거리/각도에서 최소 3회 점검(실험 설계 제안), T1~T7은 허용된 범위에서 단순 반복 기록. 통계적 의미를 갖는 인원수나 실기 Task Success Rate를 오늘 목표로 두지 않는다.

## 11. 오늘 PASS / PARTIAL / BLOCKED 판정

**PASS (오늘 End-to-End 기반 구축)**
- D415 single-camera RGB/Depth + camera_info sync와 고정 TF 보존.
- 장갑 실제 출력 채널 확인 및 시간 데이터 로깅. 손목/팔꿈치 GT를 출력한다고 과장하지 않음.
- Tag 검출 + 보정 + valid GT 필드 생성 및 tag-loss시 GT invalid 처리.
- 승인된 범위의 captured/replayed 입력 또는 synthetic poses → S/E/W → existing NERO IK → Gazebo → joint feedback 1개 완주.
- time-aligned `input, prediction, GT-valid, RobotCommand, q_sim` 로깅 및 성능/실패 측정 도구 확인.

**PARTIAL:** hardware/SDK/ROS/GT의 한 단계만 되거나 recorded replay만 되는 경우. 미검증 상태를 명시해 보고.

**BLOCKED:** 장갑 SDK 작동 불가 / GT calibration failure / ROS2 bridge 로컬 부재 / IRB 확인 대기. 각 장애에 재현 명령·로그·완료되지 않은 의존성을 기록.

**별도의 논문 Gate는 여전히 미통과:** EX04 G1은 Oracle Feasible-MAP(P) 및 비교군이 완성되어야 판정할 수 있고, EX05/G2a는 GT 품질과 실제 가림 자료가 필요하다.

## 12. 결과물 규격 및 프로젝트 내 저장 (실험 종료 후)

계획 파일(이 문서)은 GitHub에 저장하고, 실행 이후 아래 파일을 생성한다.

```text
share/11.09_exp/
  README.md                    # 본 날짜의 사전 실험 계획
  report.md                    # 실험 완료 뒤 실제 측정값과 PASS/PARTIAL/FAIL
  hardware_inventory.md
  glove_capability.md
  camera_setup.yaml
  tag_layout.yaml
  calibration_qc.md
  run_manifest.json
  figures/                    # 얼굴/신원 노출 없는 도표 및 실험 사진(공개 승인 시)
  results_summary.csv         # 원시 영상 아닌 요약 측정치
```

원시 RGB/Depth/장갑/Tag logs 및 사람 영상은 **Git 외부의 접근 제어된 저장소**에서 관리, 공개 전 동의·IRB·개인정보 검토. 시뮬레이터 raw logs와 사람이 포함된 데이터는 별도로 저장.

모든 run은 `run_id`, 실험 시작/종료 timestamp, camera/glove/Tag firmware or version, ROS/Gazebo config, calibration, git SHA, q initial, coordinate frame, camera FPS, unit, observed errors, missingness, PASS/PARTIAL/FAIL 및 실패 재현 단계 기록.

## 13. 오늘 이후 연결되는 다음 실험

- **EX01/EX02:** D415 단독 markerless GT validation + real occlusion과 artificial dropout의 차이 평가.
- **EX03:** ψ 140° vs 220° 이전 로그 불일치 및 IK branch/near-straight feasibility 재검증.
- **EX04 (G1):** S2–S6, B2/B3/B4/B7, Oracle covariance, multimodal belief 필요성 시험, **Oracle Feasible-MAP P 목적함수 및 branch continuity** 설계/구현.
- **EX05/EX06:** GT-valid mask 기반 occlusion dataset → calibrated belief. 피험자·원본 episode별 분리 및 장갑/Tag 누출 검증.
- **IROS / CoRL:** 첫 통합 시연은 시스템 준비도 증거이며 연구 신규성은 **Belief-Constrained Retargeting 개선량 + 실사용 일반화**로 별도 증명.

> **문서 상태:** PLANNED. 이 파일 업로드를 D415/장갑/Tag/Gazebo 실험 실행 성공으로 해석하지 않는다.
