# Prototype: AprilTag 기반 상대 손목 추종 Teleop

> 작성일 2026-10-30 · 상태: **프로토타입 기록 (정량 측정 없음)**
>
> 연구 Contribution 후보가 아니다. Teleop 파이프라인과 실기 안전 인프라를 먼저 확인하기 위해 만든 프로토타입이며, [01 문서](01_literature_novelty_audit.md) Part C의 **C0**에서 위치와 활용 방안을 정리한다.

---

## 1. 개요

손등에 붙인 AprilTag 1개를 단안 웹캠으로 추적하고, **clutch를 켠 시점부터의 손 움직임(상대 변위·회전)** 을 NERO의 `tcp_link`에 더해 MoveIt Servo로 추종한다.

- 입력: 일반 웹캠 (RGB, 깊이 없음), AprilTag 36h11, ID 0, 한 변 0.04 m
- 추적 대상: 손등 tag의 6D pose 하나 (어깨·팔꿈치 없음)
- Solver: MoveIt Servo (Jacobian 기반 속도 제어). 위치 수준 IK 없음
- 코드 위치: `agilexrobotics/agx_arm_ros` 로컬 클론의 `ros2` 브랜치 위 **미커밋 변경** (MIT License). 이 저장소로 옮길지, 어떤 형태로 둘지는 미정

## 2. 구성

| 구성 요소 | 파일 (`agx_arm_ros` 기준) | 역할 |
|---|---|---|
| 웹캠 스트리밍 | `scripts/webcam_stream_windows.py` | WSL은 USB 웹캠을 받을 수 없어 Windows에서 JPEG 프레임을 TCP로 전송. 최신 프레임만 유지해 지연 누적 방지 |
| 네트워크 카메라 노드 | `src/agx_arm_moveit/scripts/agx_arm_net_camera` | 프레임을 `/camera/image_raw`, `/camera/camera_info`로 발행, `camera_calibration` 결과 저장 지원 |
| Tag 검출 | `launch/tag_tracking.launch.py`, `config/apriltag.yaml` | `apriltag_ros`로 TF `camera → hand_tag` 발행. `source:=v4l2` / `net` 선택 |
| Servo | `launch/servo.launch.py`, `config/servo.yaml` | `demo.launch.py` 위에 `moveit_servo` 추가, 100 Hz `JointTrajectory` 출력 |
| Tag teleop | `src/agx_arm_moveit/scripts/agx_arm_tag_teleop` | tag pose → 목표 tcp pose → Twist 명령 (50 Hz) |
| Keyboard teleop | `src/agx_arm_moveit/scripts/agx_arm_servo_keyboard` | Servo 동작 확인용 Cartesian / joint jog |
| `ready` 자세 | `config/agx_arm.srdf.xacro` | 특이 자세인 `home`(전 관절 0) 대신 Servo 시작 자세 |
| REVO2 collision 설정 | `config/agx_arm.srdf.xacro` | 인접 손가락 링크 간 collision 비활성화 (Servo self-collision 감속 방지) |

카메라 캘리브레이션 파일(`~/.ros/camera_info/webcam.yaml`, 640×480)은 있으나 재투영 오차는 기록하지 않았다.

## 3. 데이터 흐름

```
Windows webcam
  → webcam_stream_windows.py (TCP, JPEG)
  → agx_arm_net_camera (/camera/image_raw, /camera/camera_info)
  → apriltag_node (TF camera → hand_tag)
  → agx_arm_tag_teleop (50 Hz, TwistStamped, unitless)
  → servo_node (100 Hz, JointTrajectory)
  → arm_controller
```

## 4. 매핑

Clutch를 켠 시점의 tag pose $(p_{tag,0}, R_{tag,0})$ 와 tcp pose $(p_{tcp,0}, R_{tcp,0})$ 를 기준으로 한다.

$$
p_{target} = p_{tcp,0} + s\, M (p_{tag} - p_{tag,0})
$$

$$
R_{target} = \big(M\, R_{tag} R_{tag,0}^{-1} M^\top\big)\, R_{tcp,0}
$$

- $M$: 카메라 축 → 로봇 base 축 치환 행렬 (`axis_map`, 기본값 `z,-x,-y`: 조작자가 카메라를 마주 보고 NERO가 $-x$ 방향으로 작업)
- $s$: motion scale (기본 0.5, 0.1–2.0에서 키보드로 조절)
- 회전 추종은 키보드로 끌 수 있으며, 끄면 $R_{target} = R_{tcp,0}$

명령은 P 제어로 만든다.

$$
v = k_v (p_{target} - p_{tcp}), \qquad \omega = k_\omega \log(R_{target} R_{tcp}^{-1})
$$

$v$, $\omega$ 는 Servo 최대 속도(0.4 m/s, 2.0 rad/s)로 정규화한 뒤 노름 1로 자른다. Tag pose는 위치 lerp / 회전 slerp로 저역 통과 필터를 거친다.

| 파라미터 | 기본값 |
|---|---|
| `motion_scale` | 0.5 |
| `linear_gain`, `angular_gain` | 8.0 1/s |
| `smoothing` | 0.5 |
| `tag_timeout` | 0.2 s |
| `lost_timeout` | 1.0 s |
| `rate` | 50 Hz |

## 5. 안전 동작

- **Clutch:** tag가 보일 때만 켜진다. 끄면 정지 명령을 보내고, 조작자는 로봇을 움직이지 않고 손 위치를 옮길 수 있다.
- **Tag 소실:** 1.0 s 미만이면 정지 명령으로 대기, 넘으면 clutch 해제. 재검출 시 마지막 목표 pose에서 다시 기준을 잡아 손이 다시 나타난 위치로 튀지 않는다.
- **Scale / 회전 추종 변경:** 변경 시 기준을 다시 잡아 목표가 튀지 않는다.
- **Servo:** 특이점 감속 임계 100, 정지 임계 200 (Jacobian condition number), `joint_limit_margin` 0.1 rad, self-collision 0.01 m / scene 0.02 m, 명령 timeout 0.1 s.
- **`ready` 자세:** NERO `home`은 특이 자세라 Servo가 Cartesian 명령을 거부한다. `ready`(J2 0.1, J4 1.5, J5 −1.5708, 나머지 0)의 condition number는 SRDF 주석 기준 약 10–11.

**정리 필요:** 속도·관절 제한이 `servo.yaml`과 `agx_arm_tag_teleop`(gain, 정규화, clipping)에 흩어져 있다. CONTRIBUTING의 "safety 레이어 한 곳" 규칙에 맞지 않으므로 실기 코드 PR 전에 정리한다. 시뮬레이션 재생 테스트도 아직 없다.

## 6. 부수 확인: NERO URDF는 SRS 구조

공식 URDF(`agilexrobotics/agx_arm_urdf`, `nero/urdf/nero_description.urdf`)로 관절 축의 기하를 계산했다. 무작위 관절각 200개에서 확인한 결과다.

| 항목 | 결과 |
|---|---|
| J1·J2·J3 축이 한 점(어깨)에서 교차 | 최대 거리 0.000 mm |
| J5·J6·J7 축이 한 점(손목)에서 교차 | 최대 거리 0.000 mm |
| 팔꿈치(J4)가 상완·전완 축 위에 있음 (elbow offset 없음) | 최대 거리 0.000 mm |
| 상완 \|SE\| / 전완 \|EW\| | 0.310 m / 0.270 m |
| 좁은 관절 한계 | J4 −1.01 ~ 2.14 rad, J6 −0.733 ~ 0.873 rad |

→ 01 문서 Part D.3의 ψ 1-D 축소와 해석적 IK가 **URDF 기준으로는** 성립한다. 단, 손목 pose가 feasible할 때만 1-D 문제로 축소되며, J6 한계가 좁아 손목 자세가 불가능한 목표가 자주 생길 수 있다. 실기 관절값 vs FK 검증은 Stage 2(`02_kinematics_validation.md`)에서 수행한다.

## 7. 한계

- **상대 매핑:** 사람과 로봇의 절대 대응이 없어 Embodiment 오차, Elbow posture 오차, Reachability를 정의할 수 없다.
- **팔꿈치 미추적:** ψ를 관측하지도 제어하지도 않는다. 7-DoF 여유 자유도는 Servo의 pseudo-inverse(최소 노름)에 맡겨진다.
- **Servo 구조:** null-space 보조 목표를 넣을 인터페이스가 없어 Feasible-MAP solver의 뼈대로 쓰기 어렵다.
- **센서:** 단안 RGB에 물리 마커를 쓴다. 연구 설정(단일 RGB-D, 마커 없음)과 다르고, tag pose 정확도는 tag 크기·거리·각도에 의존한다.

## 8. 측정하지 않은 것

다음 수치는 **측정하지 않았다**: end-to-end 지연, 카메라 fps, tag 검출률, 추종 오차, 캘리브레이션 재투영 오차, 실기 구동 결과.
