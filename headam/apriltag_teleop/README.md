# AprilTag 손목 추종 Teleop 프로토타입 (코드)

로드맵 Phase 0.2 항목. 설계 설명·매핑 수식·한계는 [share/docs/prototype_apriltag_wrist_teleop.md](../../share/docs/prototype_apriltag_wrist_teleop.md)를 참고한다. 이 폴더는 그 프로토타입의 **코드와 개발 환경**을 기록한다.

> 정량 측정 없음. 지연, fps, tag 검출률, 추종 오차, 실기 구동 결과는 측정하지 않았다.

## 1. 개발 환경

| 항목 | 버전 |
|---|---|
| OS | Windows + WSL2 Ubuntu 22.04.5 LTS |
| ROS 2 | Humble |
| `ros-humble-moveit-servo` | 2.5.10 |
| `ros-humble-apriltag-ros` | 3.4.0 |
| `ros-humble-v4l2-camera` | 0.6.2 |
| Python 패키지 | OpenCV 4.5.4, SciPy 1.8.0, NumPy 1.21.5 |
| NERO SDK | pyAgxArm 1.0.0 |
| 기반 저장소 | [agilexrobotics/agx_arm_ros](https://github.com/agilexrobotics/agx_arm_ros) `ros2` 브랜치 commit `286633e` |
| URDF 서브모듈 | [agilexrobotics/agx_arm_urdf](https://github.com/agilexrobotics/agx_arm_urdf) commit `680fa92` |
| 카메라 | 일반 USB 웹캠 (Windows에 연결), 640×480 캘리브레이션 |

WSL2는 USB 웹캠 영상을 받을 수 없어서(`usbipd`로 isochronous 전송 불가) 카메라는 Windows에서 열고, 프레임을 TCP로 WSL에 보낸다.

## 2. 폴더 구성

`agx_arm_ros` 위에 얹는 형태로 올렸다. 원본 저장소 전체를 복사하지 않고, 새로 만든 파일과 원본 파일 수정분만 둔다.

| 경로 | 내용 |
|---|---|
| `overlay/` | 새로 만든 파일. `agx_arm_ros` 저장소와 같은 경로 구조 |
| `upstream_changes.patch` | 원본 파일 수정분 (`git diff`, commit `286633e` 기준) |
| `LICENSE.agx_arm_ros` | 기반 저장소 MIT License 원문 (patch에 원본 코드 일부가 포함되므로 함께 둠) |

### overlay/ (새 파일)

| 파일 | 역할 |
|---|---|
| `scripts/webcam_stream_windows.py` | **Windows에서 실행.** 웹캠 프레임을 JPEG로 압축해 WSL로 전송. 최신 프레임만 유지해 지연 누적 방지 |
| `src/agx_arm_moveit/scripts/agx_arm_net_camera` | WSL에서 프레임을 받아 `/camera/image_raw`, `/camera/camera_info` 발행. `camera_calibration` 결과 저장 지원 |
| `src/agx_arm_moveit/launch/tag_tracking.launch.py` | 카메라(`source:=v4l2` 또는 `net`) + `apriltag_ros` 실행 |
| `src/agx_arm_moveit/config/apriltag.yaml` | Tag 설정: 36h11, ID 0, 한 변 0.04 m → TF `camera → hand_tag` |
| `src/agx_arm_moveit/launch/servo.launch.py` | `demo.launch.py`(move_group, ros2_control, RViz) 위에 MoveIt Servo 추가 |
| `src/agx_arm_moveit/config/servo.yaml` | Servo 설정: 100 Hz 출력, 특이점·관절 한계·충돌 감속 |
| `src/agx_arm_moveit/scripts/agx_arm_tag_teleop` | **핵심.** Tag pose → 목표 tcp pose → Servo Twist 명령 (50 Hz). Clutch, tag 소실 처리 |
| `src/agx_arm_moveit/scripts/agx_arm_servo_keyboard` | Servo 동작 확인용 키보드 Cartesian / joint jog |

### upstream_changes.patch (원본 수정)

| 파일 | 수정 내용 |
|---|---|
| `config/agx_arm.srdf.xacro` | `ready` 자세 추가 (NERO `home`은 특이 자세라 Servo가 Cartesian 명령 거부). REVO2 인접 손가락 링크 간 collision 비활성화 |
| `CMakeLists.txt` | 새 스크립트 3개 설치 |
| `package.xml` | 의존성 추가: `moveit_servo`, `apriltag_ros`, `v4l2_camera`, `python3-scipy`, `python3-opencv` 등 |
| `README.md`, `README_EN.md` | `ready` 자세 설명 |
| `scripts/agx_arm_install_deps.sh` | 실행 권한만 추가 |

## 3. 적용 방법

```bash
# 1. 기반 저장소를 같은 commit으로 받기
mkdir -p ~/agx_arm_ws/src && cd ~/agx_arm_ws/src
git clone --recurse-submodules -b ros2 https://github.com/agilexrobotics/agx_arm_ros.git
cd agx_arm_ros && git checkout 286633e

# 2. 원본 수정분 적용 + 새 파일 복사
P=~/teleoperation/headam/apriltag_teleop
git apply $P/upstream_changes.patch
cp -a $P/overlay/. .

# 3. 의존성 설치, 빌드
sudo apt install ros-humble-moveit-servo ros-humble-apriltag-ros ros-humble-v4l2-camera
cd ~/agx_arm_ws && colcon build && source install/setup.bash
```

이 절차로 만든 트리가 개발 환경의 작업 트리와 파일 단위로 같은지 확인했다 (`diff -r`, 2026-11-02).

## 4. 실행 방법 (NERO, 시뮬레이션)

```bash
# 터미널 1: MoveIt + Servo
ros2 launch agx_arm_moveit servo.launch.py arm_type:=nero

# 터미널 2: 카메라 + tag 검출 (WSL에서 net 카메라 사용 시)
ros2 launch agx_arm_moveit tag_tracking.launch.py source:=net

# Windows: 웹캠 전송 (터미널 2의 net_camera가 뜬 뒤)
py webcam_stream_windows.py --camera 0

# 터미널 3: RViz에서 arm을 ready 자세로 옮긴 뒤 tag teleop 시작
ros2 run agx_arm_moveit agx_arm_tag_teleop
```

- **키:** `space` clutch on/off, `t` 회전 추종 on/off, `+`/`-` 동작 배율, `q` 종료
- **카메라 캘리브레이션 필수:** 캘리브레이션 파일(`~/.ros/camera_info/webcam.yaml`)이 없으면 `camera_info`가 비어 tag pose가 나오지 않는다. `camera_calibration` 패키지로 만들고 COMMIT하면 `agx_arm_net_camera`가 저장한다.
- **NERO 키보드 teleop:** `ros2 run agx_arm_moveit agx_arm_servo_keyboard --ros-args -p joint_count:=7` (기본값 6은 Piper 기준)
- **축 방향:** `agx_arm_tag_teleop`의 `axis_map` 기본값 `z,-x,-y`는 조작자가 카메라를 마주 보고 NERO가 −x 방향으로 작업하는 배치 기준이다. Piper는 `-z,x,-y`.

## 5. 주의

- **실기 사용 전 정리 필요:** 속도·관절 제한이 `servo.yaml`과 `agx_arm_tag_teleop`에 흩어져 있다. CONTRIBUTING 5절(safety 레이어 한 곳, 시뮬레이션 재생 테스트)을 만족하지 않으므로 실기에 그대로 쓰지 않는다.
- `agx_arm_net_camera`는 `0.0.0.0:5600`에서 인증 없이 프레임을 받는다. WSL2 기본 NAT 환경에서는 Windows 호스트에서만 접근할 수 있지만, 네트워크 설정을 바꾸면 노출될 수 있다.
- 연구 방법론의 입력이 아니다. 활용처(정답 rig, 실기 인프라)는 share 문서 C0와 로드맵 V.5를 참고한다.
