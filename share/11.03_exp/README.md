# Jetson AGX Orin — ROS 2 Humble + RealSense D415/D435i 환경 구축 리포트

- 작성일: 2026-11-03
- 장비: NVIDIA Jetson AGX Orin (사용자 계정 `inha`)
- 관련 프로젝트: NERO Learned Retargeting — RGB-D 기반 사람→로봇 원격조작 (AGILEX NERO 7-DoF 팔 + BrainCo REVO2 Pro 핸드)

## 1. 시스템 환경

| 항목 | 값 |
|---|---|
| OS | Ubuntu 22.04.5 LTS (Jammy), arm64 |
| JetPack | 6.x (L4T R36.4.7) |
| ROS 2 | Humble (apt 설치, `~/.bashrc`에 source 등록) |
| 카메라 | Intel RealSense D415 (고정 씬 카메라), D435i (IMU 탑재, 손목 장착용) |

## 2. librealsense2 SDK 설치

- 소스: https://github.com/IntelRealSense/librealsense (v2.59.1)
- Jetson(aarch64)이라 x86 apt 저장소 대신 소스 빌드
- `-DFORCE_RSUSB_BACKEND=ON` 옵션으로 빌드 (커널 패치 불필요)
- 설치 경로: `/usr/local` (`rs-enumerate-devices`, `librealsense2.so` 등)

**문제 및 해결**
- udev 규칙(`99-realsense-libusb.rules`)이 자동 스크립트로 제대로 복사되지 않아 카메라 USB 권한 오류(`RS2_USB_STATUS_ACCESS`) 발생
  → `config/99-realsense-libusb.rules`를 `/etc/udev/rules.d/`에 수동 복사, `udevadm control --reload-rules && udevadm trigger`, 카메라 재연결로 해결
- 이후 `/dev/bus/usb/...` 권한이 `root:plugdev`, `crw-rw-rw-`로 정상화

**카메라 인식 결과 (`rs-enumerate-devices`)**

| 카메라 | Serial | FW | IMU |
|---|---|---|---|
| D415 | 249322061567 | 5.15.1 | 없음 |
| D435i | 215322077500 | 5.17.0.10 | BMI085 |

## 3. colcon 빌드 환경 손상 진단 및 복구

**증상:** `colcon build` 시 `No module named colcon_cmake` 등 반복 에러, `colcon list`가 아무 패키지도 출력하지 않음, `--packages-select` 옵션 미인식

**근본 원인**
- `python3-colcon-*` 확장 패키지 중 16개가 dpkg 상으로는 설치됨(`ii`)으로 기록돼 있었으나 실제 `.py` 파일이 디스크에서 전부 소실된 상태 (dpkg 데이터베이스 ↔ 실제 파일시스템 불일치)
- 특히 `python3-colcon-recursive-crawl` 손상 → `colcon list`가 `src/` 하위 디렉토리를 재귀 탐색하지 못해 패키지를 전혀 못 찾음
- `python3-colcon-package-selection` 손상 → `--packages-select` 미인식

**복구한 패키지 (16개, `apt install --reinstall`)**
`python3-colcon-` + argcomplete, bash, cd, cmake, defaults, devtools, installed-package-information, library-path, metadata, override-check, package-selection, parallel-executor, pkg-config, recursive-crawl, test-result, zsh

**결과:** 전부 정상화 확인 (`colcon_cmake` / `colcon_package_selection` import 성공, `colcon list` 정상 출력, `--packages-select` 정상 인식)

## 4. realsense-ros (ROS 2 래퍼) 빌드

- 저장소: `IntelRealSense/realsense-ros`, `ros2-master` 브랜치
- 워크스페이스: `~/ros2_ws`
- `rosdep install` 후 `colcon build --packages-up-to realsense2_camera --symlink-install --parallel-workers 2`로 빌드 성공
- 빌드된 패키지: `realsense2_camera`, `realsense2_camera_msgs`
  (`realsense2_rgbd_plugin`은 저장소 측 `CATKIN_IGNORE`로 의도적 제외, `realsense2_description`은 별도 확인 안 함)
- `ros2 pkg list`에서 두 패키지 정상 인식 확인

## 5. 듀얼 카메라 동시 실행 테스트

| 카메라 | camera_name | camera_namespace | serial_no |
|---|---|---|---|
| D415 | D415 | cam_1 | `_249322061567` |
| D435i | D435i | cam_2 | `_215322077500` |

- 두 노드 동시 실행 성공. USB `control_transfer` 경고는 발생하나 비치명적
- 컬러 스트림 프레임레이트: D415 약 14.6 Hz, D435i 약 14.4 Hz
  (목표 15 Hz 대비 소폭 낮음 — 두 카메라가 같은 USB 허브(Bus 001)를 공유해서 대역폭을 나눠 쓰는 영향으로 추정. 실험 목적상 허용 범위)

## 6. Intrinsic 캘리브레이션 검증

공장 캘리브레이션 값 (`camera_info` 토픽, 640×480):

| | D415 | D435i |
|---|---|---|
| fx | 602.28 | 606.16 |
| fy | 600.21 | 605.51 |
| cx | 308.26 | 323.03 |
| cy | 249.53 | 249.53 |
| distortion | 0,0,0,0,0 | 0,0,0,0,0 (plumb_bob, 펌웨어 단에서 이미 보정됨) |

**체커보드(8×6, 24 mm) 재캘리브레이션 시도**
- 결과: fx/fy가 공장값의 약 2.2배, cx가 이미지 가장자리(40/640)로 발산한 비정상적인 해 → 보드 커버리지 불균형으로 solver 발산
- 판단: COMMIT 하지 않음, **공장 캘리브레이션 값을 그대로 사용하기로 결정**
  (D400 시리즈는 공장 출고 시 고정밀 캘리브레이션이 되어 있어 DIY 체커보드로 이를 능가하기 어려움)

## 7. Extrinsic 캘리브레이션 (D415 ↔ D435i)

**방법:** ArUco 마커(`DICT_6X6_250`, id=0, 10 cm) 공동 관측 기반

스크립트: `~/extrinsic_calib.py`
- `/cam_1/D415/color/image_raw` + `camera_info`
- `/cam_2/D435i/color/image_raw` + `camera_info`
- `message_filters.ApproximateTimeSynchronizer`로 동기화
- `cv2.solvePnP`로 각 카메라 기준 마커 포즈 추정
- T(D415→D435i) = T(marker→D415) × T(marker→D435i)⁻¹
- 30개 샘플 평균 (회전은 SVD 직교화로 평균)

**중간 트러블슈팅**
1. 1차: 두 카메라가 서로 다른 곳(사람, 체커보드, 서로를)을 보고 있어 공통 마커 없음 → 마커 재배치 필요
2. 2차: D435i가 마커를 극단적으로 비스듬한 각도 / 프레임 경계에서 봐서 코너 검출 실패 → 더 정면·중앙에 오도록 재배치
3. 3차: 양쪽 카메라 모두 id=0 정상 검출 확인 후 본 측정 진행

**최종 결과** (D415 ← D435i, `D415_color_optical_frame` 기준)

| | x | y | z |
|---|---|---|---|
| Translation (m) | 0.04253 | −0.36717 | 0.27369 |
| 표준편차 (m) | 0.01840 | 0.02660 | 0.04474 |

Quaternion (x, y, z, w): −0.01473, 0.50971, 0.86021, −0.00290

```
[[-0.99955 -0.01002 -0.02830  0.04253]
 [-0.02001 -0.48037  0.87684 -0.36717]
 [-0.02238  0.87701  0.47995  0.27369]
 [ 0.00000  0.00000  0.00000  1.00000]]
```

적용 명령 (아직 미실행, 필요 시 실행):
```bash
ros2 run tf2_ros static_transform_publisher \
  0.04253 -0.36717 0.27369 -0.01473 0.50971 0.86021 -0.00290 \
  D415_color_optical_frame D435i_color_optical_frame
```

참고: z축(깊이) 표준편차가 4.5 cm로 다소 큼 → 더 정밀한 값이 필요하면 마커를 더 가까운 거리 / 정면 각도에서 재측정 권장

## 8. 현재 상태 요약

**완료**
- [x] ROS 2 Humble 설치
- [x] librealsense2 SDK 빌드 및 카메라 인식
- [x] colcon 환경 손상 진단/복구
- [x] realsense-ros 빌드 (`realsense2_camera`, `realsense2_camera_msgs`)
- [x] 듀얼 카메라 동시 실행 및 토픽 발행 확인
- [x] Intrinsic 캘리브레이션 검증 (공장값 채택)
- [x] Extrinsic 캘리브레이션 (ArUco 기반, D415↔D435i 상대 변환 획득)

**미완료 / 다음 단계**
- [ ] `static_transform_publisher` 실제 적용 및 tf 트리 등록
- [ ] extrinsic 정밀도 개선 (필요 시 재측정)
- [ ] RGB-D 기반 사람 포즈/손 랜드마크 추출 파이프라인 구축 (MediaPipe 등, Feasible-MAP Retargeting 솔버 입력 형식에 맞춰 wrist SE(3) / swivel angle 산출)
- [ ] 로봇(AGILEX NERO) 연동 시 hand-eye 캘리브레이션
- [ ] 실시간 루프 latency 프로파일링
