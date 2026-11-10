# 2026-11-09 진행 로그 (작업 중 — 완료 보고서 아님)

> **상태:** IN PROGRESS. [사전 계획](../../README.md)의 D0~D3 일부만 진행됨. 최종 PASS/PARTIAL/BLOCKED 판정은 당일 작업 종료 후 이 섹션을 갱신한다.

## 지금까지 한 것

### 1. Rokoko Smartglove 연동 확인 (D2)
- Rokoko Studio는 Windows 전용이라 **별도 Windows PC**에서 실행, Jetson으로 네트워크 스트리밍하는 구조로 확정
- Custom Streaming(UDP, JSON, 기본 포트 14043)용 수신 스크립트 작성 (`~/glove_test/rokoko_udp_dump.py`, 로컬) — 아직 실제 라이브 스트림 수신 테스트는 안 함, 대신 **FBX export 2개(`share/test.fbx`, `share/test_2.fbx`)로 먼저 내용 확인**

### 2. FBX 내용 분석 결과 — 핵심 발견
`ufbx` 라이브러리로 두 테스트 클립(7초, 24.3초)의 본별 회전 변화량을 비교:

| 본 | test (7s) | test_2 (24.3s) |
|---|---|---|
| RightShoulder | 0.00000 | 0.00000 |
| RightArm | 1.326 | 1.416 |
| RightForeArm | 1.135 | 0.502 |
| RightHand | 0.384 | 0.775 |
| Left* (전부) | 0 | 0 |

**결론**: 어깨(RightShoulder)가 두 클립 모두 정확히 0 — 실측이 아니라 고정값. 팔/팔뚝 회전은 클립마다 다르게 변하는데, 이는 글러브의 손 방향 데이터만으로 Rokoko Studio가 **IK로 역산 추정**한 값일 가능성이 높음 (실측 팔꿈치 센서 없음). → **어깨/팔꿈치 위치 GT에는 FBX를 쓸 수 없고, AprilTag가 여전히 필요**하다고 판단 (사전 계획의 D2 경고와 일치하는 결과).

(참고: `pyassimp`는 이 FBX 파일 파싱 중 ctypes 레벨에서 크래시 — `ufbx`로 대체해서 분석함)

### 3. AprilTag GT rig 준비 착수 (D3)
- OpenCV `DICT_APRILTAG_36H11` 확인, Tag 4개 생성 완료 (`~/apriltag_gt/`):
  T0_torso(id0), T1_upperarm(id1), T2_forearm(id2), T3_hand(id3)
- 권장 인쇄 크기: T0 9cm / T1,T2 6cm / T3 4cm (검은 정사각형 변, 실측 필요 — 자로 잰 값을 코드에 넣어야 함)
- 아직 실제 부착/측정은 안 됨

### 4. D415↔D435i extrinsic 재측정 착수 (계획 외, 로드맵 G0 미해결 항목 연결)
- 2026-11-03 ArUco 기반 측정(z std 44.7mm)이 "불충분"으로 플래그됐던 걸 더 정밀한 AprilTag로 재시도하기로 함
- `~/apriltag_gt/extrinsic_calib_apriltag.py` 작성 (11/3 스크립트 개선판: AprilTag 사용, `--n-samples` 증가 가능, 실측 태그 크기를 CLI 인자로 파라미터화)
- D415/D435i ROS 노드 재기동 완료 (`cam_1/D415`, `cam_2/D435i` 토픽 정상)
- **아직 실행 안 함** — T0 Tag 실측 크기(cm) 대기 중

## 아직 안 된 것 / 막힌 것

- Rokoko Studio 라이브 스트림(UDP) 실제 수신 테스트 (FBX export로만 확인함)
- AprilTag 실측 크기 미확인 → extrinsic 재측정 미실행
- AprilTag 4개 몸에 실제 부착 안 됨 → D3/D4(동기화·GT 생성) 전체 미착수
- D1(D415 고정 설치), D5(markerless pose), D6(solver+Gazebo E2E) 미착수

## 다음 할 일
1. T0 Tag 실측 크기 받아서 D415↔D435i extrinsic 재측정 실행
2. 나머지 Tag 3개 실측 + 몸에 부착
3. Rokoko Studio 쪽에서 실제 UDP 스트리밍 켜서 라이브 수신 테스트
