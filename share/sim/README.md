# share/sim — NERO 기구학 (Roadmap Phase 1)

ROS 없이 Python(numpy)만으로 돈다. 결과 해석: [../docs/02_kinematics_validation.md](../docs/02_kinematics_validation.md)

| 파일 | 내용 | Roadmap |
|---|---|---|
| `kinematics.py` | URDF FK, SEW 각 ψ로 매개화한 SRS 해석적 IK (최대 8개 해), 관절 한계 아래 ψ feasible 구간 | 1.1–1.3 |
| `analysis_wrist_feasibility.py` | 손목 자세 실행 가능 비율과 원인 관절 분석 → `results/` | 1.4 |
| `tests/test_kinematics.py` | URDF 체인 FK 대조, IK 왕복·완전성, feasible 구간 안/밖 검사 | 1.1–1.3 |
| `assets/nero_description.urdf` | 공식 URDF 사본 (`agilexrobotics/agx_arm_urdf` @ `680fa92`, MIT, `assets/LICENSE.agx_arm_urdf`) | — |

```bash
pip install numpy pytest matplotlib
cd share/sim
python -m pytest -q tests
python analysis_wrist_feasibility.py --n 1000 --grid 180   # 16코어 기준 약 10분
```

## 규약

- 도구 프레임 = `link7`. 단위 m, rad.
- NERO의 "앞"은 **−x** (ready 자세에서 도구가 x = −0.30 m).
- ψ = stereographic SEW 각 (Elias & Wen, arXiv 2307.13122). 특이 방향 −z(베이스 아래), 기준 방향 −x.
  앞으로 뻗었을 때 ψ = 0 이 팔꿈치 아래, +90° 가 팔꿈치 −y 쪽. ready 자세는 ψ = ±180° (팔꿈치 위).
- 시각화(1.5)는 아직 없다.
