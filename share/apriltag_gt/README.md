# AprilTag GT rig assets (2026-11-09/10)

4 AprilTag 36h11 images for the body-tag GT rig in [11.09_exp/README.md](../11.09_exp/README.md) D3, plus the measured print sizes and the D415<->D435i extrinsic re-calibration script.

| File | Tag ID | Attach to | Print size (cm) |
|---|---|---|---|
| `T0_torso_id0.png` | 0 | torso | 9.0 |
| `T1_upperarm_id1.png` | 1 | upper arm | 6.0 |
| `T2_forearm_id2.png` | 2 | forearm | 6.0 |
| `T3_hand_id3.png` | 3 | dorsal hand | 4.0 |

Sizes recorded in `tag_measurements.yaml`. **These are the requested/nominal print sizes, not yet ruler-confirmed on the actual printout** - do that before trusting T1-T3 for anything quantitative (see the 2026-11-03 lesson below: printer scaling drift from nominal was part of the original error).

`extrinsic_calib_apriltag.py`: re-does the 2026-11-03 D415<->D435i extrinsic calibration (ArUco-based) using AprilTag instead. Usage:

```bash
python3 extrinsic_calib_apriltag.py --tag-id 0 --tag-length-m 0.09 --n-samples 60
```

Requires both `/cam_1/D415/...` and `/cam_2/D435i/...` realsense2_camera nodes running.

## Calibration results (T0, 2026-11-10)

| Attempt | Tag motion | z std | translation std magnitude |
|---|---|---|---|
| 2026-11-03 (ArUco, for reference) | mostly static | 44.7mm | - |
| 2026-11-10 #1 (AprilTag, 9cm) | hand-held, moved fast (~60 samples in ~22s) | 33.7mm | 41.5mm |
| 2026-11-10 #2 (AprilTag, 9cm) | hand-held, paused 1-2s per position | **14.2mm** | **18.1mm** |

Moving the tag slowly with pauses, instead of sweeping it quickly, roughly halved the uncertainty. Still not validated against an independent ruler/rig measurement (G0 in experiment_roadmap.md requires that), and a tripod/rigid mount instead of hand-held would likely reduce it further - not yet tried.

Latest result (attempt #2):
```
Translation (m): [-0.47115, 0.01617, 0.56663]
ros2 run tf2_ros static_transform_publisher -0.47115 0.01617 0.56663 0.00020 0.70309 -0.00907 0.71104 D415_color_optical_frame D435i_color_optical_frame
```

**Not yet done**: independent accuracy check (known-length object, not just repeatability).

## Tag -> joint center offsets + D4 prep (2026-11-10)

All 4 tags physically mounted on the body (torso/upper-arm/forearm/hand). Tag-to-joint-center
(S=shoulder, E=elbow, W=wrist) offsets measured by ruler, recorded in `tag_joint_offsets.yaml`
(along-bone distance + limb-circumference/2 depth estimate per tag, reference pose = arm relaxed
at side, see in-file comments for the exact measurement method).

`compute_joint_centers.py`: reads `tag_measurements.yaml` + `tag_joint_offsets.yaml`, detects all
4 tags live from the D435i (GT camera) feed, and estimates S/E/W positions in the camera frame
from each tag's pose + its measured offset. Publishes a debug overlay to `/gt_overlay/image` and
logs an elbow cross-check (`|T1-derived E - T2-derived E|`) whenever both the upper-arm and
forearm tags are visible, since they should independently agree on the same elbow position.

```bash
python3 compute_joint_centers.py --rate 5
```

Each tag's local +y axis (the "up" direction of the printed pattern) is assumed to point toward
the proximal joint (the one closer to the torso) when mounted. The forearm tag (T2) in this
session's physical mount turned out to be rotated 180 degrees relative to that assumption, which
showed up immediately as a large elbow cross-check error (~190-220mm) and a `W(T2)` point
appearing near the upper arm instead of the wrist in the overlay. Fixed with a per-tag
`mount_sign: -1` override in `tag_joint_offsets.yaml` (no need to re-mount the tag) rather than by
re-attaching it - after the fix, elbow cross-check dropped to ~110-130mm, and both `W(T2)`/`W(T3)`
land near the wrist as expected.

Remaining ~110-130mm elbow disagreement is attributed to ruler measurement precision and tags not
being perfectly parallel to the bone axis, not an orientation bug - acceptable for this stage but
worth tightening before trusting it as real quantitative GT (D4 gate).