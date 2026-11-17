# AprilTag GT rig assets (2026-11-09/10)

4 AprilTag 36h11 images for the body-tag GT rig in [11.09_exp/README.md](../11.09_exp/README.md) D3, plus the measured print sizes and the D415<->D435i extrinsic re-calibration script.

| File | Tag ID | Attach to | Print size (cm) |
|---|---|---|---|
| `T0_torso_id0.png` | 0 | torso | 9.0 |
| `T1_upperarm_id1.png` | 1 | upper arm | 6.0 |
| `T2_forearm_id2.png` | 2 | forearm | 6.0 |
| `T3_hand_id3.png` | 3 | dorsal hand | 4.0 |

Sizes recorded in `tag_measurements.yaml`, ruler-confirmed on the reprinted tags (2026-11-10). Re-measure after any reprint: the first print came out at ~79% scale (7.1/4.8/4.8/3.2 cm).

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

**Not yet done** (as of 2026-11-10): independent accuracy check (known-length object, not just
repeatability).

## Re-calibration + independent accuracy check (2026-11-17)

Re-ran the extrinsic calibration and, this time, also validated it against an independent
ground-truth distance (a ruler-measured move), not just repeatability. `extrinsic_calib_apriltag.py`
now also saves its result as YAML (`--out`, defaults to
`calib_results/extrinsic_D415_D435i_<date>.yaml`) instead of only printing it.

Live ROS2 preview while positioning the rig: `python3 live_dual_view.py` (two OpenCV windows, no
web artifact, AprilTag overlay). For the manual 30cm-move accuracy check below:
`python3 manual_capture_gui.py` (two camera windows + a Control window with CAPTURE A / CAPTURE B /
SAVE BEFORE / SAVE AFTER / RESET buttons -- every capture is a deliberate click, nothing automatic).

### Extrinsic results (D415 <- D435i translation, meters)

| Run | Tag | Motion | n | std magnitude | translation (m) |
|---|---|---|---|---|---|
| `extrinsic_D415_D435i_2026-11-17.yaml` | T0 (9cm) | static | 60 | 26.4mm | [-0.298, -0.010, 0.160] |
| `extrinsic_D415_D435i_2026-11-17_v2.yaml` | T0 (9cm) | slow, paused | 60 | **17.5mm** | [-0.291, -0.011, 0.155] |
| `extrinsic_D415_D435i_2026-11-17_tag1.yaml` | T1 (6cm) | slow, paused | 60 | 14.5mm | [-0.282, 0.025, 0.183] |

**Recommended result: `extrinsic_D415_D435i_2026-11-17_v2.yaml`** (T0, moving the tag slowly
with pauses -- confirms the 2026-11-10 finding that this halves the std vs. a static tag).

**Important caveat, found by cross-checking T0 against T1**: the two tags' independently-computed
extrinsics disagree by **46.3mm** (axis-wise: x 9.5mm, y 35.5mm, z 28.2mm) -- 2-3x each run's own
repeatability std. Since the two cameras are rigidly fixed, a correct extrinsic should not depend
on which tag was used to measure it, so this 46mm gap is a real systematic error, not just noise.
T0's printed size was re-measured with a ruler and confirmed exactly 9.0cm (not the cause).
Suspected remaining causes, not yet isolated: T1 is mounted on a curved surface (upper arm) vs T0's
flatter mount (torso); the two tags sit at different distances/angles from the cameras, which
changes how much a fixed angular pose error turns into a translation error. **Treat the ~46mm
cross-tag gap as the current accuracy floor for this extrinsic, not the 14-17mm repeatability
std.**

### Metric-scale accuracy check: move T0 by a ruler-measured 30cm, independently per camera

This checks each camera's own depth/scale accuracy (not the inter-camera extrinsic): capture T0's
PnP position at a start point, move it exactly 30cm (ruler-measured), capture again, and compare
each camera's *own* measured displacement to the known 30cm -- no assumption about the other
camera or the rig geometry is involved.

| Trial | Cam A (D415) error | Cam B (D435i) error |
|---|---|---|
| close distance (~0.85-0.97m) | -17.5mm (-5.8%) | -8.4mm (-2.8%) |
| close distance, repeat | -10.2mm (-3.4%) | -6.1mm (-2.0%) |
| far distance (~1.6-1.8m) | +4.2mm (+1.4%) | +9.9mm (+3.3%) |

Both cameras are accurate to within **1-6%** of the ruler-measured 30cm, at both close and far
range -- moving the tag further away did *not* make it worse here, which argues against distance
being the main driver of the 46mm cross-tag extrinsic gap above (a hypothesis raised and tested,
not confirmed).

**Pitfall hit while running this** (left in for future sessions): a SAVE AFTER compared against a
*stale* SAVE BEFORE from an earlier, unrelated tag position produced a nonsense "+47-63% error" --
not a real accuracy problem, just the wrong baseline. `manual_capture_gui.py` does not warn about
this; always RESET and do a fresh BEFORE immediately before moving the tag.

**Still not done**: isolating the actual cause of the 46mm T0-vs-T1 cross-tag gap (candidates:
tag-mount curvature, tag-to-camera angle/distance at capture time, residual lens distortion not
captured by the factory `D=0` camera_info).

## Tag -> joint center offsets + D4 prep (2026-11-10)

All 4 tags physically mounted on the body (torso/upper-arm/forearm/hand). Tag-to-joint-center
(S=shoulder, E=elbow, W=wrist) offsets measured by ruler, recorded in `tag_joint_offsets.yaml`
(along-bone distance + limb circumference/2 per tag, which the script divides by pi to get the
surface-to-center depth (radius), reference pose = arm relaxed
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

Remaining ~110-130mm elbow disagreement was measured with two bugs still in place, fixed on
2026-11-10 (re-measure the cross-check before drawing conclusions):

- `depth_cm` (circumference/2) was used directly as the surface-to-center depth. The radius is
  circumference/2pi, so the joint points were pushed ~3x too deep (upper arm 16 cm instead of
  ~5.1 cm, forearm 14 cm instead of ~4.5 cm). Since T1 and T2 face different directions, that
  extra ~10 cm of depth lands directly in the elbow disagreement.
- `tag_measurements.yaml` still had the first print's sizes (7.1/4.8/4.8/3.2 cm) while the
  mounted tags are the 9/6/6/4 cm reprint, so every tag distance came out ~20% short.

T0 caveat: the torso is not a circle and the shoulder joint is not at the torso's center depth,
so the chest radius (~16 cm) only roughly approximates the chest-surface-to-shoulder depth.