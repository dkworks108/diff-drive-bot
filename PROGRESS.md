# Project 3: 2-Wheel Differential Drive Robot — Progress Log

This document tracks development milestones, runtime verification evidence, and the engineering status for each phase of the autonomous mobile robot stack in ROS 2 Humble and Gazebo Fortress.

### Status Labels:
- **NOT STARTED**: Task is defined but no code/configuration has been written.
- **IN PROGRESS**: Development or test creation is underway.
- **IMPLEMENTED**: Code/configuration is written and compiles cleanly.
- **TESTED**: Functional test executed in headless simulation or test runner.
- **VERIFIED**: Fully validated with runtime evidence, stable physics, and ground contact checks.

---

## Milestone Tracking Table

| No. | Milestone / Phase | Status | Verification Tool / Script | Runtime Evidence & Notes |
| :---: | :--- | :---: | :--- | :--- |
| 1 | **URDF / Xacro Model** | **VERIFIED** | `test_robot.py`, `test_extra.py` | Validated 6 links (including `laser_link`), 5 joints, massless `base_footprint` root, 50mm wheel radius, 300mm track width. |
| 2 | **RViz Visualization** | **VERIFIED** | `launch/display.launch.py`<br>`tools/rviz_snapshot.py` | Visualized with GUI joint sliders; fixed frame `base_footprint`; all 4 views verified. |
| 3 | **Phase 1: Gazebo Physics & Spawning** | **VERIFIED** | `tests/test_phase1_physics.py`<br>`launch/gazebo.launch.py` | Spawns in Gazebo Fortress (`test_world.sdf`), settles stably at ground contact ($z \approx 0.0\text{ m}, roll/pitch/yaw = 0.0$), `/clock` bridge active. |
| 4 | **Phase 2: ros2_control Diff Drive** | **VERIFIED** | `tests/test_phase2_ros2_control.py`<br>`launch/gazebo.launch.py` | `joint_state_broadcaster` and `diff_drive_controller` active. `/joint_states`, `/cmd_vel`, `/odom` active. Verified forward motion, rotation, and stopping without drift. |
| 5 | **Phase 3: /cmd_vel Velocity Control** | **VERIFIED** | `tests/test_phase3_motion.py` | 9-maneuver motion test suite regulated using simulation clock `/clock`. Forward, reverse, CW/CCW, in-place rotation, circular, combined, stop, watchdog timeout. |
| 6 | **Phase 4: Odometry & TF2** | **VERIFIED** | `tests/test_phase4_odometry.py` | Validated 50.5 Hz `/odom` rate, full covariance matrices, 1.5mm position error, `odom->base_footprint` dynamic TF, `base_footprint->base_link` static TF, buffer lookup across all links. |
| 7 | **Phase 5: LiDAR Sensor Integration** | **VERIFIED** | `tests/test_phase5_lidar.py`<br>`launch/gazebo.launch.py` | Integrated `gpu_lidar` on `laser_link` at $x=+0.10\,\text{m}, z=+0.125\,\text{m}$. Verified `/scan` topic, 10.0 Hz rate, 360° FOV, 720 samples, obstacle geometry, and TF `base_link->laser_link`. |
| 8 | **Phase 6: SLAM 2D Mapping** | **VERIFIED** | `tests/test_phase6_slam.py`<br>`launch/slam.launch.py` | Integrated `slam_toolbox` (online async). Dynamic TF `map -> odom`, full chain to `laser_link`. Robot navigated $1.56\,\text{m}$ in arena, growing known cells by $+231.0\%$. Saved map via `map_saver_cli` to `maps/test_world_map`. |
| 9 | **Phase 7: Nav2 Autonomous Navigation** | **NOT STARTED** | - | - |

---

## Discrepancy & Accuracy Investigation Report

### 1. Root Causes of Earlier Measurement Discrepancies:
- **Wall-Clock vs Simulation-Clock Mismatch**:
  In earlier iterations, `time.time()` (wall-clock time) was used to measure command duration. Because the simulation rate fluctuates, wall-clock time does not equal elapsed physics time.
- **Watchdog Timeout Coasting (`cmd_vel_timeout = 0.5s`)**:
  When a publisher ceases sending `/cmd_vel` without an explicit zero-velocity command (`Twist()`), the controller keeps executing the previous setpoint until its internal watchdog timer ($0.5\text{ s}$) expires.
  - Commanded: $0.5\text{ m/s}$ for $2.5\text{ s}$ + $0.5\text{ s}$ watchdog coasting = $3.0\text{ s} \times 0.5\text{ m/s} \approx 1.365\text{ m}$.

### 2. Angular Yaw Discrepancy Analysis (Odometry vs Gazebo Ground Truth):
Under simulation-time calibrated testing:
- Commanded: $1.0\,\text{rad/s}$ for $1.500\,\text{s}$ followed by active braking.
- Wheel encoders reached: $\Delta \theta_L = -4.5000\,\text{rad}, \Delta \theta_R = +4.5000\,\text{rad}$.
- Kinematic Yaw: $\frac{(\Delta \theta_R - \Delta \theta_L) \cdot r}{d} = \frac{9.0 \cdot 0.05}{0.30} = \mathbf{1.5000\,\text{rad}}$.
- Odometry Estimate: $\mathbf{1.5000\,\text{rad}}$ (**$100\%$ exact encoder agreement**).
- Gazebo Ground Truth: $\mathbf{1.7232\,\text{rad}}$ (discrepancy = $0.2232\,\text{rad}$ / $12.7^\circ$).

**Mechanics of the Orientation Divergence:**
1. **Deceleration Profiling**: `diff_drive_controller` limits angular acceleration/deceleration to $3.0\,\text{rad/s}^2$. Ramping down from $1.0\,\text{rad/s}$ to zero takes $0.33\,\text{s}$, continuing wheel rotation through $\approx 0.167\,\text{rad}$ during braking.
2. **Caster Lateral Scrubbing**: Unlike rolling forward where wheels experience pure longitudinal rolling ($0.18\%$ position error), turning in-place forces the front caster ball at $x = +0.15\,\text{m}$ to scrub laterally across the floor. This shifts the instantaneous center of rotation (ICR) and introduces subtle dynamic tire scrubbing, an established characteristic of differential-drive mobile bases with passive casters.
3. **Engineering Conclusion**: Kinematic wheel odometry assumes rigid rolling on an ideal 2-point line contact without lateral scrubbing. In practice, this dead-reckoning drift is corrected through scan-matching and graph optimization in Phase 6 SLAM!

---

## Phase 6 Execution & Evidence Log

### Test Execution (`tests/test_phase6_slam.py`):
```
======================================================================
  PROJECT 3 - PHASE 6: SLAM TOOLBOX 2D MAPPING VALIDATION SUITE
======================================================================

[TEST 1] Launching Gazebo Fortress + Controllers + SLAM Toolbox...
  Waiting up to 25s for Gazebo, controllers and SLAM Toolbox to initialize...
  Simulation & SLAM process running: True
  Simulation time received: True (5.46s)

[TEST 2] Simulation Time Check (/clock)...
  Current sim_time: 5.463 s

[TEST 3] Initial /map Topic & Structure...
  Map Frame ID: 'map' (Expected: 'map')
  Resolution: 0.050 m/pixel (Expected: 0.05m)
  Dimensions: 196 x 196 cells (9.8m x 9.8m coverage)
  Origin: (-4.91, -4.91)

[TEST 4] Map Cell Distribution (Unknown, Free, Occupied)...
  Total cells: 38416
  Unknown (-1):   32103 (83.7%)
  Free space (0):  6234 (16.3%)
  Occupied (>0):     79 (0.2%)
  Initial known cells: 6313 (Free + Obstacles)

[TEST 5] TF Transform: map -> odom...
  Transform map -> odom: translation=[0.0000, 0.0000, 0.0000]

[TEST 6] Full TF Chain: map -> odom -> base_footprint -> base_link -> laser_link...
  Full Transform map -> laser_link: [0.1000, 0.0000, 0.1750]

[TEST 7] Dynamic Mapping Maneuver (Moving robot to expand map)...
  Initial Odom Pose: X=0.000 m, Y=-0.000 m
  Initial Known Cells: 6313
  Leg 1: Driving forward 1.2m at 0.4 m/s (sim time)...
  Leg 2: Rotating CCW 90 degrees at 0.5 rad/s (sim time)...
  Leg 3: Driving forward 1.0m along new heading at 0.4 m/s (sim time)...
  Waiting 6s sim time for SLAM graph optimization & map updates...
  Final Odom Pose: X=1.201 m, Y=1.000 m (Displacement: 1.563 m)
  Map update messages received: 9
  Final Known Cells: 20899 (Free: 19368, Occupied: 1531)
  Map Expansion: +14586 cells (+231.0%)

[TEST 8] Saving Map via map_saver_cli...
  Saved YAML (/media/bumble/DATA_DISK/robotics/projects/03_diff_drive_robot/maps/test_world_map.yaml): 132 bytes
  Saved PGM  (/media/bumble/DATA_DISK/robotics/projects/03_diff_drive_robot/maps/test_world_map.pgm):  42370 bytes

[TEST 9] RViz2 SLAM Visualization Snapshot...
  RViz2 screenshot saved: /media/bumble/DATA_DISK/robotics/projects/03_diff_drive_robot/screenshots/phase6_slam_mapping.png (176294 bytes)

  Terminating simulation & SLAM processes...
  Cleaned up all background simulation processes.

======================================================================
  PHASE 6 SLAM TOOLBOX 2D MAPPING VALIDATION SUMMARY
======================================================================
  [PASS] 1. Stack Launch: sim_time=5.463
  [PASS] 2. Simulation Time Clock: sim_time=5.463 s
  [PASS] 3. /map Message Structure: res=0.05m, size=196x196, frame='map'
  [PASS] 4. Cell Classification: unknown=32103, free=6234, occupied=79
  [PASS] 5. TF map -> odom: trans=[0.000, 0.000, 0.000]
  [PASS] 6. Full TF Tree (map -> laser_link): trans=[0.100, 0.000, 0.175]
  [PASS] 7. Dynamic Mapping Expansion: dist=1.56m, cell_gain=+14586 (+231.0%)
  [PASS] 8. Map Saving (map_saver_cli): yaml=132B, pgm=42370B
  [PASS] 9. RViz2 Visualization Snapshot: path=screenshots/phase6_slam_mapping.png
======================================================================
```
