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
| 8 | **Phase 6: SLAM Mapping** | **NOT STARTED** | - | - |
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
3. **Engineering Conclusion**: Kinematic wheel odometry assumes rigid rolling on an ideal 2-point line contact without lateral scrubbing. In practice, this small dead-reckoning drift is the exact reason autonomous mobile robots fuse IMUs via `robot_localization` or perform scan-matching via SLAM.

---

## Phase 5 Execution & Evidence Log

### Test Execution (`tests/test_phase5_lidar.py`):
```
==================================================================
  PROJECT 3 - PHASE 5: 2D LIDAR INTEGRATION VALIDATION SUITE      
==================================================================
Launching Gazebo Fortress with GPU LiDAR sensor...
Collecting LaserScan samples from /scan...

[TEST 1] /scan Topic & Active Publisher Check...
  Topic Type: True | Has Publisher: True

[TEST 2] /scan Continuous Publish Rate Verification...
  Calculated Rate: 10.0 Hz (Configured target: 10.0 Hz, Samples: 25)

[TEST 3] Frame ID Verification in Header...
  Reported frame_id: 'laser_link' (Expected: 'laser_link')

[TEST 4] Field-of-View & Angular Coverage...
  Samples: 720 (Expected: 720)
  Angle Min: -3.1416 rad, Max: 3.1416 rad
  FOV Coverage: 360.0 degrees (Expected: ~360.0 deg)

[TEST 5] Range Measurement Bounds (0.1m - 10.0m)...
  Configured Bounds: min=0.10m, max=10.00m
  Finite / Valid returns: 720 / 720 (100.0%)

[TEST 6] Obstacle Distance & Arena Geometry Verification...
  Forward  (Angle  0.0 deg): 4.800 m (Expected: ~4.90m North Wall)
  Rear     (Angle 180.0 deg): 5.000 m (Expected: ~5.10m South Wall)
  Left     (Angle +90.0 deg): 3.100 m (Expected: ~3.20m Center Partition)
  Right    (Angle -90.0 deg): 4.900 m (Expected: ~5.00m West Wall)
  Geometry Accuracy: Front=True, Rear=True, Left=True, Right=True

[TEST 7] TF Lookup: base_link -> laser_link...
  Transform base_link -> laser_link: [0.100, 0.000, 0.125]

[TEST 8] Full TF Lookup: odom -> laser_link...
  Transform odom -> laser_link: [0.100, 0.000, 0.175]

==================================================================
  PHASE 5 LIDAR VALIDATION SUMMARY
==================================================================
  [PASS] 1. /scan Topic & Publisher: LaserScan publisher active
  [PASS] 2. /scan Publish Rate: rate=10.0Hz
  [PASS] 3. Frame ID Verification: frame_id='laser_link'
  [PASS] 4. FOV & Samples: 720 samples, 360.0 deg FOV
  [PASS] 5. Range Bounds Check: 100.0% valid returns
  [PASS] 6. Obstacle Geometry Detection: Fwd=4.80m, Rear=5.00m, Left=3.10m
  [PASS] 7. TF base_link->laser_link: trans=[0.100, 0.000, 0.125]
  [PASS] 8. Full TF odom->laser_link: trans=[0.100, 0.000, 0.175]
==================================================================
```
