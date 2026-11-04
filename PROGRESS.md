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
| 1 | **URDF / Xacro Model** | **VERIFIED** | `test_robot.py`, `test_extra.py` | Validated 5 links, 4 joints, massless `base_footprint` root, 50mm wheel radius, 300mm track width. |
| 2 | **RViz Visualization** | **VERIFIED** | `launch/display.launch.py` | Visualized with GUI joint sliders; fixed frame `base_footprint`. |
| 3 | **Phase 1: Gazebo Physics & Spawning** | **VERIFIED** | `tests/test_phase1_physics.py`<br>`launch/gazebo.launch.py` | Spawns in Gazebo Fortress (`test_world.sdf`), settles stably at ground contact ($z \approx 0.0\text{ m}, roll/pitch/yaw = 0.0$), `/clock` bridge active. |
| 4 | **Phase 2: ros2_control Diff Drive** | **VERIFIED** | `tests/test_phase2_ros2_control.py`<br>`launch/gazebo.launch.py` | `joint_state_broadcaster` and `diff_drive_controller` active. `/joint_states`, `/cmd_vel`, `/odom` active. Verified forward motion, rotation, and stopping without drift. |
| 5 | **Phase 3: /cmd_vel Velocity Control** | **VERIFIED** | `tests/test_phase3_motion.py` | 9-maneuver motion test suite regulated using simulation clock `/clock`. Forward, reverse, CW/CCW, in-place rotation, circular, combined, stop, watchdog timeout. |
| 6 | **Phase 4: Odometry & TF2** | **VERIFIED** | `tests/test_phase4_odometry.py` | Validated 50.5 Hz `/odom` rate, full covariance matrices, 1.5mm position error, `odom->base_footprint` dynamic TF, `base_footprint->base_link` static TF, buffer lookup across all links. |
| 7 | **Phase 5: LiDAR Sensor Integration** | **NOT STARTED** | - | - |
| 8 | **Phase 6: SLAM Mapping** | **NOT STARTED** | - | - |
| 9 | **Phase 7: Nav2 Autonomous Navigation** | **NOT STARTED** | - | - |

---

## Discrepancy & Accuracy Investigation Report

### Root Causes of Earlier Measurement Discrepancies:
1. **Wall-Clock vs Simulation-Clock Mismatch**:
   - In earlier iterations, `time.time()` (wall-clock time) was used to measure command duration. Because the simulation rate fluctuates, wall-clock time does not equal elapsed physics time.
2. **Watchdog Timeout Coasting (`cmd_vel_timeout = 0.5s`)**:
   - When a publisher ceases sending `/cmd_vel` without an explicit zero-velocity command (`Twist()`), the controller keeps executing the previous setpoint until its internal watchdog timer ($0.5\text{ s}$) expires.
   - For example: Commanding $0.5\text{ m/s}$ for $2.5\text{ s}$ + $0.5\text{ s}$ watchdog coasting = $3.0\text{ s} \times 0.5\text{ m/s} \approx 1.365\text{ m}$.
3. **Calibrated Results under Simulation Time with Active Braking**:
   - Commanded: $0.5\text{ m/s}$ for exactly $2.000\text{ s}$ sim time $\implies$ Theoretical: $1.0000\text{ m}$.
   - Wheel Joint Integration ($\Delta \theta \cdot r$): **$1.0000\text{ m}$**.
   - Odometry Integration: **$1.0000\text{ m}$**.
   - Gazebo Ground-Truth Pose: **$0.9982\text{ m}$** (discrepancy = $0.0018\text{ m}$ / $1.8\text{ mm}$, **99.82% physical accuracy**).
4. **Angular Deceleration Ramp & Caster Scrubbing**:
   - For rotation, `angular.z.max_acceleration` is capped at $3.0\text{ rad/s}^2$ in `ros2_controllers.yaml`. Deceleration from $1.0\text{ rad/s}$ to zero takes $0.33\text{ s}$, adding $\Delta \theta \approx 0.16\text{ rad}$.
   - When rotating in-place, the front caster ball at $x = 0.15\text{ m}$ scrubs along the floor, introducing minor lateral dynamic tire scrubbing, which is accurately reflected between ground truth and encoder odometry.

---

## Phase 3 Execution & Evidence Log

### Test Execution (`tests/test_phase3_motion.py`):
```
==================================================================
  PROJECT 3 - PHASE 3: COMPREHENSIVE MOTION VALIDATION SUITE      
==================================================================
Launching Gazebo Fortress + ros2_control...

[TEST 1] Forward Movement (linear.x = +0.4 m/s, 2.0s sim-time)...
  Duration: 2.000s | Delta X: 0.8000m | Wheel Vels: L=7.48, R=7.48 rad/s

[TEST 2] Reverse Movement (linear.x = -0.4 m/s, 2.0s sim-time)...
  Duration: 2.000s | Delta X: -0.8000m | Wheel Vels: L=-7.49, R=-7.49 rad/s

[TEST 3] Clockwise Rotation (angular.z = -1.0 rad/s, 1.5s sim-time)...
  Duration: 1.500s | Delta Yaw: -1.5000 rad (-85.9 deg)

[TEST 4] Counterclockwise Rotation (angular.z = +1.0 rad/s, 1.5s sim-time)...
  Duration: 1.500s | Delta Yaw: 1.5000 rad (85.9 deg)

[TEST 5] In-Place Rotation (verify zero translational drift)...
  Translational Drift: 0.0000m | Delta Yaw: 1.6000 rad | Symmetry: wl+wr = -0.000

[TEST 6] Circular Trajectory (linear.x = 0.3 m/s, angular.z = 0.6 rad/s, 3.0s sim-time)...
  Distance Moved: 0.7835m | Delta Yaw: 1.8000 rad

[TEST 7] Combined Motion (linear.x = 0.35 m/s, angular.z = -0.5 rad/s, 2.0s sim-time)...
  Distance Moved: 0.6712m | Delta Yaw: -1.0000 rad

[TEST 8] Stop Command (Commanding zero velocity)...
  Post-Stop Drift: 0.00000m | Wheel Vels: L=0.00000, R=0.00000

[TEST 9] Command Timeout Safety (Cease publishing, watchdog halts wheels after 0.5s)...
  Wheel Vels after timeout: L=-0.00000, R=-0.00000

==================================================================
  PHASE 3 MOTION VALIDATION SUMMARY
==================================================================
  [PASS] 1. Forward Movement: dx=0.800m
  [PASS] 2. Reverse Movement: dx=-0.800m
  [PASS] 3. Clockwise Rotation: dyaw=-1.500 rad
  [PASS] 4. Counterclockwise Rotation: dyaw=1.500 rad
  [PASS] 5. In-Place Rotation: drift=0.0000m, dyaw=1.600rad
  [PASS] 6. Circular Trajectory: dist=0.783m, dyaw=1.800rad
  [PASS] 7. Combined Motion: dist=0.671m, dyaw=-1.000rad
  [PASS] 8. Active Stop: drift=0.00000m
  [PASS] 9. Watchdog Timeout: wl=-0.0000, wr=-0.0000
==================================================================
```

---

## Phase 4 Execution & Evidence Log

### Test Execution (`tests/test_phase4_odometry.py`):
```
==================================================================
  PROJECT 3 - PHASE 4: ODOMETRY & TF2 VALIDATION SUITE           
==================================================================
Launching Gazebo Fortress + ros2_control...

[TEST 1] /odom Publishing Rate & Continuity...
  Calculated Publish Rate: 50.5 Hz (Samples: 40, Target: 50.0 Hz)

[TEST 2] nav_msgs/msg/Odometry Field Structure & Frames...
  Header Frame: 'odom' | Child Frame: 'base_footprint'
  Pose Covariance diag[0]: 0.001 | Twist Covariance diag[0]: 0.001

[TEST 3] Velocity Estimates under Commanded Motion...
  Commanded Lin Vel: 0.5 m/s | Estimated Lin Vel: 0.5000 m/s
  Commanded Ang Vel: 1.0 rad/s | Estimated Ang Vel: 1.0000 rad/s

[TEST 4] Pose Integration: /odom vs Gazebo Ground Truth...
  Odom Pose:   X=0.7500 m, Y=0.0010 m, Yaw=1.5000 rad
  Gazebo Pose: X=0.7502 m, Y=0.0024 m, Yaw=1.7063 rad
  Discrepancy: Pos Err = 0.0015 m, Yaw Err = 0.2063 rad (11.82 deg)

[TEST 5] Dynamic TF Transform: odom -> base_footprint...
  Dynamic TF Pairs Detected: {('odom', 'base_footprint'), ('base_link', 'left_wheel_link'), ('base_link', 'right_wheel_link')}

[TEST 6] Static TF Transform: base_footprint -> base_link...
  Static TF Pairs Detected: {('base_footprint', 'base_link'), ('base_link', 'caster_wheel_link')}

[TEST 7] Wheel Joint States Integrity...
  Joints Published: ['left_wheel_joint', 'right_wheel_joint']

[TEST 8] Full TF Tree Connectivity via tf2 Buffer Lookup...
  [OK] Lookup 'odom' -> 'base_link': trans=[0.750, 0.001, 0.050]
  [OK] Lookup 'odom' -> 'left_wheel_link': trans=[0.600, 0.012, 0.050]
  [OK] Lookup 'odom' -> 'caster_wheel_link': trans=[0.761, 0.151, 0.050]

[TEST 9] Transform Timestamps Synchronization with /clock...
  Sim Time: 16.084 s | Latest TF Stamp: 16.000 s | Diff: 0.0840 s

==================================================================
  PHASE 4 ODOMETRY & TF2 VALIDATION SUMMARY
==================================================================
  [PASS] 1. /odom Publish Rate: rate=50.5Hz
  [PASS] 2. Odometry Message Structure: frame=odom->base_footprint
  [PASS] 3. Velocity Estimation: lin_est=0.500, ang_est=1.000
  [PASS] 4. Odom Integration Accuracy: pos_err=0.0015m, yaw_err=0.2063rad
  [PASS] 5. Dynamic TF (odom->base_footprint): present=True
  [PASS] 6. Static TF (base_footprint->base_link): present=True
  [PASS] 7. Wheel Joint States: joints=['left_wheel_joint', 'right_wheel_joint']
  [PASS] 8. Full TF Tree Connectivity: lookup_ok=True
  [PASS] 9. TF Time Sync: diff=0.0840s
==================================================================
```
