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
| 5 | **Phase 3: /cmd_vel Velocity Control** | **NOT STARTED** | - | - |
| 6 | **Phase 4: Odometry & TF2** | **NOT STARTED** | - | - |
| 7 | **Phase 5: LiDAR Sensor Integration** | **NOT STARTED** | - | - |
| 8 | **Phase 6: SLAM Mapping** | **NOT STARTED** | - | - |
| 9 | **Phase 7: Nav2 Autonomous Navigation** | **NOT STARTED** | - | - |

---

## Phase 1 Execution & Evidence Log

### Test Execution (`tests/test_phase1_physics.py`):
```
==================================================================
  PROJECT 3 - PHASE 1: GAZEBO FORTRESS PHYSICS VALIDATION SUITE   
==================================================================

[TEST 1] Verifying Package & Configuration Structure...
  [OK] CMakeLists.txt exists
  [OK] package.xml exists
  [OK] urdf/robot.urdf exists
  [OK] urdf/robot.xacro exists
  [OK] worlds/test_world.sdf exists
  [OK] launch/gazebo.launch.py exists
  [OK] config/rviz.rviz exists

[TEST 2] Processing robot.xacro -> URDF...
  [OK] Xacro parsed successfully. Links: 5, Joints: 4
  [OK] All expected links present: {'base_footprint', 'base_link', 'left_wheel_link', 'right_wheel_link', 'caster_wheel_link'}
  [OK] All expected joints present: {'base_footprint_joint', 'left_wheel_joint', 'right_wheel_joint', 'caster_joint'}

[TEST 3] Validating worlds/test_world.sdf with ign sdf...
  [OK] worlds/test_world.sdf is valid SDF.

[TEST 4] Rendering Physics Ground Contact in PyBullet...
  [OK] Robot settled at base z = 0.0010 m (expected ~0.05m chassis altitude)
  [OK] Total contact points with ground: 5
  [OK] Saved ground contact snapshot to: screenshots/phase1_physics_contact.png

[TEST 5] Launching Gazebo Fortress (Headless) Runtime Test...
  Waiting 6 seconds for Gazebo Fortress world initialization...
  [OK] diff_drive_robot successfully spawned in Gazebo Fortress.
  [OK] Gazebo Model Pose Output:
Model: [46]
  - Name: diff_drive_robot
  - Pose [ XYZ (m) ] [ RPY (rad) ]:
    [0.000000 0.000000 -0.000001]
    [-0.000000 -0.000000 -0.000000]
  [OK] /clock topic actively bridged from Gazebo Fortress to ROS 2.
  Shutting down Gazebo Fortress test instance...
  [OK] Gazebo Fortress test instance cleanly terminated.

==================================================================
  PHASE 1 STATUS: ALL TESTS PASSED (VERIFIED)
==================================================================
```

---

## Phase 2 Execution & Evidence Log

### Test Execution (`tests/test_phase2_ros2_control.py`):
```
==================================================================
  PROJECT 3 - PHASE 2: ros2_control DIFF DRIVE VALIDATION SUITE   
==================================================================

[TEST 1] Verifying ros2_control Configuration Files...
  [OK] config/ros2_controllers.yaml exists.

[TEST 2] Launching Gazebo Fortress + ros2_control (Headless)...
  Waiting 12 seconds for Gazebo Fortress and controller manager to start...

[TEST 3] Checking Controller Manager Active Controllers...
  Active controllers output:
joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster  active
diff_drive_controller   diff_drive_controller/DiffDriveController      active
  [OK] joint_state_broadcaster is ACTIVE.
  [OK] diff_drive_controller is ACTIVE.

  Initial Robot Pose: XYZ=[0.0, -0.0, -1e-06], RPY=[-0.0, -0.0, -0.0]
  [OK] /joint_states verified. Joints: ['left_wheel_joint', 'right_wheel_joint']

[TEST 4] Testing FORWARD Motion via /cmd_vel (0.5 m/s for 2.5s)...
  Pose after forward driving: XYZ=[1.365, -0.0, 0.001588], RPY=[0.0, -0.25268, 0.0]
  Displacement: delta_X = 1.3650 m, delta_Y = 0.0000 m
  [OK] Robot moved forward successfully (delta_X = 1.365 m > 0.40 m).

[TEST 5] Testing ROTATION via /cmd_vel (1.0 rad/s for 2.0s)...
  Pose after rotation: XYZ=[1.4682, -2.1e-05, -0.0], RPY=[0.0, -0.0, 2.47228]
  Yaw Change: delta_Yaw = 2.4723 rad (141.7 deg)
  [OK] Robot rotated counter-clockwise successfully (delta_Yaw = 2.472 rad > 0.5 rad).

[TEST 6] Testing STOP Command (0.0 m/s for 1.5s)...
  Drift after stop: 0.00000 m
  [OK] Robot halted cleanly (drift = 0.00000 m < 0.02 m).

  Terminating Gazebo Fortress test instance...
  [OK] Gazebo Fortress test instance cleanly terminated.

==================================================================
  PHASE 2 STATUS: ALL TESTS PASSED (VERIFIED)
==================================================================
```
