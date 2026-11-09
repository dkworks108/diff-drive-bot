# 03_diff_drive_robot: Autonomous 2-Wheel Differential Drive Mobile Robot

A complete simulated differential-drive mobile robot stack developed for **ROS 2 Humble** and **Gazebo Fortress (Ignition 6.18.0)**, featuring `ros2_control`, precision odometry, 360° 2D LiDAR integration, online asynchronous 2D SLAM (`slam_toolbox`), and full Nav2 autonomous navigation with AMCL localization, global/local costmaps, DWB trajectory controller, Navfn global planner, and behavior tree navigation.

---

## 1. Robot Kinematics & Specifications

| Component | Parameter | Value | Notes |
| :--- | :--- | :--- | :--- |
| **Chassis** | Dimensions ($L \times W \times H$) | $0.40 \times 0.25 \times 0.10\,\text{m}$ | Centered at $z=+0.05\,\text{m}$ relative to `base_link` |
| **Mass** | Total Weight | $6.30\,\text{kg}$ | Chassis: $5.0\,\text{kg}$, Wheels: $2 \times 0.5\,\text{kg}$, Caster: $0.2\,\text{kg}$, LiDAR: $0.1\,\text{kg}$ |
| **Wheel Radius ($r$)** | `left/right_wheel_link` | **$0.05\,\text{m}$** ($50\,\text{mm}$) | Cylinder length: $0.04\,\text{m}$ |
| **Wheel Separation ($d$)** | Track Width | **$0.30\,\text{m}$** ($300\,\text{mm}$) | Symmetrical offsets: $y_L = +0.15\,\text{m}, y_R = -0.15\,\text{m}$ |
| **Caster Ball** | Passive Omnidirectional Support | Radius $0.05\,\text{m}$ | Mounted at $x = +0.15\,\text{m}, z = 0.0\,\text{m}$, low-friction contact ($\mu = 0.001$) |
| **LiDAR Sensor** | `laser_link` | Mounted on chassis | Offset $xyz=(+0.10, 0.0, 0.125)\,\text{m}$ relative to `base_link` ($0.175\,\text{m}$ above ground) |

---

## 2. Differential Drive Kinematic Equations

For a differential drive robot with wheel radius $r$ and track width $d$:

### Forward Kinematics (Wheel Speeds to Body Velocities)
Given left wheel angular velocity $\omega_L$ and right wheel angular velocity $\omega_R$:
$$v_x = \frac{r}{2} (\omega_R + \omega_L)$$
$$\omega_z = \frac{r}{d} (\omega_R - \omega_L)$$

### Inverse Kinematics (Body Velocities to Wheel Speeds)
Given commanded linear velocity $v_x$ and angular velocity $\omega_z$:
$$\omega_L = \frac{v_x - \frac{d}{2} \omega_z}{r}$$
$$\omega_R = \frac{v_x + \frac{d}{2} \omega_z}{r}$$

---

## 3. Link Hierarchy & Complete TF2 Transformation Tree

```mermaid
flowchart TD
    map["map (Global Fixed SLAM Frame)"]
    odom["odom (Odometry World Frame)"]
    bf["base_footprint (Ground Projection Frame, z=0)"]
    bl["base_link (Chassis Frame, z=0.05m)"]
    lw["left_wheel_link"]
    rw["right_wheel_link"]
    cw["caster_wheel_link"]
    ll["laser_link (2D LiDAR Frame)"]

    map -->|/tf dynamic (slam_toolbox)| odom
    odom -->|/tf dynamic (diff_drive_controller)| bf
    bf -->|/tf_static (robot_state_publisher)| bl
    bl -->|/tf dynamic (robot_state_publisher)| lw
    bl -->|/tf dynamic (robot_state_publisher)| rw
    bl -->|/tf_static (robot_state_publisher)| cw
    bl -->|/tf_static (robot_state_publisher)| ll
```

---

## 4. 2D LiDAR Sensor Specifications

- **Sensor Type**: Gazebo Fortress `gpu_lidar` plugin
- **Mount Position**: $x = +0.10\,\text{m}, y = 0.0\,\text{m}, z = +0.125\,\text{m}$ relative to `base_link` ($0.175\,\text{m}$ above ground)
- **Field of View (FOV)**: $360.0^\circ$ ($[-\pi, +\pi]$ rad)
- **Beam Samples**: $720$ samples ($0.5^\circ$ angular resolution)
- **Update Frequency**: $10\,\text{Hz}$
- **Detection Range**: $0.10\,\text{m}$ to $10.0\,\text{m}$
- **ROS 2 Topic**: `/scan` (`sensor_msgs/msg/LaserScan`)
- **Frame ID**: `laser_link`
- **Bridge**: `ros_gz_bridge parameter_bridge` (`/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan`)

---

## 5. Phase 6: SLAM Toolbox 2D Mapping Architecture

- **SLAM Mode**: Online Asynchronous Mapping (`async_slam_toolbox_node`)
- **Solver Plugin**: `solver_plugins::CeresSolver` (Ceres Nonlinear Least Squares optimization)
- **Scan Matcher**: Real-time correlation scan-matching + scan barycenter
- **Map Resolution**: $0.05\,\text{m}$/pixel ($5\,\text{cm}$ grid)
- **Update Rate**: $2.0\,\text{s}$ periodic map publishing and graph optimization
- **Dynamic Transform**: Publishes `map -> odom` transform at $50\,\text{Hz}$ (`transform_publish_period: 0.02`)
- **Map Output**: `nav_msgs/msg/OccupancyGrid` on `/map` (trinary format: $-1$ unknown, $0$ free, $>0$ occupied)

---

---

## 6. Phase 7: Nav2 Autonomous Navigation Architecture

The navigation stack integrates the complete ROS 2 Navigation2 (Nav2) architecture with managed lifecycles:

- **Map Server (`nav2_map_server`)**: Loads and serves the static 2D occupancy grid map (`maps/test_world_map.yaml`) at $0.05\,\text{m/pixel}$.
- **AMCL Localization (`nav2_amcl`)**: Adaptive Monte Carlo Localization with differential motion model (`nav2_amcl::DifferentialMotionModel`) and likelihood field laser model, publishing the dynamic correction transform `map -> odom`.
- **Costmap 2D (`nav2_costmap_2d`)**:
  - **Global Costmap**: Static obstacle map layer + real-time laser obstacle clearing/marking layer + inflation layer ($r_{robot} = 0.22\,\text{m}$, $r_{inflation} = 0.45\,\text{m}$, scaling factor $3.0$).
  - **Local Costmap**: Dynamic rolling window ($3.0 \times 3.0\,\text{m}$) tracking local sensor obstacles and safety inflation buffers.
- **Global Path Planner (`nav2_planner`)**: Navfn Dijkstra / wavefront planner (`nav2_navfn_planner/NavfnPlanner`) computing optimal, collision-free global path waypoints in the `map` frame.
- **Local Controller Server (`nav2_controller`)**: DWB Local Planner (`dwb_core::DWBLocalPlanner`) with trajectory rollout evaluation (`RotateToGoal`, `Oscillation`, `BaseObstacle`, `GoalAlign`, `PathAlign`, `PathDist`, `GoalDist`), velocity limits ($v_{max} = 0.4\,\text{m/s}, \omega_{max} = 1.0\,\text{rad/s}$), and active stopping.
- **Behavior Server (`nav2_behaviors`)**: Recovery behaviors for obstacle deadlocks (`Spin`, `BackUp`, `Wait`).
- **BT Navigator (`nav2_bt_navigator`)**: Behavior tree execution engine dispatching navigation goals with automatic replanning and recovery strategies.
- **Lifecycle Manager (`nav2_lifecycle_manager`)**: Managed state machine transitions for all navigation servers (`map_server`, `amcl`, `controller_server`, `planner_server`, `behavior_server`, `bt_navigator`).

---

## 7. Build and Launch Instructions

### Setup Environment
```bash
source /opt/ros/humble/setup.bash
source /media/bumble/DATA_DISK/robotics/opt/ros_extra/setup.bash
source install/setup.bash
```

### Build Workspace
```bash
colcon build --symlink-install
```

### Launch Gazebo Simulation (Physics & Controllers only)
```bash
# Launch Gazebo GUI + robot controllers + LiDAR bridge
ros2 launch diff_drive_description gazebo.launch.py headless:=false rviz:=true
```

### Launch Complete SLAM Mapping Stack
```bash
# Launches Gazebo Fortress + ros2_control + SLAM Toolbox + RViz2 (fixed frame: map)
ros2 launch diff_drive_description slam.launch.py headless:=false rviz:=true
```

### Launch Complete Nav2 Autonomous Navigation Stack
```bash
# Launches Gazebo Fortress + ros2_control + Map Server + AMCL + Costmaps + Planner + Controller + BT Navigator + RViz2
ros2 launch diff_drive_description navigation.launch.py headless:=false rviz:=true
```

### Send Navigation Goal via CLI
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.8, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

---

## 8. Verification Test Suite

All phases are supported by automated validation suites:

```bash
# Phase 1: Gazebo Physics & Ground Contact
python3 tests/test_phase1_physics.py

# Phase 2: ros2_control Diff Drive Controllers
python3 tests/test_phase2_ros2_control.py

# Phase 3: Comprehensive 9-Maneuver Motion Suite
python3 tests/test_phase3_motion.py

# Phase 4: Odometry & TF2 Accuracy
python3 tests/test_phase4_odometry.py

# Phase 5: 2D LiDAR Integration & Scan Geometry
python3 tests/test_phase5_lidar.py

# Phase 6: SLAM Toolbox 2D Mapping & Map Persistence
python3 tests/test_phase6_slam.py

# Phase 7: Nav2 Autonomous Navigation Full Validation
python3 tests/test_phase7_navigation.py
```

---

## 9. Milestone Summary

| Milestone | Status | Key Evidence |
| :--- | :---: | :--- |
| **Phase 1: Gazebo Physics** | **VERIFIED** | Spawns in `test_world.sdf`, stable ground contact at $z \approx 0.0\,\text{m}$. |
| **Phase 2: ros2_control** | **VERIFIED** | `diff_drive_controller` & `joint_state_broadcaster` active at $50\,\text{Hz}$. |
| **Phase 3: Motion Validation** | **VERIFIED** | 9 maneuvers verified using simulation clock (`/clock`). |
| **Phase 4: Odometry & TF2** | **VERIFIED** | $1.5\,\text{mm}$ position error, dynamic TF `odom -> base_footprint`, static TF `base_footprint -> base_link`. |
| **Phase 5: 2D LiDAR** | **VERIFIED** | $360^\circ$ FOV, 720 samples, 10Hz, accurate detection of arena geometry. |
| **Phase 6: SLAM 2D Mapping** | **VERIFIED** | Ceres-backed async mapping, dynamic `map -> odom` TF, $+231.0\%$ map growth during motion, saved to `maps/test_world_map`. |
| **Phase 7: Nav2 Navigation** | **VERIFIED** | Map Server + AMCL + Costmaps + Navfn + DWB + BT Navigator. 11/11 tests pass: dynamic TF, path planning (80 waypoints), autonomous navigation to $(1.8, 0.0)$, active braking ($v=0.035\,\text{m/s} < 0.05\,\text{m/s}$), obstacle clearance ($1.12\,\text{m}$). |
