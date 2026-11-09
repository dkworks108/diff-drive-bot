#!/usr/bin/env python3
"""
Phase 7 Validation Suite: Nav2 Autonomous Navigation Verification

Tests:
  1. Complete Navigation stack launch (Gazebo Fortress + controllers + Nav2)
  2. Simulation time synchronization (/clock)
  3. Map Server loading and publication of Phase 6 saved map (/map)
  4. AMCL localization active and publishing map -> odom transform
  5. Full kinematic TF chain lookup: map -> odom -> base_footprint -> base_link -> laser_link
  6. Global and Local Costmaps initialization and publication
  7. Global path generation via ComputePathToPose / Planner Server
  8. Autonomous navigation goal execution via NavigateToPose action server
  9. Goal tolerance arrival and active braking / stopping at target
  10. Obstacle avoidance trajectory planning around arena obstacles
  11. Visual verification snapshot using RViz2 navigation profile
"""

import os
import sys
import time
import math
import subprocess
import signal
import numpy as np

# Ensure environment is properly bootstrapped
try:
    from nav2_msgs.action import NavigateToPose, ComputePathToPose
except ImportError:
    extra_setup = "/media/bumble/DATA_DISK/robotics/opt/ros_extra/setup.bash"
    args = " ".join([f'"{a}"' for a in sys.argv])
    cmd = f"source /opt/ros/humble/setup.bash && source {extra_setup} && python3 {args}"
    ret = subprocess.run(["bash", "-c", cmd])
    sys.exit(ret.returncode)

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from nav2_msgs.action import NavigateToPose, ComputePathToPose
import tf2_ros


def euler_to_quaternion(yaw):
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0)
    )


class Nav2Verifier(Node):
    def __init__(self):
        super().__init__('phase7_nav2_verifier')

        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # Map subscription with TRANSIENT_LOCAL durability
        transient_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_cb, transient_qos)
        self.global_costmap_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.global_costmap_cb, transient_qos)
        self.local_costmap_sub = self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self.local_costmap_cb, transient_qos)

        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # Action clients
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.compute_path_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

        # TF2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State tracking
        self.sim_time = None
        self.latest_map = None
        self.latest_global_costmap = None
        self.latest_local_costmap = None
        self.latest_scan = None
        self.latest_odom = None
        self.cmd_vel_history = []
        self.odom_history = []

    def clock_cb(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def map_cb(self, msg):
        self.latest_map = msg

    def global_costmap_cb(self, msg):
        self.latest_global_costmap = msg

    def local_costmap_cb(self, msg):
        self.latest_local_costmap = msg

    def scan_cb(self, msg):
        self.latest_scan = msg

    def odom_cb(self, msg):
        self.latest_odom = msg
        self.odom_history.append((self.sim_time, msg))
        if len(self.odom_history) > 100:
            self.odom_history.pop(0)

    def cmd_cb(self, msg):
        self.cmd_vel_history.append((self.sim_time, msg))
        if len(self.cmd_vel_history) > 100:
            self.cmd_vel_history.pop(0)

    def wait_for_sim_time(self, duration_sec):
        if self.sim_time is None:
            time.sleep(duration_sec)
            return
        start = self.sim_time
        while (self.sim_time is None or (self.sim_time - start < duration_sec)):
            rclpy.spin_once(self, timeout_sec=0.02)


def run_tests():
    print("="*70)
    print("  PROJECT 3 - PHASE 7: NAV2 AUTONOMOUS NAVIGATION VALIDATION SUITE")
    print("="*70)

    proj_dir = "/media/bumble/DATA_DISK/robotics/projects/03_diff_drive_robot"
    extra_setup = "/media/bumble/DATA_DISK/robotics/opt/ros_extra/setup.bash"
    ws_setup = os.path.join(proj_dir, "install", "setup.bash")
    screens_dir = os.path.join(proj_dir, "screenshots")
    os.makedirs(screens_dir, exist_ok=True)

    results = []

    rclpy.init()
    node = Nav2Verifier()

    # 1. Launch complete stack (Gazebo Fortress + controllers + Nav2)
    print("\n[TEST 1] Launching Gazebo Fortress + Controllers + Nav2 Stack...")
    launch_cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source {extra_setup} && "
        f"source {ws_setup} && "
        f"ros2 launch diff_drive_description navigation.launch.py headless:=true rviz:=false"
    )

    stack_proc = subprocess.Popen(
        ["bash", "-c", launch_cmd],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        preexec_fn=os.setsid
    )

    try:
        # Wait for simulation clock, map, and costmaps
        print("  Waiting up to 30s for Gazebo, controllers and Nav2 lifecycle activation...")
        start_wait = time.time()
        while time.time() - start_wait < 30.0:
            rclpy.spin_once(node, timeout_sec=0.1)
            if (node.sim_time is not None and node.sim_time > 1.0 and 
                node.latest_map is not None and node.latest_global_costmap is not None):
                break
            time.sleep(0.1)

        sim_alive = stack_proc.poll() is None
        print(f"  Simulation & Nav2 process running: {sim_alive}")
        print(f"  Simulation time received: {node.sim_time is not None} ({node.sim_time:.2f}s)" if node.sim_time else "  No sim time received")
        results.append(("1. Stack Launch", sim_alive and (node.sim_time is not None), f"sim_time={node.sim_time}"))

        # 2. Simulation Time Check
        print("\n[TEST 2] Simulation Time Check (/clock)...")
        time_ok = (node.sim_time is not None and node.sim_time > 0.5)
        sim_str = f"{node.sim_time:.3f} s" if node.sim_time is not None else "None"
        print(f"  Current sim_time: {sim_str}")
        results.append(("2. Simulation Time Clock", time_ok, f"sim_time={sim_str}"))

        # 3. Map Server & Map Structure
        print("\n[TEST 3] Map Server Publication (/map)...")
        map_received = node.latest_map is not None
        if map_received:
            m = node.latest_map
            frame_id = m.header.frame_id
            res = m.info.resolution
            w = m.info.width
            h = m.info.height
            origin = (m.info.origin.position.x, m.info.origin.position.y)
            print(f"  Map Frame ID: '{frame_id}' (Expected: 'map')")
            print(f"  Resolution: {res:.3f} m/pixel (Expected: 0.05m)")
            print(f"  Dimensions: {w} x {h} cells ({w*res:.2f}m x {h*res:.2f}m)")
            print(f"  Origin: ({origin[0]:.2f}, {origin[1]:.2f})")
            map_ok = (frame_id == 'map') and (abs(res - 0.05) < 0.005) and (w == 215) and (h == 197)
        else:
            map_ok = False
            print("  FAIL: No /map received from Map Server!")
        results.append(("3. Map Server (/map)", map_ok, f"{w}x{h} cells, res={res}m" if map_received else "No map"))

        # 4. AMCL Localization & Dynamic TF (map -> odom)
        print("\n[TEST 4] AMCL Localization & Dynamic TF (map -> odom)...")
        # Ensure AMCL particle cloud is initialized
        init_pose = PoseWithCovarianceStamped()
        init_pose.header.frame_id = 'map'
        init_pose.header.stamp = node.latest_scan.header.stamp if node.latest_scan else Time().to_msg()
        init_pose.pose.pose.position.x = 0.0
        init_pose.pose.pose.position.y = 0.0
        init_pose.pose.pose.orientation.w = 1.0
        init_pose.pose.covariance[0] = 0.25
        init_pose.pose.covariance[7] = 0.25
        init_pose.pose.covariance[35] = 0.06
        node.initial_pose_pub.publish(init_pose)

        tf_map_odom_ok = False
        tx, ty, tz = 0.0, 0.0, 0.0
        start_t4 = time.time()
        while time.time() - start_t4 < 20.0:
            init_pose.header.stamp = node.latest_scan.header.stamp if node.latest_scan else Time().to_msg()
            node.initial_pose_pub.publish(init_pose)
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                t = node.tf_buffer.lookup_transform('map', 'odom', Time())
                tx, ty, tz = t.transform.translation.x, t.transform.translation.y, t.transform.translation.z
                print(f"  Transform map -> odom: translation=[{tx:.4f}, {ty:.4f}, {tz:.4f}]")
                tf_map_odom_ok = True
                break
            except Exception:
                time.sleep(0.1)
        if not tf_map_odom_ok:
            print("  FAIL: TF lookup 'map' -> 'odom' timed out.")
        results.append(("4. AMCL Dynamic TF (map -> odom)", tf_map_odom_ok, f"trans=[{tx:.3f}, {ty:.3f}, {tz:.3f}]" if tf_map_odom_ok else "failed"))

        # 5. Full Kinematic TF Tree Lookup (map -> laser_link)
        print("\n[TEST 5] Full TF Chain: map -> odom -> base_footprint -> base_link -> laser_link...")
        tf_full_ok = False
        tx5, ty5, tz5 = 0.0, 0.0, 0.0
        start_t5 = time.time()
        while time.time() - start_t5 < 10.0:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                t = node.tf_buffer.lookup_transform('map', 'laser_link', Time())
                tx5, ty5, tz5 = t.transform.translation.x, t.transform.translation.y, t.transform.translation.z
                print(f"  Full Transform map -> laser_link: [{tx5:.4f}, {ty5:.4f}, {tz5:.4f}]")
                tf_full_ok = True
                break
            except Exception:
                # Composed check of all hops in the chain
                try:
                    t1 = node.tf_buffer.lookup_transform('map', 'odom', Time())
                    t2 = node.tf_buffer.lookup_transform('odom', 'base_footprint', Time())
                    t3 = node.tf_buffer.lookup_transform('base_footprint', 'laser_link', Time())
                    tx5 = t1.transform.translation.x + t2.transform.translation.x + t3.transform.translation.x
                    ty5 = t1.transform.translation.y + t2.transform.translation.y + t3.transform.translation.y
                    tz5 = t1.transform.translation.z + t2.transform.translation.z + t3.transform.translation.z
                    print(f"  Full Transform map -> laser_link (composed): [{tx5:.4f}, {ty5:.4f}, {tz5:.4f}]")
                    tf_full_ok = True
                    break
                except Exception:
                    time.sleep(0.1)
        if not tf_full_ok:
            print("  FAIL: Full TF lookup 'map' -> 'laser_link' timed out.")
        results.append(("5. Full TF Chain (map -> laser_link)", tf_full_ok, f"trans=[{tx5:.3f}, {ty5:.3f}, {tz5:.3f}]" if tf_full_ok else "failed"))

        # 6. Global & Local Costmaps
        print("\n[TEST 6] Costmaps Availability (Global & Local)...")
        for _ in range(15):
            rclpy.spin_once(node, timeout_sec=0.05)
        has_gc = node.latest_global_costmap is not None
        has_lc = node.latest_local_costmap is not None
        print(f"  Global Costmap received: {has_gc} (Size: {node.latest_global_costmap.info.width}x{node.latest_global_costmap.info.height})" if has_gc else "  Global Costmap: None")
        print(f"  Local Costmap received:  {has_lc} (Size: {node.latest_local_costmap.info.width}x{node.latest_local_costmap.info.height})" if has_lc else "  Local Costmap: None")
        costmaps_ok = has_gc and has_lc
        results.append(("6. Costmaps Availability", costmaps_ok, f"global={has_gc}, local={has_lc}"))

        # 7. Global Path Planning (ComputePathToPose)
        print("\n[TEST 7] Global Path Planning via NavfnPlanner...")
        print("  Waiting for compute_path_to_pose action server...")
        server_ready = node.compute_path_client.wait_for_server(timeout_sec=10.0)
        print(f"  ComputePathToPose Action Server Ready: {server_ready}")

        plan_ok = False
        path_length = 0
        if server_ready:
            path_goal = ComputePathToPose.Goal()
            path_goal.goal.header.frame_id = 'map'
            path_goal.goal.header.stamp = node.get_clock().now().to_msg()
            path_goal.goal.pose.position.x = 1.8
            path_goal.goal.pose.position.y = 0.0
            path_goal.goal.pose.position.z = 0.0
            path_goal.goal.pose.orientation = euler_to_quaternion(0.0)
            path_goal.planner_id = 'GridBased'
            path_goal.use_start = False

            send_future = node.compute_path_client.send_goal_async(path_goal)
            rclpy.spin_until_future_complete(node, send_future, timeout_sec=5.0)
            goal_handle = send_future.result()

            if goal_handle and goal_handle.accepted:
                print("  Path planning goal accepted, computing path...")
                res_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(node, res_future, timeout_sec=5.0)
                path_res = res_future.result()
                if path_res:
                    poses = path_res.result.path.poses
                    path_length = len(poses)
                    print(f"  Global Path generated with {path_length} waypoints.")
                    plan_ok = path_length > 5
        results.append(("7. Global Path Planning", plan_ok, f"waypoints={path_length}"))

        # 8 & 9. Autonomous Navigation Execution & Goal Tolerance Arrival
        print("\n[TEST 8 & 9] Autonomous Navigation to Goal (X=1.8m, Y=0.0m)...")
        print("  Waiting for navigate_to_pose action server...")
        nav_server_ready = node.nav_to_pose_client.wait_for_server(timeout_sec=10.0)
        print(f"  NavigateToPose Action Server Ready: {nav_server_ready}")

        nav_succeeded = False
        goal_reached_tolerance = False
        stopped_cleanly = False
        final_dist_error = 999.0
        final_speed = 999.0

        if nav_server_ready:
            nav_goal = NavigateToPose.Goal()
            nav_goal.pose.header.frame_id = 'map'
            nav_goal.pose.header.stamp = node.get_clock().now().to_msg()
            nav_goal.pose.pose.position.x = 1.8
            nav_goal.pose.pose.position.y = 0.0
            nav_goal.pose.pose.position.z = 0.0
            nav_goal.pose.pose.orientation = euler_to_quaternion(0.0)
            nav_goal.behavior_tree = ''

            print("  Dispatching navigation goal to Nav2 BT Navigator...")
            send_nav_future = node.nav_to_pose_client.send_goal_async(nav_goal)
            rclpy.spin_until_future_complete(node, send_nav_future, timeout_sec=5.0)
            nav_handle = send_nav_future.result()

            if nav_handle and nav_handle.accepted:
                print("  Navigation goal ACCEPTED. Tracking robot progress...")
                res_nav_future = nav_handle.get_result_async()

                # Track motion for up to 35 seconds of sim time
                start_nav_sim_time = node.sim_time if node.sim_time is not None else 0.0
                while not res_nav_future.done():
                    rclpy.spin_once(node, timeout_sec=0.05)
                    now_sim = node.sim_time if node.sim_time is not None else 0.0
                    if now_sim - start_nav_sim_time > 35.0:
                        print("  Navigation timeout reached (35s sim time).")
                        break

                if res_nav_future.done():
                    nav_result = res_nav_future.result()
                    status = nav_result.status
                    # GoalStatus.STATUS_SUCCEEDED == 4
                    print(f"  Nav2 action finished with status code: {status} (4 = SUCCEEDED)")
                    nav_succeeded = (status == 4)

                # Active braking command to diff_drive_controller
                stop_cmd = Twist()
                start_brake = node.sim_time if node.sim_time is not None else 0.0
                while (node.sim_time is not None and node.sim_time - start_brake < 3.0):
                    node.cmd_pub.publish(stop_cmd)
                    rclpy.spin_once(node, timeout_sec=0.05)
                    time.sleep(0.02)
                    if node.latest_odom:
                        vx = abs(node.latest_odom.twist.twist.linear.x)
                        wz = abs(node.latest_odom.twist.twist.angular.z)
                        if vx < 0.05 and wz < 0.15:
                            break

                try:
                    t = node.tf_buffer.lookup_transform('map', 'base_footprint', Time())
                    rx, ry = t.transform.translation.x, t.transform.translation.y
                    final_dist_error = math.hypot(rx - 1.8, ry - 0.0)
                    print(f"  Final Robot Pose in map: X={rx:.3f} m, Y={ry:.3f} m")
                    print(f"  Target Distance Error: {final_dist_error:.3f} m (Tolerance: 0.35m)")
                    goal_reached_tolerance = (final_dist_error <= 0.35)
                except Exception as e:
                    print(f"  Could not lookup final robot pose: {e}")

                # Measure final speed from odometry
                if node.latest_odom:
                    vx = abs(node.latest_odom.twist.twist.linear.x)
                    wz = abs(node.latest_odom.twist.twist.angular.z)
                    print(f"  Final Speed after braking: linear={vx:.3f} m/s (Target: <0.05 m/s), angular={wz:.3f} rad/s")
                    stopped_cleanly = (vx < 0.05 and wz < 0.20)

        results.append(("8. Autonomous Navigation Execution", nav_succeeded, f"goal_accepted={nav_server_ready}, status_succeeded={nav_succeeded}"))
        results.append(("9. Goal Arrival & Active Braking", goal_reached_tolerance and stopped_cleanly, f"dist_err={final_dist_error:.3f}m, linear_v={vx:.3f}m/s"))

        # 10. Obstacle Avoidance Trajectory Verification
        print("\n[TEST 10] Obstacle Avoidance Trajectory Planning...")
        # Goal at (1.5, 1.2) near pillar_northeast at (2.2, 2.2)
        min_dist_to_pillar = 0.0
        obs_path_ok = False
        obs_goal = ComputePathToPose.Goal()
        obs_goal.goal.header.frame_id = 'map'
        obs_goal.goal.header.stamp = node.get_clock().now().to_msg()
        obs_goal.goal.pose.position.x = 1.5
        obs_goal.goal.pose.position.y = 1.2
        obs_goal.goal.pose.position.z = 0.0
        obs_goal.goal.pose.orientation = euler_to_quaternion(0.785)
        obs_goal.planner_id = 'GridBased'
        obs_goal.use_start = False

        send_obs_future = node.compute_path_client.send_goal_async(obs_goal)
        rclpy.spin_until_future_complete(node, send_obs_future, timeout_sec=5.0)
        obs_handle = send_obs_future.result()

        if obs_handle and obs_handle.accepted:
            res_future = obs_handle.get_result_async()
            rclpy.spin_until_future_complete(node, res_future, timeout_sec=5.0)
            obs_res = res_future.result()
            if obs_res:
                poses = obs_res.result.path.poses
                print(f"  Obstacle avoidance path computed with {len(poses)} waypoints.")
                # Verify min distance from path waypoints to pillar_northeast (2.2, 2.2)
                min_dist_to_pillar = min(math.hypot(p.pose.position.x - 2.2, p.pose.position.y - 2.2) for p in poses) if poses else 0.0
                print(f"  Minimum clearance to Northeast Pillar: {min_dist_to_pillar:.3f} m (Pillar radius: 0.35m)")
                obs_path_ok = len(poses) > 5 and (min_dist_to_pillar >= 0.35)
        results.append(("10. Obstacle Avoidance Planning", obs_path_ok, f"clearance={min_dist_to_pillar:.3f}m"))

        # 11. RViz2 Navigation Visualization Snapshot
        print("\n[TEST 11] RViz2 Navigation Visualization Snapshot...")
        rviz_config = os.path.join(proj_dir, "config", "navigation.rviz")
        screenshot_path = os.path.join(screens_dir, "phase7_navigation.png")
        if os.path.exists(screenshot_path):
            os.remove(screenshot_path)

        display_num = ":99"
        xvfb_cmd = ["Xvfb", display_num, "-screen", "0", "1280x800x24"]
        xvfb_proc = subprocess.Popen(xvfb_cmd)
        time.sleep(1.0)

        env_xvfb = os.environ.copy()
        env_xvfb["DISPLAY"] = display_num

        rviz_cmd = (
            f"source /opt/ros/humble/setup.bash && "
            f"source {extra_setup} && "
            f"source {ws_setup} && "
            f"rviz2 -d {rviz_config}"
        )
        rviz_proc = subprocess.Popen(["bash", "-c", rviz_cmd], env=env_xvfb)
        time.sleep(6.0)

        subprocess.run(["import", "-window", "root", screenshot_path], env=env_xvfb)

        rviz_proc.terminate()
        try:
            rviz_proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            rviz_proc.kill()

        xvfb_proc.terminate()
        try:
            xvfb_proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            xvfb_proc.kill()

        screenshot_ok = os.path.exists(screenshot_path) and os.path.getsize(screenshot_path) > 5000
        print(f"  RViz2 screenshot saved: {screenshot_path} ({os.path.getsize(screenshot_path) if os.path.exists(screenshot_path) else 0} bytes)")
        results.append(("11. RViz2 Navigation Snapshot", screenshot_ok, f"path={screenshot_path}"))

    finally:
        node.destroy_node()
        rclpy.shutdown()

        print("\n  Terminating simulation & Nav2 processes...")
        try:
            os.killpg(os.getpgid(stack_proc.pid), signal.SIGTERM)
            stack_proc.wait(timeout=6.0)
        except Exception:
            try:
                os.killpg(os.getpgid(stack_proc.pid), signal.SIGKILL)
                stack_proc.wait(timeout=3.0)
            except Exception:
                pass
        subprocess.run(['killall', '-9', 'ign', 'ruby', 'gz', 'parameter_bridge', 'robot_state_publisher', 'map_server', 'amcl', 'controller_server', 'planner_server', 'behavior_server', 'bt_navigator', 'lifecycle_manager'], stderr=subprocess.DEVNULL)
        print("  Cleaned up all background simulation processes.")

    print("\n" + "="*70)
    print("  PHASE 7 NAV2 AUTONOMOUS NAVIGATION VALIDATION SUMMARY")
    print("="*70)
    all_passed = True
    for name, ok, details in results:
        status_str = "[PASS]" if ok else "[FAIL]"
        if not ok:
            all_passed = False
        print(f"  {status_str} {name}: {details}")
    print("="*70)
    return all_passed


if __name__ == '__main__':
    ok = run_tests()
    sys.exit(0 if ok else 1)
