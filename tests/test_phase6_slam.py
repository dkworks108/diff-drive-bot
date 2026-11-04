#!/usr/bin/env python3
"""
Phase 6 Validation Suite: SLAM Toolbox 2D Mapping Verification

Tests:
  1. Complete SLAM stack launch (Gazebo Fortress + controllers + slam_toolbox)
  2. Simulation time synchronization (/clock)
  3. /map topic and nav_msgs/msg/OccupancyGrid structure
  4. Map resolution (0.05m), dimensions, frame_id ('map'), and cell classification
  5. SLAM transform broadcast: map -> odom
  6. Full kinematic TF chain lookup: map -> odom -> base_footprint -> base_link -> laser_link
  7. Dynamic mapping trajectory: drive robot around arena, verify map growth
  8. Map persistence via map_saver_cli (generates maps/test_world_map.yaml & .pgm)
  9. Visual verification: RViz2 SLAM displays snapshot capture
"""

import os
import sys
import time
import math
import subprocess
import signal
import numpy as np
from PIL import Image

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
import tf2_ros


class SlamVerifier(Node):
    def __init__(self):
        super().__init__('phase6_slam_verifier')

        # Subscriptions
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # /map is published with TRANSIENT_LOCAL durability
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_cb, map_qos)

        # Publisher for motion commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # TF2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State tracking
        self.sim_time = None
        self.latest_map = None
        self.map_count = 0
        self.latest_scan = None
        self.latest_odom = None
        self.odom_history = []

    def clock_cb(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def map_cb(self, msg):
        self.latest_map = msg
        self.map_count += 1

    def scan_cb(self, msg):
        self.latest_scan = msg

    def odom_cb(self, msg):
        self.latest_odom = msg
        self.odom_history.append((self.sim_time, msg))
        if len(self.odom_history) > 100:
            self.odom_history.pop(0)

    def wait_for_sim_time(self, duration_sec):
        """Advance execution until duration_sec of simulation time has passed."""
        if self.sim_time is None:
            time.sleep(duration_sec)
            return
        start = self.sim_time
        while (self.sim_time is None or (self.sim_time - start < duration_sec)):
            rclpy.spin_once(self, timeout_sec=0.02)

    def publish_cmd(self, linear_x, angular_z, duration_sim_sec):
        """Publish velocity command smoothly over simulation time duration."""
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)

        start = self.sim_time if self.sim_time is not None else 0.0
        while True:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.02)
            now = self.sim_time if self.sim_time is not None else 0.0
            if now - start >= duration_sim_sec:
                break

    def stop_robot(self):
        """Actively command zero velocity and wait for robot to settle."""
        stop_twist = Twist()
        start = self.sim_time if self.sim_time is not None else 0.0
        while True:
            self.cmd_pub.publish(stop_twist)
            rclpy.spin_once(self, timeout_sec=0.02)
            now = self.sim_time if self.sim_time is not None else 0.0
            if now - start >= 1.0:
                break


def run_tests():
    print("="*70)
    print("  PROJECT 3 - PHASE 6: SLAM TOOLBOX 2D MAPPING VALIDATION SUITE")
    print("="*70)

    proj_dir = "/media/bumble/DATA_DISK/robotics/projects/03_diff_drive_robot"
    extra_setup = "/media/bumble/DATA_DISK/robotics/opt/ros_extra/setup.bash"
    ws_setup = os.path.join(proj_dir, "install", "setup.bash")
    maps_dir = os.path.join(proj_dir, "maps")
    screens_dir = os.path.join(proj_dir, "screenshots")
    os.makedirs(maps_dir, exist_ok=True)
    os.makedirs(screens_dir, exist_ok=True)

    results = []

    # 1. Launch complete stack (Gazebo Fortress + diff_drive controllers + SLAM Toolbox)
    print("\n[TEST 1] Launching Gazebo Fortress + Controllers + SLAM Toolbox...")
    launch_cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source {extra_setup} && "
        f"source {ws_setup} && "
        f"ros2 launch diff_drive_description slam.launch.py headless:=true rviz:=false"
    )

    stack_proc = subprocess.Popen(
        ["bash", "-c", launch_cmd],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        preexec_fn=os.setsid
    )

    rclpy.init()
    node = SlamVerifier()

    try:
        # Wait for simulation clock and initial map publication
        print("  Waiting up to 25s for Gazebo, controllers and SLAM Toolbox to initialize...")
        start_wait = time.time()
        while time.time() - start_wait < 25.0:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.sim_time is not None and node.latest_map is not None:
                break
            time.sleep(0.1)

        sim_alive = stack_proc.poll() is None
        print(f"  Simulation & SLAM process running: {sim_alive}")
        print(f"  Simulation time received: {node.sim_time is not None} ({node.sim_time:.2f}s)" if node.sim_time else "  No sim time received")
        results.append(("1. Stack Launch", sim_alive and (node.sim_time is not None), f"sim_time={node.sim_time}"))

        # 2. Simulation Time Check
        print("\n[TEST 2] Simulation Time Check (/clock)...")
        time_ok = (node.sim_time is not None and node.sim_time > 0.5)
        sim_str = f"{node.sim_time:.3f} s" if node.sim_time is not None else "None"
        print(f"  Current sim_time: {sim_str}")
        results.append(("2. Simulation Time Clock", time_ok, f"sim_time={sim_str}"))

        # 3. Initial Map Topic & Structure
        print("\n[TEST 3] Initial /map Topic & Structure...")
        map_received = node.latest_map is not None
        if not map_received:
            # Spin a bit more if first map update hasn't triggered yet
            print("  Waiting for initial map update (up to 10s)...")
            start_wait = time.time()
            while time.time() - start_wait < 10.0 and node.latest_map is None:
                rclpy.spin_once(node, timeout_sec=0.1)
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
            print(f"  Dimensions: {w} x {h} cells ({w*res:.1f}m x {h*res:.1f}m coverage)")
            print(f"  Origin: ({origin[0]:.2f}, {origin[1]:.2f})")
            map_struct_ok = (frame_id == 'map') and (abs(res - 0.05) < 0.005) and (w > 50) and (h > 50)
        else:
            map_struct_ok = False
            print("  FAIL: No /map message received!")

        results.append(("3. /map Message Structure", map_struct_ok, f"res={res}m, size={w}x{h}, frame='{frame_id}'" if map_received else "No map"))

        # 4. Map Cell Distribution & Classification
        print("\n[TEST 4] Map Cell Distribution (Unknown, Free, Occupied)...")
        cells = np.array(node.latest_map.data, dtype=int)
        unknown_count = np.sum(cells == -1)
        free_count = np.sum(cells == 0)
        occupied_count = np.sum(cells > 0)
        total_cells = len(cells)

        print(f"  Total cells: {total_cells}")
        print(f"  Unknown (-1):  {unknown_count:6d} ({unknown_count/total_cells*100.2:.1f}%)")
        print(f"  Free space (0):{free_count:6d} ({free_count/total_cells*100.2:.1f}%)")
        print(f"  Occupied (>0): {occupied_count:6d} ({occupied_count/total_cells*100.2:.1f}%)")

        initial_known_cells = free_count + occupied_count
        cells_classified_ok = (unknown_count > 0) and (free_count > 100) and (occupied_count > 10)
        print(f"  Initial known cells: {initial_known_cells} (Free + Obstacles)")
        results.append(("4. Cell Classification", cells_classified_ok, f"unknown={unknown_count}, free={free_count}, occupied={occupied_count}"))

        # 5. TF Lookup: map -> odom
        print("\n[TEST 5] TF Transform: map -> odom...")
        for _ in range(15):
            rclpy.spin_once(node, timeout_sec=0.05)

        tf_map_odom_ok = False
        try:
            t = node.tf_buffer.lookup_transform('map', 'odom', Time())
            tx, ty, tz = t.transform.translation.x, t.transform.translation.y, t.transform.translation.z
            print(f"  Transform map -> odom: translation=[{tx:.4f}, {ty:.4f}, {tz:.4f}]")
            tf_map_odom_ok = True
        except Exception as e:
            print(f"  FAIL: TF lookup 'map' -> 'odom' failed: {e}")
        results.append(("5. TF map -> odom", tf_map_odom_ok, f"trans=[{tx:.3f}, {ty:.3f}, {tz:.3f}]" if tf_map_odom_ok else "failed"))

        # 6. Full TF Chain Lookup: map -> laser_link
        print("\n[TEST 6] Full TF Chain: map -> odom -> base_footprint -> base_link -> laser_link...")
        tf_full_ok = False
        try:
            t = node.tf_buffer.lookup_transform('map', 'laser_link', Time())
            tx, ty, tz = t.transform.translation.x, t.transform.translation.y, t.transform.translation.z
            print(f"  Full Transform map -> laser_link: [{tx:.4f}, {ty:.4f}, {tz:.4f}]")
            tf_full_ok = True
        except Exception as e:
            print(f"  FAIL: Full TF lookup 'map' -> 'laser_link' failed: {e}")
        results.append(("6. Full TF Tree (map -> laser_link)", tf_full_ok, f"trans=[{tx:.3f}, {ty:.3f}, {tz:.3f}]" if tf_full_ok else "failed"))

        # 7. Dynamic Mapping Maneuver: Drive robot around arena and observe map growth
        print("\n[TEST 7] Dynamic Mapping Maneuver (Moving robot to expand map)...")
        initial_map_count = node.map_count
        initial_odom_x = node.latest_odom.pose.pose.position.x if node.latest_odom else 0.0
        initial_odom_y = node.latest_odom.pose.pose.position.y if node.latest_odom else 0.0

        print(f"  Initial Odom Pose: X={initial_odom_x:.3f} m, Y={initial_odom_y:.3f} m")
        print(f"  Initial Known Cells: {initial_known_cells}")

        # Leg 1: Drive forward 1.2m
        print("  Leg 1: Driving forward 1.2m at 0.4 m/s (sim time)...")
        node.publish_cmd(linear_x=0.4, angular_z=0.0, duration_sim_sec=3.0)
        node.stop_robot()

        # Leg 2: Rotate CCW 90 degrees (~1.57 rad)
        print("  Leg 2: Rotating CCW 90 degrees at 0.5 rad/s (sim time)...")
        node.publish_cmd(linear_x=0.0, angular_z=0.5, duration_sim_sec=3.14)
        node.stop_robot()

        # Leg 3: Drive along new heading 1.0m
        print("  Leg 3: Driving forward 1.0m along new heading at 0.4 m/s (sim time)...")
        node.publish_cmd(linear_x=0.4, angular_z=0.0, duration_sim_sec=2.5)
        node.stop_robot()

        # Allow SLAM graph and map publisher to process scans
        print("  Waiting 6s sim time for SLAM graph optimization & map updates...")
        node.wait_for_sim_time(6.0)

        final_odom_x = node.latest_odom.pose.pose.position.x if node.latest_odom else 0.0
        final_odom_y = node.latest_odom.pose.pose.position.y if node.latest_odom else 0.0
        dist_moved = math.hypot(final_odom_x - initial_odom_x, final_odom_y - initial_odom_y)

        # Analyze updated map
        final_cells = np.array(node.latest_map.data, dtype=int)
        final_free_count = np.sum(final_cells == 0)
        final_occupied_count = np.sum(final_cells > 0)
        final_known_cells = final_free_count + final_occupied_count
        cell_gain = final_known_cells - initial_known_cells
        pct_gain = (cell_gain / initial_known_cells * 100.0) if initial_known_cells > 0 else 0

        print(f"  Final Odom Pose: X={final_odom_x:.3f} m, Y={final_odom_y:.3f} m (Displacement: {dist_moved:.3f} m)")
        print(f"  Map update messages received: {node.map_count - initial_map_count}")
        print(f"  Final Known Cells: {final_known_cells} (Free: {final_free_count}, Occupied: {final_occupied_count})")
        print(f"  Map Expansion: +{cell_gain} cells (+{pct_gain:.1f}%)")

        dynamic_map_ok = (dist_moved > 0.8) and (cell_gain > 50) and (node.map_count > initial_map_count)
        results.append(("7. Dynamic Mapping Expansion", dynamic_map_ok, f"dist={dist_moved:.2f}m, cell_gain=+{cell_gain} (+{pct_gain:.1f}%)"))

        # 8. Save map using map_saver_cli
        print("\n[TEST 8] Saving Map via map_saver_cli...")
        map_basename = os.path.join(maps_dir, "test_world_map")
        yaml_path = f"{map_basename}.yaml"
        pgm_path = f"{map_basename}.pgm"

        # Remove previous map files if any
        if os.path.exists(yaml_path):
            os.remove(yaml_path)
        if os.path.exists(pgm_path):
            os.remove(pgm_path)

        save_cmd = (
            f"source /opt/ros/humble/setup.bash && "
            f"source {extra_setup} && "
            f"ros2 run nav2_map_server map_saver_cli -t /map -f {map_basename} --ros-args -p use_sim_time:=true"
        )
        save_res = subprocess.run(["bash", "-c", save_cmd], capture_output=True, text=True, timeout=15)
        print(f"  map_saver_cli output:\n  {save_res.stdout.strip()}")

        files_saved = os.path.exists(yaml_path) and os.path.exists(pgm_path)
        yaml_size = os.path.getsize(yaml_path) if os.path.exists(yaml_path) else 0
        pgm_size = os.path.getsize(pgm_path) if os.path.exists(pgm_path) else 0
        print(f"  Saved YAML ({yaml_path}): {yaml_size} bytes")
        print(f"  Saved PGM  ({pgm_path}):  {pgm_size} bytes")

        map_save_ok = files_saved and (yaml_size > 50) and (pgm_size > 1000)
        results.append(("8. Map Saving (map_saver_cli)", map_save_ok, f"yaml={yaml_size}B, pgm={pgm_size}B"))

        # 9. RViz2 Visual Verification Snapshot
        print("\n[TEST 9] RViz2 SLAM Visualization Snapshot...")
        rviz_config = os.path.join(proj_dir, "config", "slam.rviz")
        screenshot_path = os.path.join(screens_dir, "phase6_slam_mapping.png")
        if os.path.exists(screenshot_path):
            os.remove(screenshot_path)

        # Launch virtual X display and RViz2
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

        # Capture root window screenshot using ImageMagick 'import'
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
        results.append(("9. RViz2 Visualization Snapshot", screenshot_ok, f"path={screenshot_path}"))

    finally:
        node.destroy_node()
        rclpy.shutdown()

        # Clean shutdown of Gazebo and SLAM processes
        print("\n  Terminating simulation & SLAM processes...")
        try:
            os.killpg(os.getpgid(stack_proc.pid), signal.SIGTERM)
            stack_proc.wait(timeout=6.0)
        except Exception:
            try:
                os.killpg(os.getpgid(stack_proc.pid), signal.SIGKILL)
                stack_proc.wait(timeout=3.0)
            except Exception:
                pass
        print("  Cleaned up all background simulation processes.")

    print("\n" + "="*70)
    print("  PHASE 6 SLAM TOOLBOX 2D MAPPING VALIDATION SUMMARY")
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
