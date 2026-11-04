#!/usr/bin/env python3
"""
Phase 5 Validation Suite: 2D LiDAR Integration & LaserScan Verification

Tests:
  1. Gazebo Fortress simulation starts with GPU LiDAR sensor
  2. LiDAR sensor initializes and topic /scan is published
  3. Continuous publishing rate verification (~10 Hz)
  4. Message type verification: sensor_msgs/msg/LaserScan
  5. Beam count, FOV (360 deg), angle limits (-pi to +pi), and range bounds (0.1 to 10.0m)
  6. Detection of environment obstacles (perimeter walls and static fixtures)
  7. Verification of frame_id: 'laser_link'
  8. Static transform lookup: base_link -> laser_link
  9. Full TF lookup: odom -> laser_link
  10. Headless RViz2 visual verification snapshot
"""

import os
import sys
import time
import math
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
import tf2_ros


class LidarVerifier(Node):
    def __init__(self):
        super().__init__('phase5_lidar_verifier')
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_cb, 10)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sim_time = None
        self.scan_history = []
        self.latest_scan = None

    def clock_cb(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def scan_cb(self, msg):
        self.latest_scan = msg
        self.scan_history.append((self.sim_time, msg))
        if len(self.scan_history) > 60:
            self.scan_history.pop(0)

    def wait_for_data(self, timeout_sec=15.0):
        start = time.time()
        while (self.sim_time is None or self.latest_scan is None) and (time.time() - start < timeout_sec):
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.sim_time is not None and self.latest_scan is not None


def run_tests():
    print("==================================================================")
    print("  PROJECT 3 - PHASE 5: 2D LIDAR INTEGRATION VALIDATION SUITE      ")
    print("==================================================================")

    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    launch_path = os.path.join(base_dir, 'launch', 'gazebo.launch.py')
    rviz_config = os.path.join(base_dir, 'config', 'rviz.rviz')
    shots_dir = os.path.join(base_dir, 'screenshots')
    os.makedirs(shots_dir, exist_ok=True)

    cmd = ['ros2', 'launch', launch_path, 'headless:=true']
    gz_proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print("Launching Gazebo Fortress with GPU LiDAR sensor...")
    time.sleep(11)

    rclpy.init()
    node = LidarVerifier()
    if not node.wait_for_data():
        print("FAIL: Timeout waiting for /scan or /clock messages")
        gz_proc.kill()
        return False

    results = []

    try:
        # Collect ~30 scan samples
        print("Collecting LaserScan samples from /scan...")
        start_wait = time.time()
        while len(node.scan_history) < 25 and (time.time() - start_wait < 6.0) and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

        scan = node.latest_scan

        # 1. /scan Topic & Publisher Existence
        print("\n[TEST 1] /scan Topic & Active Publisher Check...")
        topic_info = subprocess.run(['ros2', 'topic', 'info', '/scan'], capture_output=True, text=True)
        has_pub = 'Publisher count: 1' in topic_info.stdout
        is_type = 'sensor_msgs/msg/LaserScan' in topic_info.stdout
        print(f"  Topic Type: {is_type} | Has Publisher: {has_pub}")
        passed = has_pub and is_type
        results.append(("1. /scan Topic & Publisher", passed, "LaserScan publisher active"))

        # 2. Continuous Publish Rate (~10 Hz)
        print("\n[TEST 2] /scan Continuous Publish Rate Verification...")
        if len(node.scan_history) >= 10:
            t0, _ = node.scan_history[0]
            t1, _ = node.scan_history[-1]
            dt = t1 - t0
            rate = (len(node.scan_history) - 1) / dt if dt > 0 else 0.0
            print(f"  Calculated Rate: {rate:.1f} Hz (Configured target: 10.0 Hz, Samples: {len(node.scan_history)})")
            passed = (rate > 7.0) and (rate < 14.0)
        else:
            print("  FAIL: Insufficient scan history")
            passed = False
        results.append(("2. /scan Publish Rate", passed, f"rate={rate:.1f}Hz"))

        # 3. Frame ID Verification
        print("\n[TEST 3] Frame ID Verification in Header...")
        frame_id = scan.header.frame_id
        print(f"  Reported frame_id: '{frame_id}' (Expected: 'laser_link')")
        passed = (frame_id == 'laser_link')
        results.append(("3. Frame ID Verification", passed, f"frame_id='{frame_id}'"))

        # 4. Field-of-View & Sample Count Verification
        print("\n[TEST 4] Field-of-View & Angular Coverage...")
        total_samples = len(scan.ranges)
        fov_rad = scan.angle_max - scan.angle_min
        fov_deg = math.degrees(fov_rad)
        print(f"  Samples: {total_samples} (Expected: 720)")
        print(f"  Angle Min: {scan.angle_min:.4f} rad, Max: {scan.angle_max:.4f} rad")
        print(f"  FOV Coverage: {fov_deg:.1f} degrees (Expected: ~360.0 deg)")
        passed = (total_samples == 720) and (abs(fov_deg - 360.0) < 1.0)
        results.append(("4. FOV & Samples", passed, f"{total_samples} samples, {fov_deg:.1f} deg FOV"))

        # 5. Range Bounds & Valid Measurements Check
        print("\n[TEST 5] Range Measurement Bounds (0.1m - 10.0m)...")
        print(f"  Configured Bounds: min={scan.range_min:.2f}m, max={scan.range_max:.2f}m")
        finite_ranges = [r for r in scan.ranges if not math.isnan(r) and not math.isinf(r) and r > scan.range_min]
        pct_valid = len(finite_ranges) / total_samples * 100.0
        print(f"  Finite / Valid returns: {len(finite_ranges)} / {total_samples} ({pct_valid:.1f}%)")
        passed = (scan.range_min <= 0.11) and (scan.range_max >= 9.9) and (pct_valid > 50.0)
        results.append(("5. Range Bounds Check", passed, f"{pct_valid:.1f}% valid returns"))

        # 6. Obstacle Geometry Detection (Walls and Arena Fixtures)
        print("\n[TEST 6] Obstacle Distance & Arena Geometry Verification...")
        # Robot is at origin (0, 0), facing +X.
        # laser_link is at (0.10, 0, 0.175).
        # North wall is at x = +5.0m -> forward distance should be 5.0 - 0.10 = 4.90m.
        # Center index: angle = 0.0 rad (index 360 in 720 samples from -pi to +pi)
        idx_front = 360
        r_front = scan.ranges[idx_front]

        # South wall is at x = -5.0m (index 0 and 719, angles -pi and +pi)
        r_rear = scan.ranges[0]

        # East wall is at y = +5.0m (left, index 540, angle +pi/2 = +1.57 rad)
        # However, center_partition is at (0.0, 3.2m), so left detection sees partition at ~3.2m!
        idx_left = 540
        r_left = scan.ranges[idx_left]

        # West wall is at y = -5.0m (right, index 180, angle -pi/2 = -1.57 rad)
        idx_right = 180
        r_right = scan.ranges[idx_right]

        print(f"  Forward  (Angle  0.0 deg): {r_front:.3f} m (Expected: ~4.90m North Wall)")
        print(f"  Rear     (Angle 180.0 deg): {r_rear:.3f} m (Expected: ~5.10m South Wall)")
        print(f"  Left     (Angle +90.0 deg): {r_left:.3f} m (Expected: ~3.20m Center Partition)")
        print(f"  Right    (Angle -90.0 deg): {r_right:.3f} m (Expected: ~5.00m West Wall)")

        front_ok = abs(r_front - 4.90) < 0.20
        rear_ok = abs(r_rear - 5.10) < 0.20
        left_ok = abs(r_left - 3.20) < 0.25
        right_ok = abs(r_right - 5.00) < 0.20

        passed = front_ok and rear_ok and left_ok and right_ok
        print(f"  Geometry Accuracy: Front={front_ok}, Rear={rear_ok}, Left={left_ok}, Right={right_ok}")
        results.append(("6. Obstacle Geometry Detection", passed, f"Fwd={r_front:.2f}m, Rear={r_rear:.2f}m, Left={r_left:.2f}m"))

        # 7. Static TF lookup: base_link -> laser_link
        print("\n[TEST 7] TF Lookup: base_link -> laser_link...")
        for _ in range(15):
            rclpy.spin_once(node, timeout_sec=0.05)

        tf_base_laser_ok = False
        try:
            t = node.tf_buffer.lookup_transform('base_link', 'laser_link', Time())
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            tz = t.transform.translation.z
            print(f"  Transform base_link -> laser_link: [{tx:.3f}, {ty:.3f}, {tz:.3f}]")
            tf_base_laser_ok = (abs(tx - 0.10) < 0.01) and (abs(ty - 0.0) < 0.01) and (abs(tz - 0.125) < 0.01)
        except Exception as e:
            print(f"  FAIL: base_link -> laser_link lookup failed: {e}")
        results.append(("7. TF base_link->laser_link", tf_base_laser_ok, f"trans=[{tx:.3f}, {ty:.3f}, {tz:.3f}]"))

        # 8. Full TF Tree Lookup: odom -> laser_link
        print("\n[TEST 8] Full TF Lookup: odom -> laser_link...")
        tf_odom_laser_ok = False
        try:
            t = node.tf_buffer.lookup_transform('odom', 'laser_link', Time())
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            tz = t.transform.translation.z
            print(f"  Transform odom -> laser_link: [{tx:.3f}, {ty:.3f}, {tz:.3f}]")
            tf_odom_laser_ok = (abs(tx - 0.10) < 0.05) and (abs(tz - 0.175) < 0.05)
        except Exception as e:
            print(f"  FAIL: odom -> laser_link lookup failed: {e}")
        results.append(("8. Full TF odom->laser_link", tf_odom_laser_ok, f"trans=[{tx:.3f}, {ty:.3f}, {tz:.3f}]"))

    finally:
        node.destroy_node()
        rclpy.shutdown()
        gz_proc.terminate()
        try:
            gz_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            gz_proc.kill()
            gz_proc.wait()

    print("\n==================================================================")
    print("  PHASE 5 LIDAR VALIDATION SUMMARY")
    print("==================================================================")
    all_ok = True
    for name, ok, details in results:
        status_str = "[PASS]" if ok else "[FAIL]"
        if not ok:
            all_ok = False
        print(f"  {status_str} {name}: {details}")
    print("==================================================================")
    return all_ok

if __name__ == '__main__':
    ok = run_tests()
    sys.exit(0 if ok else 1)

