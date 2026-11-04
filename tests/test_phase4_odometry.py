#!/usr/bin/env python3
"""
Phase 4 Validation Suite: Odometry and TF2 Transformation Verification

Validates:
  1. /odom message publishing & rate (~50 Hz)
  2. nav_msgs/msg/Odometry message structure & covariances
  3. Linear and angular velocity estimation under motion
  4. Position and orientation integration (Odom vs Ground Truth)
  5. odom -> base_footprint TF transform
  6. base_footprint -> base_link static TF transform
  7. Wheel joint state publication
  8. Full TF tree connectivity (lookup_transform across multiple links)
  9. Transform timestamp synchronization with simulation clock (/clock)
"""

import os
import sys
import time
import math
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
import tf2_ros


class OdometryTFVerifier(Node):
    def __init__(self):
        super().__init__('phase4_odom_tf_verifier')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.js_sub = self.create_subscription(JointState, '/joint_states', self.js_cb, 10)
        
        # Buffer and Listener for TF validation
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Raw TF subscribers for header inspection
        self.dynamic_tf_sub = self.create_subscription(TFMessage, '/tf', self.tf_cb, 10)
        static_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.static_tf_sub = self.create_subscription(TFMessage, '/tf_static', self.tf_static_cb, static_qos)

        self.sim_time = None
        self.odom_history = []
        self.latest_odom = None
        self.latest_js = None
        self.raw_dynamic_tfs = []
        self.raw_static_tfs = []

    def clock_cb(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def odom_cb(self, msg):
        self.latest_odom = msg
        self.odom_history.append((self.sim_time, msg))
        if len(self.odom_history) > 100:
            self.odom_history.pop(0)

    def js_cb(self, msg):
        self.latest_js = msg

    def tf_cb(self, msg):
        for t in msg.transforms:
            self.raw_dynamic_tfs.append(t)

    def tf_static_cb(self, msg):
        for t in msg.transforms:
            self.raw_static_tfs.append(t)

    def wait_for_data(self, timeout_sec=15.0):
        start = time.time()
        while (self.sim_time is None or self.latest_odom is None or self.latest_js is None) and (time.time() - start < timeout_sec):
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.sim_time is not None and self.latest_odom is not None and self.latest_js is not None


def parse_gazebo_pose(output_str):
    lines = output_str.strip().splitlines()
    for idx, l in enumerate(lines):
        if "Pose [ XYZ (m) ] [ RPY (rad) ]:" in l and idx + 2 < len(lines):
            try:
                xyz = [float(val) for val in lines[idx + 1].strip().strip('[]').split()]
                rpy = [float(val) for val in lines[idx + 2].strip().strip('[]').split()]
                return xyz, rpy
            except Exception:
                pass
    return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]


def get_gz_pose():
    res = subprocess.run(['ign', 'model', '-m', 'diff_drive_robot', '-p'], capture_output=True, text=True)
    return parse_gazebo_pose(res.stdout)


def run_tests():
    print("==================================================================")
    print("  PROJECT 3 - PHASE 4: ODOMETRY & TF2 VALIDATION SUITE           ")
    print("==================================================================")

    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    launch_path = os.path.join(base_dir, 'launch', 'gazebo.launch.py')

    cmd = ['ros2', 'launch', launch_path, 'headless:=true']
    gz_proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print("Launching Gazebo Fortress + ros2_control...")
    time.sleep(11)

    rclpy.init()
    node = OdometryTFVerifier()
    if not node.wait_for_data():
        print("FAIL: Timeout waiting for clock / odom / joint_states")
        gz_proc.kill()
        return False

    results = []

    try:
        # Collect baseline messages (ensure at least 30 samples)
        rate = 0.0
        start_wait = time.time()
        while len(node.odom_history) < 40 and (time.time() - start_wait < 5.0) and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)

        # 1. /odom message publishing & rate
        print("\n[TEST 1] /odom Publishing Rate & Continuity...")
        if len(node.odom_history) >= 10:
            t0, _ = node.odom_history[0]
            t1, _ = node.odom_history[-1]
            dt = t1 - t0
            rate = (len(node.odom_history) - 1) / dt if dt > 0 else 0
            print(f"  Calculated Publish Rate: {rate:.1f} Hz (Samples: {len(node.odom_history)}, Target: 50.0 Hz)")
            passed = (rate > 35.0) and (rate < 65.0)
        else:
            print(f"  FAIL: Insufficient odom history: {len(node.odom_history)} samples")
            passed = False
        results.append(("1. /odom Publish Rate", passed, f"rate={rate:.1f}Hz"))

        # 2. nav_msgs/msg/Odometry Structure & Covariances
        print("\n[TEST 2] nav_msgs/msg/Odometry Field Structure & Frames...")
        m = node.latest_odom
        has_frames = (m.header.frame_id == 'odom') and (m.child_frame_id == 'base_footprint')
        has_pose_cov = (len(m.pose.covariance) == 36) and (m.pose.covariance[0] > 0)
        has_twist_cov = (len(m.twist.covariance) == 36) and (m.twist.covariance[0] > 0)
        print(f"  Header Frame: '{m.header.frame_id}' | Child Frame: '{m.child_frame_id}'")
        print(f"  Pose Covariance diag[0]: {m.pose.covariance[0]} | Twist Covariance diag[0]: {m.twist.covariance[0]}")
        passed = has_frames and has_pose_cov and has_twist_cov
        results.append(("2. Odometry Message Structure", passed, f"frame={m.header.frame_id}->{m.child_frame_id}"))

        # 3. Linear & Angular Velocity Estimation
        print("\n[TEST 3] Velocity Estimates under Commanded Motion...")
        # Command linear velocity
        twist = Twist()
        twist.linear.x = 0.5
        t_end = node.sim_time + 1.5
        while node.sim_time < t_end and rclpy.ok():
            node.cmd_pub.publish(twist)
            rclpy.spin_once(node, timeout_sec=0.02)
        
        lin_est = node.latest_odom.twist.twist.linear.x
        print(f"  Commanded Lin Vel: 0.5 m/s | Estimated Lin Vel: {lin_est:.4f} m/s")
        lin_ok = abs(lin_est - 0.5) < 0.05

        # Command angular velocity
        twist.linear.x = 0.0
        twist.angular.z = 1.0
        t_end = node.sim_time + 1.5
        while node.sim_time < t_end and rclpy.ok():
            node.cmd_pub.publish(twist)
            rclpy.spin_once(node, timeout_sec=0.02)

        ang_est = node.latest_odom.twist.twist.angular.z
        print(f"  Commanded Ang Vel: 1.0 rad/s | Estimated Ang Vel: {ang_est:.4f} rad/s")
        ang_ok = abs(ang_est - 1.0) < 0.08

        # Stop
        stop_twist = Twist()
        for _ in range(10):
            node.cmd_pub.publish(stop_twist)
            rclpy.spin_once(node, timeout_sec=0.02)

        passed = lin_ok and ang_ok
        results.append(("3. Velocity Estimation", passed, f"lin_est={lin_est:.3f}, ang_est={ang_est:.3f}"))

        # 4. Position & Orientation Integration Comparison
        print("\n[TEST 4] Pose Integration: /odom vs Gazebo Ground Truth...")
        # Let robot settle
        t_settle = node.sim_time + 0.5
        while node.sim_time < t_settle and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)

        odom_p = node.latest_odom.pose.pose.position
        odom_q = node.latest_odom.pose.pose.orientation
        siny_cosp = 2 * (odom_q.w * odom_q.z + odom_q.x * odom_q.y)
        cosy_cosp = 1 - 2 * (odom_q.y * odom_q.y + odom_q.z * odom_q.z)
        odom_yaw = math.atan2(siny_cosp, cosy_cosp)

        gz_xyz, gz_rpy = get_gz_pose()
        
        pos_err = math.hypot(odom_p.x - gz_xyz[0], odom_p.y - gz_xyz[1])
        yaw_err = abs(math.atan2(math.sin(odom_yaw - gz_rpy[2]), math.cos(odom_yaw - gz_rpy[2])))
        print(f"  Odom Pose:   X={odom_p.x:.4f} m, Y={odom_p.y:.4f} m, Yaw={odom_yaw:.4f} rad")
        print(f"  Gazebo Pose: X={gz_xyz[0]:.4f} m, Y={gz_xyz[1]:.4f} m, Yaw={gz_rpy[2]:.4f} rad")
        print(f"  Discrepancy: Pos Err = {pos_err:.4f} m, Yaw Err = {yaw_err:.4f} rad ({math.degrees(yaw_err):.2f} deg)")
        passed = (pos_err < 0.08) and (yaw_err < 0.30)
        results.append(("4. Odom Integration Accuracy", passed, f"pos_err={pos_err:.4f}m, yaw_err={yaw_err:.4f}rad"))

        # 5. odom -> base_footprint dynamic TF
        print("\n[TEST 5] Dynamic TF Transform: odom -> base_footprint...")
        dyn_pairs = set((t.header.frame_id, t.child_frame_id) for t in node.raw_dynamic_tfs)
        has_odom_bf = ('odom', 'base_footprint') in dyn_pairs
        print(f"  Dynamic TF Pairs Detected: {dyn_pairs}")
        passed = has_odom_bf
        results.append(("5. Dynamic TF (odom->base_footprint)", passed, f"present={has_odom_bf}"))

        # 6. base_footprint -> base_link static TF
        print("\n[TEST 6] Static TF Transform: base_footprint -> base_link...")
        static_pairs = set((t.header.frame_id, t.child_frame_id) for t in node.raw_static_tfs)
        has_bf_bl = ('base_footprint', 'base_link') in static_pairs
        has_bl_cw = ('base_link', 'caster_wheel_link') in static_pairs
        print(f"  Static TF Pairs Detected: {static_pairs}")
        passed = has_bf_bl and has_bl_cw
        results.append(("6. Static TF (base_footprint->base_link)", passed, f"present={has_bf_bl}"))

        # 7. Wheel Joint States Publication
        print("\n[TEST 7] Wheel Joint States Integrity...")
        js = node.latest_js
        has_left = 'left_wheel_joint' in js.name
        has_right = 'right_wheel_joint' in js.name
        has_pos = len(js.position) >= 2
        has_vel = len(js.velocity) >= 2
        print(f"  Joints Published: {js.name}")
        passed = has_left and has_right and has_pos and has_vel
        results.append(("7. Wheel Joint States", passed, f"joints={js.name}"))

        # 8. Complete TF Tree Connectivity (Lookup Transform)
        print("\n[TEST 8] Full TF Tree Connectivity via tf2 Buffer Lookup...")
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.05)
        
        lookup_ok = True
        try:
            # Lookup odom -> base_link
            t_bl = node.tf_buffer.lookup_transform('odom', 'base_link', Time())
            print(f"  [OK] Lookup 'odom' -> 'base_link': trans=[{t_bl.transform.translation.x:.3f}, {t_bl.transform.translation.y:.3f}, {t_bl.transform.translation.z:.3f}]")
            # Lookup odom -> left_wheel_link
            t_lw = node.tf_buffer.lookup_transform('odom', 'left_wheel_link', Time())
            print(f"  [OK] Lookup 'odom' -> 'left_wheel_link': trans=[{t_lw.transform.translation.x:.3f}, {t_lw.transform.translation.y:.3f}, {t_lw.transform.translation.z:.3f}]")
            # Lookup odom -> caster_wheel_link
            t_cw = node.tf_buffer.lookup_transform('odom', 'caster_wheel_link', Time())
            print(f"  [OK] Lookup 'odom' -> 'caster_wheel_link': trans=[{t_cw.transform.translation.x:.3f}, {t_cw.transform.translation.y:.3f}, {t_cw.transform.translation.z:.3f}]")
        except Exception as e:
            print(f"  FAIL: TF Lookup exception: {e}")
            lookup_ok = False
        passed = lookup_ok
        results.append(("8. Full TF Tree Connectivity", passed, f"lookup_ok={lookup_ok}"))

        # 9. Transform Timestamps and Simulation Time Synchronization
        print("\n[TEST 9] Transform Timestamps Synchronization with /clock...")
        t_sample = node.raw_dynamic_tfs[-1] if node.raw_dynamic_tfs else None
        if t_sample:
            stamp_sec = t_sample.header.stamp.sec + t_sample.header.stamp.nanosec * 1e-9
            time_diff = abs(node.sim_time - stamp_sec)
            print(f"  Sim Time: {node.sim_time:.3f} s | Latest TF Stamp: {stamp_sec:.3f} s | Diff: {time_diff:.4f} s")
            passed = time_diff < 0.10
        else:
            print("  FAIL: No dynamic TF sample available")
            passed = False
        results.append(("9. TF Time Sync", passed, f"diff={time_diff:.4f}s"))

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
    print("  PHASE 4 ODOMETRY & TF2 VALIDATION SUMMARY")
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
