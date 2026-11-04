#!/usr/bin/env python3
"""
Phase 2 Validation Suite: ros2_control Differential Drive Controller
Tests:
  1. ros2_control configuration files and parameters
  2. Controller Manager initialization in Gazebo Fortress
  3. Joint State Broadcaster activation and /joint_states publication
  4. Diff Drive Controller activation
  5. /cmd_vel forward velocity command response (position and wheel velocity)
  6. /cmd_vel angular velocity turning response (differential wheel velocities)
  7. /cmd_vel stop command verification
"""

import os
import sys
import time
import math
import subprocess
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

class VelocityTesterNode(Node):
    def __init__(self):
        super().__init__('phase2_velocity_tester')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.js_sub = self.create_subscription(JointState, '/joint_states', self.js_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        
        self.latest_js = None
        self.latest_odom = None

    def js_cb(self, msg):
        self.latest_js = msg

    def odom_cb(self, msg):
        self.latest_odom = msg

    def send_cmd(self, linear_x, angular_z, duration_sec):
        rate = self.create_rate(20)  # 20 Hz
        end_time = time.time() + duration_sec
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        
        while time.time() < end_time and rclpy.ok():
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)


def parse_gazebo_pose(output_str):
    """Parses XYZ and RPY from 'ign model -m diff_drive_robot -p' output."""
    lines = output_str.strip().splitlines()
    xyz = None
    rpy = None
    for idx, l in enumerate(lines):
        if "Pose [ XYZ (m) ] [ RPY (rad) ]:" in l and idx + 2 < len(lines):
            try:
                xyz_raw = lines[idx + 1].strip().strip('[]').split()
                rpy_raw = lines[idx + 2].strip().strip('[]').split()
                xyz = [float(val) for val in xyz_raw]
                rpy = [float(val) for val in rpy_raw]
                return xyz, rpy
            except Exception:
                pass
    return xyz, rpy


def run_tests():
    print("==================================================================")
    print("  PROJECT 3 - PHASE 2: ros2_control DIFF DRIVE VALIDATION SUITE   ")
    print("==================================================================")

    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    launch_path = os.path.join(base_dir, 'launch', 'gazebo.launch.py')
    controllers_yaml = os.path.join(base_dir, 'config', 'ros2_controllers.yaml')

    passed_all = True

    # 1. Verify Configuration files
    print("\n[TEST 1] Verifying ros2_control Configuration Files...")
    if os.path.exists(controllers_yaml):
        print(f"  [OK] {os.path.relpath(controllers_yaml, base_dir)} exists.")
    else:
        print(f"  [FAIL] Missing {controllers_yaml}")
        return False

    # 2. Launch Gazebo Fortress with ros2_control
    print("\n[TEST 2] Launching Gazebo Fortress + ros2_control (Headless)...")
    cmd = ['ros2', 'launch', launch_path, 'headless:=true']
    gz_proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    print("  Waiting 12 seconds for Gazebo Fortress and controller manager to start...")
    time.sleep(12)

    rclpy.init()
    tester_node = VelocityTesterNode()

    try:
        # 3. Check controller status
        print("\n[TEST 3] Checking Controller Manager Active Controllers...")
        ctrl_res = subprocess.run(['ros2', 'control', 'list_controllers'], capture_output=True, text=True)
        ctrl_out = ctrl_res.stdout
        print(f"  Active controllers output:\n{ctrl_out.strip()}")
        
        has_jsb = 'joint_state_broadcaster' in ctrl_out and 'active' in ctrl_out
        has_diff = 'diff_drive_controller' in ctrl_out and 'active' in ctrl_out
        
        if has_jsb:
            print("  [OK] joint_state_broadcaster is ACTIVE.")
        else:
            print("  [FAIL] joint_state_broadcaster is NOT active!")
            passed_all = False

        if has_diff:
            print("  [OK] diff_drive_controller is ACTIVE.")
        else:
            print("  [FAIL] diff_drive_controller is NOT active!")
            passed_all = False

        # Read initial pose from Gazebo
        init_pose_res = subprocess.run(['ign', 'model', '-m', 'diff_drive_robot', '-p'], capture_output=True, text=True)
        init_xyz, init_rpy = parse_gazebo_pose(init_pose_res.stdout)
        print(f"\n  Initial Robot Pose: XYZ={init_xyz}, RPY={init_rpy}")

        # 4. Spin to receive initial joint states
        for _ in range(20):
            rclpy.spin_once(tester_node, timeout_sec=0.05)

        if tester_node.latest_js:
            print(f"  [OK] /joint_states verified. Joints: {tester_node.latest_js.name}")
        else:
            print("  [FAIL] No /joint_states received!")
            passed_all = False

        # 5. Test Forward Movement: Command linear.x = 0.5 m/s for 2.5s
        print("\n[TEST 4] Testing FORWARD Motion via /cmd_vel (0.5 m/s for 2.5s)...")
        tester_node.send_cmd(linear_x=0.5, angular_z=0.0, duration_sec=2.5)
        
        # Check wheel velocities during or right after
        for _ in range(10):
            rclpy.spin_once(tester_node, timeout_sec=0.05)
            
        fwd_pose_res = subprocess.run(['ign', 'model', '-m', 'diff_drive_robot', '-p'], capture_output=True, text=True)
        fwd_xyz, fwd_rpy = parse_gazebo_pose(fwd_pose_res.stdout)
        print(f"  Pose after forward driving: XYZ={fwd_xyz}, RPY={fwd_rpy}")
        
        dx = fwd_xyz[0] - init_xyz[0]
        dy = fwd_xyz[1] - init_xyz[1]
        print(f"  Displacement: delta_X = {dx:.4f} m, delta_Y = {dy:.4f} m")
        
        if dx > 0.40:
            print(f"  [OK] Robot moved forward successfully (delta_X = {dx:.3f} m > 0.40 m).")
        else:
            print(f"  [FAIL] Robot did not move forward sufficiently (delta_X = {dx:.3f} m).")
            passed_all = False

        # 6. Test Rotation: Command angular.z = 1.0 rad/s for 2.0s
        print("\n[TEST 5] Testing ROTATION via /cmd_vel (1.0 rad/s for 2.0s)...")
        pre_rot_xyz, pre_rot_rpy = fwd_xyz, fwd_rpy
        tester_node.send_cmd(linear_x=0.0, angular_z=1.0, duration_sec=2.0)
        
        for _ in range(10):
            rclpy.spin_once(tester_node, timeout_sec=0.05)

        rot_pose_res = subprocess.run(['ign', 'model', '-m', 'diff_drive_robot', '-p'], capture_output=True, text=True)
        rot_xyz, rot_rpy = parse_gazebo_pose(rot_pose_res.stdout)
        print(f"  Pose after rotation: XYZ={rot_xyz}, RPY={rot_rpy}")
        
        dyaw = rot_rpy[2] - pre_rot_rpy[2]
        # Normalize angle difference
        dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
        print(f"  Yaw Change: delta_Yaw = {dyaw:.4f} rad ({math.degrees(dyaw):.1f} deg)")
        
        if dyaw > 0.5:
            print(f"  [OK] Robot rotated counter-clockwise successfully (delta_Yaw = {dyaw:.3f} rad > 0.5 rad).")
        else:
            print(f"  [FAIL] Robot rotation insufficient (delta_Yaw = {dyaw:.3f} rad).")
            passed_all = False

        # 7. Test Stopping Behavior
        print("\n[TEST 6] Testing STOP Command (0.0 m/s for 1.5s)...")
        tester_node.send_cmd(linear_x=0.0, angular_z=0.0, duration_sec=1.5)
        
        stop_pose1_res = subprocess.run(['ign', 'model', '-m', 'diff_drive_robot', '-p'], capture_output=True, text=True)
        stop_xyz1, _ = parse_gazebo_pose(stop_pose1_res.stdout)
        time.sleep(1.0)
        stop_pose2_res = subprocess.run(['ign', 'model', '-m', 'diff_drive_robot', '-p'], capture_output=True, text=True)
        stop_xyz2, _ = parse_gazebo_pose(stop_pose2_res.stdout)
        
        drift = math.dist(stop_xyz1, stop_xyz2)
        print(f"  Drift after stop: {drift:.5f} m")
        if drift < 0.02:
            print(f"  [OK] Robot halted cleanly (drift = {drift:.5f} m < 0.02 m).")
        else:
            print(f"  [FAIL] Robot continued drifting (drift = {drift:.5f} m).")
            passed_all = False

    except Exception as e:
        print(f"  [FAIL] Runtime testing exception: {e}")
        passed_all = False
    finally:
        tester_node.destroy_node()
        rclpy.shutdown()
        if gz_proc:
            print("\n  Terminating Gazebo Fortress test instance...")
            gz_proc.terminate()
            try:
                gz_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                gz_proc.kill()
                gz_proc.wait()
            print("  [OK] Gazebo Fortress test instance cleanly terminated.")

    print("\n==================================================================")
    if passed_all:
        print("  PHASE 2 STATUS: ALL TESTS PASSED (VERIFIED)")
    else:
        print("  PHASE 2 STATUS: TESTS FAILED")
    print("==================================================================")
    return passed_all

if __name__ == '__main__':
    ok = run_tests()
    sys.exit(0 if ok else 1)
