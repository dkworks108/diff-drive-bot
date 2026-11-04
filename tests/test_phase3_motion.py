#!/usr/bin/env python3
"""
Phase 3 Validation Suite: Comprehensive Motion Validation using Simulation Time

Tests 9 motion maneuvers:
  1. Forward movement
  2. Reverse movement
  3. Clockwise rotation
  4. Counterclockwise rotation
  5. In-place rotation (symmetric opposite wheel velocities)
  6. Circular trajectory
  7. Combined linear and angular velocity
  8. Stop command (active braking)
  9. Command timeout / stopped-command safety (watchdog expiration)

All timing is regulated against simulation clock (/clock), not wall-clock delays.
"""

import os
import sys
import time
import math
import subprocess
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


class SimMotionController(Node):
    def __init__(self):
        super().__init__('phase3_motion_controller')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.js_sub = self.create_subscription(JointState, '/joint_states', self.js_cb, 10)

        self.sim_time = None
        self.latest_odom = None
        self.latest_js = None

    def clock_cb(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def odom_cb(self, msg):
        self.latest_odom = msg

    def js_cb(self, msg):
        self.latest_js = msg

    def wait_for_data(self, timeout_sec=15.0):
        start = time.time()
        while (self.sim_time is None or self.latest_odom is None or self.latest_js is None) and (time.time() - start < timeout_sec):
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.sim_time is not None and self.latest_odom is not None and self.latest_js is not None

    def get_odom_pose(self):
        p = self.latest_odom.pose.pose.position
        q = self.latest_odom.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return p.x, p.y, yaw

    def get_wheel_velocities(self):
        if not self.latest_js or 'left_wheel_joint' not in self.latest_js.name:
            return 0.0, 0.0
        l_idx = self.latest_js.name.index('left_wheel_joint')
        r_idx = self.latest_js.name.index('right_wheel_joint')
        return self.latest_js.velocity[l_idx], self.latest_js.velocity[r_idx]

    def execute_motion_sim_time(self, linear_x, angular_z, duration_sec, active_stop=True):
        """Executes a command for an exact duration measured in simulation time."""
        while self.sim_time is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

        start_t = self.sim_time
        target_end_t = start_t + duration_sec
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)

        recorded_w_l = []
        recorded_w_r = []

        # Stream command at ~20 Hz simulation-time loop
        while self.sim_time < target_end_t and rclpy.ok():
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.02)
            wl, wr = self.get_wheel_velocities()
            recorded_w_l.append(wl)
            recorded_w_r.append(wr)

        actual_duration = self.sim_time - start_t

        if active_stop:
            stop_twist = Twist()
            stop_end_t = self.sim_time + 0.3
            while self.sim_time < stop_end_t and rclpy.ok():
                self.cmd_pub.publish(stop_twist)
                rclpy.spin_once(self, timeout_sec=0.02)

        # Allow readings to settle
        settle_end_t = self.sim_time + 0.2
        while self.sim_time < settle_end_t and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

        avg_wl = sum(recorded_w_l) / len(recorded_w_l) if recorded_w_l else 0.0
        avg_wr = sum(recorded_w_r) / len(recorded_w_r) if recorded_w_r else 0.0

        return actual_duration, avg_wl, avg_wr


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


def angle_diff(a, b):
    d = a - b
    return math.atan2(math.sin(d), math.cos(d))


def run_tests():
    print("==================================================================")
    print("  PROJECT 3 - PHASE 3: COMPREHENSIVE MOTION VALIDATION SUITE      ")
    print("==================================================================")

    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    launch_path = os.path.join(base_dir, 'launch', 'gazebo.launch.py')

    cmd = ['ros2', 'launch', launch_path, 'headless:=true']
    gz_proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print("Launching Gazebo Fortress + ros2_control...")
    time.sleep(11)

    rclpy.init()
    ctrl = SimMotionController()
    if not ctrl.wait_for_data():
        print("FAIL: Timeout waiting for clock / odom / joint_states")
        gz_proc.kill()
        return False

    results = []

    try:
        # 1. Forward Movement
        print("\n[TEST 1] Forward Movement (linear.x = +0.4 m/s, 2.0s sim-time)...")
        x0, y0, yaw0 = ctrl.get_odom_pose()
        dur, wl, wr = ctrl.execute_motion_sim_time(0.4, 0.0, 2.0)
        x1, y1, yaw1 = ctrl.get_odom_pose()
        dx = x1 - x0
        print(f"  Duration: {dur:.3f}s | Delta X: {dx:.4f}m | Wheel Vels: L={wl:.2f}, R={wr:.2f} rad/s")
        passed = (dx > 0.65) and (wl > 5.0) and (wr > 5.0)
        results.append(("1. Forward Movement", passed, f"dx={dx:.3f}m"))

        # 2. Reverse Movement
        print("\n[TEST 2] Reverse Movement (linear.x = -0.4 m/s, 2.0s sim-time)...")
        x0, y0, yaw0 = ctrl.get_odom_pose()
        dur, wl, wr = ctrl.execute_motion_sim_time(-0.4, 0.0, 2.0)
        x1, y1, yaw1 = ctrl.get_odom_pose()
        dx = x1 - x0
        print(f"  Duration: {dur:.3f}s | Delta X: {dx:.4f}m | Wheel Vels: L={wl:.2f}, R={wr:.2f} rad/s")
        passed = (dx < -0.65) and (wl < -5.0) and (wr < -5.0)
        results.append(("2. Reverse Movement", passed, f"dx={dx:.3f}m"))

        # 3. Clockwise Rotation
        print("\n[TEST 3] Clockwise Rotation (angular.z = -1.0 rad/s, 1.5s sim-time)...")
        x0, y0, yaw0 = ctrl.get_odom_pose()
        dur, wl, wr = ctrl.execute_motion_sim_time(0.0, -1.0, 1.5)
        x1, y1, yaw1 = ctrl.get_odom_pose()
        dyaw = angle_diff(yaw1, yaw0)
        print(f"  Duration: {dur:.3f}s | Delta Yaw: {dyaw:.4f} rad ({math.degrees(dyaw):.1f} deg)")
        passed = (dyaw < -1.0) and (wl > 0.0) and (wr < 0.0)
        results.append(("3. Clockwise Rotation", passed, f"dyaw={dyaw:.3f} rad"))

        # 4. Counterclockwise Rotation
        print("\n[TEST 4] Counterclockwise Rotation (angular.z = +1.0 rad/s, 1.5s sim-time)...")
        x0, y0, yaw0 = ctrl.get_odom_pose()
        dur, wl, wr = ctrl.execute_motion_sim_time(0.0, 1.0, 1.5)
        x1, y1, yaw1 = ctrl.get_odom_pose()
        dyaw = angle_diff(yaw1, yaw0)
        print(f"  Duration: {dur:.3f}s | Delta Yaw: {dyaw:.4f} rad ({math.degrees(dyaw):.1f} deg)")
        passed = (dyaw > 1.0) and (wl < 0.0) and (wr > 0.0)
        results.append(("4. Counterclockwise Rotation", passed, f"dyaw={dyaw:.3f} rad"))

        # 5. In-Place Rotation (Pure rotation about geometric center)
        print("\n[TEST 5] In-Place Rotation (verify zero translational drift)...")
        x0, y0, yaw0 = ctrl.get_odom_pose()
        dur, wl, wr = ctrl.execute_motion_sim_time(0.0, 0.8, 2.0)
        x1, y1, yaw1 = ctrl.get_odom_pose()
        trans_disp = math.hypot(x1 - x0, y1 - y0)
        dyaw = angle_diff(yaw1, yaw0)
        print(f"  Translational Drift: {trans_disp:.4f}m | Delta Yaw: {dyaw:.4f} rad | Symmetry: wl+wr = {wl+wr:.3f}")
        passed = (trans_disp < 0.05) and (abs(dyaw) > 1.0) and (abs(wl + wr) < 0.5)
        results.append(("5. In-Place Rotation", passed, f"drift={trans_disp:.4f}m, dyaw={dyaw:.3f}rad"))

        # 6. Circular Trajectory
        print("\n[TEST 6] Circular Trajectory (linear.x = 0.3 m/s, angular.z = 0.6 rad/s, 3.0s sim-time)...")
        x0, y0, yaw0 = ctrl.get_odom_pose()
        dur, wl, wr = ctrl.execute_motion_sim_time(0.3, 0.6, 3.0)
        x1, y1, yaw1 = ctrl.get_odom_pose()
        dist_moved = math.hypot(x1 - x0, y1 - y0)
        dyaw = angle_diff(yaw1, yaw0)
        print(f"  Distance Moved: {dist_moved:.4f}m | Delta Yaw: {dyaw:.4f} rad")
        passed = (dist_moved > 0.4) and (abs(dyaw) > 1.0)
        results.append(("6. Circular Trajectory", passed, f"dist={dist_moved:.3f}m, dyaw={dyaw:.3f}rad"))

        # 7. Combined Linear and Angular Velocity
        print("\n[TEST 7] Combined Motion (linear.x = 0.35 m/s, angular.z = -0.5 rad/s, 2.0s sim-time)...")
        x0, y0, yaw0 = ctrl.get_odom_pose()
        dur, wl, wr = ctrl.execute_motion_sim_time(0.35, -0.5, 2.0)
        x1, y1, yaw1 = ctrl.get_odom_pose()
        dist_moved = math.hypot(x1 - x0, y1 - y0)
        dyaw = angle_diff(yaw1, yaw0)
        print(f"  Distance Moved: {dist_moved:.4f}m | Delta Yaw: {dyaw:.4f} rad")
        passed = (dist_moved > 0.4) and (dyaw < -0.5)
        results.append(("7. Combined Motion", passed, f"dist={dist_moved:.3f}m, dyaw={dyaw:.3f}rad"))

        # 8. Stop Command (Active Braking)
        print("\n[TEST 8] Stop Command (Commanding zero velocity)...")
        # Drive briefly
        ctrl.execute_motion_sim_time(0.4, 0.0, 1.0, active_stop=True)
        # Check drift over 1.0 second of simulation time
        start_stop_t = ctrl.sim_time
        x0, y0, _ = ctrl.get_odom_pose()
        while ctrl.sim_time < start_stop_t + 1.0 and rclpy.ok():
            rclpy.spin_once(ctrl, timeout_sec=0.02)
        x1, y1, _ = ctrl.get_odom_pose()
        drift = math.hypot(x1 - x0, y1 - y0)
        wl, wr = ctrl.get_wheel_velocities()
        print(f"  Post-Stop Drift: {drift:.5f}m | Wheel Vels: L={wl:.5f}, R={wr:.5f}")
        passed = (drift < 0.005) and (abs(wl) < 0.01) and (abs(wr) < 0.01)
        results.append(("8. Active Stop", passed, f"drift={drift:.5f}m"))

        # 9. Command Timeout Safety (Watchdog)
        print("\n[TEST 9] Command Timeout Safety (Cease publishing, watchdog halts wheels after 0.5s)...")
        # Send 1 command without following up
        twist = Twist()
        twist.linear.x = 0.4
        ctrl.cmd_pub.publish(twist)
        
        # Wait 1.0s of sim time (cmd_vel_timeout = 0.5s)
        t_expire = ctrl.sim_time + 1.0
        while ctrl.sim_time < t_expire and rclpy.ok():
            rclpy.spin_once(ctrl, timeout_sec=0.02)

        wl, wr = ctrl.get_wheel_velocities()
        print(f"  Wheel Vels after timeout: L={wl:.5f}, R={wr:.5f}")
        passed = (abs(wl) < 0.05) and (abs(wr) < 0.05)
        results.append(("9. Watchdog Timeout", passed, f"wl={wl:.4f}, wr={wr:.4f}"))

    finally:
        ctrl.destroy_node()
        rclpy.shutdown()
        gz_proc.terminate()
        try:
            gz_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            gz_proc.kill()
            gz_proc.wait()

    print("\n==================================================================")
    print("  PHASE 3 MOTION VALIDATION SUMMARY")
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
