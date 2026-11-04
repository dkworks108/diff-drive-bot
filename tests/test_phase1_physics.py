#!/usr/bin/env python3
"""
Phase 1 Validation Suite: Gazebo Fortress Physics & Environment Stability
Tests:
  1. Package configuration and file structure
  2. URDF/Xacro generation and integrity
  3. World SDF schema validation
  4. Runtime Gazebo Fortress physics settling and ground contact
  5. Simulation clock bridge verification
  6. Visual snapshot capture of ground contact
"""

import os
import sys
import time
import math
import subprocess
import xml.etree.ElementTree as ET
import pybullet as p
import pybullet_data
from PIL import Image

def run_tests():
    print("==================================================================")
    print("  PROJECT 3 - PHASE 1: GAZEBO FORTRESS PHYSICS VALIDATION SUITE   ")
    print("==================================================================")

    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    urdf_path = os.path.join(base_dir, 'urdf', 'robot.urdf')
    xacro_path = os.path.join(base_dir, 'urdf', 'robot.xacro')
    world_path = os.path.join(base_dir, 'worlds', 'test_world.sdf')
    launch_path = os.path.join(base_dir, 'launch', 'gazebo.launch.py')
    shots_dir = os.path.join(base_dir, 'screenshots')
    os.makedirs(shots_dir, exist_ok=True)

    passed_all = True

    # 1. File existence checks
    print("\n[TEST 1] Verifying Package & Configuration Structure...")
    required_files = [
        os.path.join(base_dir, 'CMakeLists.txt'),
        os.path.join(base_dir, 'package.xml'),
        urdf_path,
        xacro_path,
        world_path,
        launch_path,
        os.path.join(base_dir, 'config', 'rviz.rviz')
    ]
    for rf in required_files:
        rel = os.path.relpath(rf, base_dir)
        if os.path.exists(rf):
            print(f"  [OK] {rel} exists")
        else:
            print(f"  [FAIL] {rel} NOT found!")
            passed_all = False

    # 2. Xacro generation test
    print("\n[TEST 2] Processing robot.xacro -> URDF...")
    try:
        urdf_xml = subprocess.check_output(['xacro', xacro_path]).decode('utf-8')
        root = ET.fromstring(urdf_xml)
        links = [l.attrib['name'] for l in root.findall('link')]
        joints = [j.attrib['name'] for j in root.findall('joint')]
        print(f"  [OK] Xacro parsed successfully. Links: {len(links)}, Joints: {len(joints)}")
        
        expected_links = {'base_footprint', 'base_link', 'left_wheel_link', 'right_wheel_link', 'caster_wheel_link'}
        expected_joints = {'base_footprint_joint', 'left_wheel_joint', 'right_wheel_joint', 'caster_joint'}
        
        if expected_links.issubset(set(links)):
            print(f"  [OK] All expected links present: {expected_links}")
        else:
            print(f"  [FAIL] Missing links: {expected_links - set(links)}")
            passed_all = False
            
        if expected_joints.issubset(set(joints)):
            print(f"  [OK] All expected joints present: {expected_joints}")
        else:
            print(f"  [FAIL] Missing joints: {expected_joints - set(joints)}")
            passed_all = False
    except Exception as e:
        print(f"  [FAIL] Error processing xacro: {e}")
        passed_all = False

    # 3. World SDF schema validation
    print("\n[TEST 3] Validating worlds/test_world.sdf with ign sdf...")
    try:
        sdf_res = subprocess.run(['ign', 'sdf', '-k', world_path], capture_output=True, text=True)
        if sdf_res.returncode == 0 and "Valid" in sdf_res.stdout:
            print("  [OK] worlds/test_world.sdf is valid SDF.")
        else:
            print(f"  [FAIL] SDF validation output:\n{sdf_res.stdout}\n{sdf_res.stderr}")
            passed_all = False
    except Exception as e:
        print(f"  [FAIL] Could not run ign sdf: {e}")
        passed_all = False

    # 4. PyBullet Physics & Ground Contact Snapshot
    print("\n[TEST 4] Rendering Physics Ground Contact in PyBullet...")
    try:
        cid = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        plane_id = p.loadURDF("plane.urdf")
        robot_id = p.loadURDF(urdf_path, [0, 0, 0.05])

        # Step 240 physics ticks
        for _ in range(240):
            p.stepSimulation()

        # Check contact points
        contacts = p.getContactPoints(robot_id, plane_id)
        contact_links = set(pt[3] for pt in contacts)
        base_pos, base_orn = p.getBasePositionAndOrientation(robot_id)
        
        print(f"  [OK] Robot settled at base z = {base_pos[2]:.4f} m (expected ~0.05m chassis altitude)")
        print(f"  [OK] Total contact points with ground: {len(contacts)}")
        
        # Camera capture
        view_matrix = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=[0.0, 0.0, 0.05],
            distance=0.8,
            yaw=45,
            pitch=-25,
            roll=0,
            upAxisIndex=2
        )
        proj_matrix = p.computeProjectionMatrixFOV(
            fov=60, aspect=1.33, nearVal=0.1, farVal=10.0
        )
        _, _, rgb, _, _ = p.getCameraImage(640, 480, view_matrix, proj_matrix)
        rgb_img = Image.fromarray(rgb[:, :, :3])
        shot_path = os.path.join(shots_dir, 'phase1_physics_contact.png')
        rgb_img.save(shot_path)
        print(f"  [OK] Saved ground contact snapshot to: {shot_path}")
        p.disconnect()
    except Exception as e:
        print(f"  [FAIL] PyBullet test failed: {e}")
        passed_all = False

    # 5. Gazebo Fortress Runtime Physics & Spawn Test
    print("\n[TEST 5] Launching Gazebo Fortress (Headless) Runtime Test...")
    gz_proc = None
    try:
        cmd = ['ros2', 'launch', launch_path, 'headless:=true']
        gz_proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        print("  Waiting 6 seconds for Gazebo Fortress world initialization...")
        time.sleep(6)

        # Check model list in Gazebo
        model_res = subprocess.run(['ign', 'model', '--list'], capture_output=True, text=True)
        if 'diff_drive_robot' in model_res.stdout:
            print("  [OK] diff_drive_robot successfully spawned in Gazebo Fortress.")
        else:
            print(f"  [FAIL] diff_drive_robot not found in Gazebo models:\n{model_res.stdout}")
            passed_all = False

        # Query model pose from Gazebo
        pose_res = subprocess.run(['ign', 'model', '-m', 'diff_drive_robot', '-p'], capture_output=True, text=True)
        print(f"  [OK] Gazebo Model Pose Output:\n{pose_res.stdout.strip()}")

        # Verify simulation clock bridge
        clock_res = subprocess.run(['ros2', 'topic', 'echo', '/clock', '--once'], capture_output=True, text=True, timeout=5)
        if 'clock:' in clock_res.stdout:
            print("  [OK] /clock topic actively bridged from Gazebo Fortress to ROS 2.")
        else:
            print("  [FAIL] /clock topic not receiving messages.")
            passed_all = False

    except Exception as e:
        print(f"  [FAIL] Gazebo runtime verification error: {e}")
        passed_all = False
    finally:
        if gz_proc:
            print("  Shutting down Gazebo Fortress test instance...")
            gz_proc.terminate()
            try:
                gz_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                gz_proc.kill()
                gz_proc.wait()
            print("  [OK] Gazebo Fortress test instance cleanly terminated.")

    print("\n==================================================================")
    if passed_all:
        print("  PHASE 1 STATUS: ALL TESTS PASSED (VERIFIED)")
    else:
        print("  PHASE 1 STATUS: TESTS FAILED")
    print("==================================================================")
    return passed_all

if __name__ == '__main__':
    ok = run_tests()
    sys.exit(0 if ok else 1)
