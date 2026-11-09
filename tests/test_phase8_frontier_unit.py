#!/usr/bin/env python3
"""
Phase 8 Unit Test Suite: Frontier Detection & Exploration Planning Algorithms.

Runs all 14 algorithmic test cases:
  1. Frontier cell detection
  2. Multiple frontier clusters
  3. Minimum cluster filtering
  4. Occupied-cell rejection
  5. Non-convex frontier clusters
  6. Safe goal extraction
  7. Goal ranking
  8. Failed-goal blacklisting
  9. Blacklist expiry
  10. No-frontier handling
  11. Temporary unreachable frontier handling
  12. Grid-to-world coordinate conversion
  13. Map origin and resolution handling
  14. Invalid occupancy grids
"""

import sys
import os
import unittest
import math

# Add package directory to sys.path
proj_dir = "/media/bumble/DATA_DISK/robotics/projects/03_diff_drive_robot"
fe_dir = os.path.join(proj_dir, "frontier_exploration")
if fe_dir not in sys.path:
    sys.path.insert(0, fe_dir)

from frontier_exploration.frontier_detector import (
    FrontierDetector,
    FrontierGoal,
    FrontierCluster,
    quaternion_to_yaw
)


def create_synthetic_grid(
    width: int = 50,
    height: int = 50,
    resolution: float = 0.05,
    origin_x: float = -1.25,
    origin_y: float = -1.25,
    default_val: int = -1
) -> dict:
    data = [default_val] * (width * height)
    return {
        'width': width,
        'height': height,
        'resolution': resolution,
        'origin_x': origin_x,
        'origin_y': origin_y,
        'origin_yaw': 0.0,
        'data': data
    }


def set_rect(grid_dict: dict, x1: int, y1: int, x2: int, y2: int, val: int):
    w = grid_dict['width']
    for y in range(y1, y2):
        for x in range(x1, x2):
            grid_dict['data'][y * w + x] = val


class TestFrontierDetectorUnit(unittest.TestCase):

    def setUp(self):
        self.detector = FrontierDetector(
            min_cluster_size=5,
            obstacle_safety_distance=0.20,
            min_distance_to_goal=0.3,
            max_distance_to_goal=10.0,
            blacklist_radius=0.5,
            blacklist_duration=30.0
        )

    def test_01_frontier_cell_detection(self):
        grid = create_synthetic_grid(30, 30, resolution=0.05, default_val=-1)
        set_rect(grid, 10, 10, 20, 20, val=0)
        goals, clusters = self.detector.detect_and_rank(grid, robot_pose=(0.0, 0.0), current_time=0.0)
        self.assertGreater(len(clusters), 0)
        total_cells = sum(c.size for c in clusters)
        self.assertGreaterEqual(total_cells, 10)

    def test_02_multiple_frontier_clusters(self):
        grid = create_synthetic_grid(40, 40, resolution=0.05, default_val=-1)
        set_rect(grid, 5, 5, 15, 15, val=0)
        set_rect(grid, 25, 25, 35, 35, val=0)
        goals, clusters = self.detector.detect_and_rank(grid, robot_pose=(0.0, 0.0), current_time=0.0)
        self.assertGreaterEqual(len(clusters), 2)

    def test_03_min_cluster_filtering(self):
        grid = create_synthetic_grid(30, 30, resolution=0.05, default_val=-1)
        set_rect(grid, 5, 5, 7, 7, val=0)
        goals, clusters = self.detector.detect_and_rank(grid, robot_pose=(0.0, 0.0), current_time=0.0)
        self.assertEqual(len(clusters), 0)

    def test_04_occupied_cell_rejection(self):
        grid = create_synthetic_grid(30, 30, resolution=0.05, default_val=-1)
        set_rect(grid, 10, 10, 25, 25, val=0)
        set_rect(grid, 24, 10, 25, 25, val=100)
        goals, clusters = self.detector.detect_and_rank(grid, robot_pose=(0.0, 0.0), current_time=0.0)
        for cluster in clusters:
            for (gx, gy) in cluster.cells:
                idx = gy * grid['width'] + gx
                self.assertEqual(grid['data'][idx], 0)

    def test_05_non_convex_frontier_clusters(self):
        grid = create_synthetic_grid(40, 40, resolution=0.05, default_val=-1)
        set_rect(grid, 10, 10, 15, 30, val=0)
        set_rect(grid, 10, 10, 30, 15, val=0)
        set_rect(grid, 25, 10, 30, 30, val=0)
        goals, clusters = self.detector.detect_and_rank(grid, robot_pose=(0.0, 0.0), current_time=0.0)
        self.assertGreater(len(goals), 0)
        for g in goals:
            idx = g.grid_y * grid['width'] + g.grid_x
            self.assertEqual(grid['data'][idx], 0)

    def test_06_safe_goal_extraction(self):
        grid = create_synthetic_grid(40, 40, resolution=0.05, default_val=-1)
        set_rect(grid, 10, 10, 30, 30, val=0)
        set_rect(grid, 10, 10, 11, 30, val=100)
        goals, clusters = self.detector.detect_and_rank(grid, robot_pose=(0.0, 0.0), current_time=0.0)
        self.assertGreater(len(goals), 0)
        for g in goals:
            self.assertGreaterEqual(g.obstacle_distance, self.detector.obstacle_safety_distance - 1e-4)

    def test_07_goal_ranking(self):
        grid = create_synthetic_grid(60, 60, resolution=0.05, default_val=-1)
        set_rect(grid, 10, 10, 22, 22, val=0)
        set_rect(grid, 45, 45, 55, 55, val=0)
        grid['origin_x'] = 0.0
        grid['origin_y'] = 0.0
        rx = 15 * 0.05
        ry = 15 * 0.05
        goals, clusters = self.detector.detect_and_rank(grid, robot_pose=(rx, ry), current_time=0.0)
        self.assertGreaterEqual(len(goals), 2)
        self.assertLess(goals[0].distance_to_robot, goals[-1].distance_to_robot)

    def test_08_failed_goal_blacklisting(self):
        grid = create_synthetic_grid(40, 40, resolution=0.05, default_val=-1)
        set_rect(grid, 15, 15, 30, 30, val=0)
        goals, _ = self.detector.detect_and_rank(grid, robot_pose=(0.0, 0.0), current_time=0.0)
        self.assertGreater(len(goals), 0)
        top_goal = goals[0]
        self.detector.add_to_blacklist(top_goal.x, top_goal.y, current_time=1.0)
        self.assertTrue(self.detector.is_blacklisted(top_goal.x, top_goal.y, current_time=2.0))
        new_goals, _ = self.detector.detect_and_rank(grid, robot_pose=(0.0, 0.0), current_time=2.0)
        for g in new_goals:
            dist = math.hypot(g.x - top_goal.x, g.y - top_goal.y)
            self.assertGreater(dist, self.detector.blacklist_radius)

    def test_09_blacklist_expiry(self):
        self.detector.add_to_blacklist(1.0, 2.0, current_time=10.0)
        self.assertTrue(self.detector.is_blacklisted(1.0, 2.0, current_time=20.0))
        self.assertFalse(self.detector.is_blacklisted(1.0, 2.0, current_time=45.0))

    def test_10_no_frontier_handling(self):
        grid = create_synthetic_grid(20, 20, resolution=0.05, default_val=0)
        goals, clusters = self.detector.detect_and_rank(grid, robot_pose=(0.0, 0.0), current_time=0.0)
        self.assertEqual(len(goals), 0)
        self.assertEqual(len(clusters), 0)

    def test_11_unreachable_frontier_blacklisting_recovery(self):
        grid = create_synthetic_grid(30, 30, resolution=0.05, default_val=-1)
        set_rect(grid, 10, 10, 20, 20, val=0)
        goals, _ = self.detector.detect_and_rank(grid, robot_pose=(0.0, 0.0), current_time=0.0)
        self.assertGreater(len(goals), 0)
        for g in goals:
            self.detector.add_to_blacklist(g.x, g.y, current_time=0.0)
        goals_bl, _ = self.detector.detect_and_rank(grid, robot_pose=(0.0, 0.0), current_time=5.0)
        self.assertEqual(len(goals_bl), 0)
        self.detector.clear_blacklist()
        goals_rec, _ = self.detector.detect_and_rank(grid, robot_pose=(0.0, 0.0), current_time=5.0)
        self.assertGreater(len(goals_rec), 0)

    def test_12_grid_to_world_conversion(self):
        origin_x = -5.0
        origin_y = -3.0
        res = 0.05
        gx, gy = 120, 80
        wx, wy = self.detector.grid_to_world(gx, gy, origin_x, origin_y, res)
        rgx, rgy = self.detector.world_to_grid(wx, wy, origin_x, origin_y, res)
        self.assertEqual(gx, rgx)
        self.assertEqual(gy, rgy)

    def test_13_map_origin_and_resolution(self):
        res = 0.10
        ox, oy = -10.0, -8.0
        grid = create_synthetic_grid(50, 50, resolution=res, origin_x=ox, origin_y=oy, default_val=-1)
        set_rect(grid, 20, 20, 30, 30, val=0)
        goals, _ = self.detector.detect_and_rank(grid, robot_pose=(ox, oy), current_time=0.0)
        self.assertGreater(len(goals), 0)
        for g in goals:
            self.assertGreaterEqual(g.x, ox)
            self.assertGreaterEqual(g.y, oy)

    def test_14_invalid_occupancy_grids(self):
        self.assertEqual(self.detector.detect_and_rank(None, (0, 0), 0.0), ([], []))
        self.assertEqual(self.detector.detect_and_rank({}, (0, 0), 0.0), ([], []))
        bad_grid = {'width': 10, 'height': 10, 'resolution': 0.05, 'data': [0] * 50}
        self.assertEqual(self.detector.detect_and_rank(bad_grid, (0, 0), 0.0), ([], []))


def run_unit_tests():
    print("=" * 70)
    print("  PROJECT 3 - PHASE 8: FRONTIER DETECTOR ALGORITHMIC UNIT TESTS")
    print("=" * 70)
    suite = unittest.TestLoader().loadTestsFromTestCase(TestFrontierDetectorUnit)
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    print("=" * 70)
    print(f"  Tests run: {result.testsRun}, Failures: {len(result.failures)}, Errors: {len(result.errors)}")
    print("=" * 70)
    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_unit_tests()
    sys.exit(0 if success else 1)

