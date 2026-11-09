#!/usr/bin/env python3
"""
Unit tests for FrontierDetector algorithmic engine.
Validates all 14 algorithmic test cases using synthetic occupancy grids.
"""

import math
import numpy as np
import pytest

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
    """Create a synthetic occupancy grid dictionary."""
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
    """Set a rectangular region in the synthetic grid."""
    w = grid_dict['width']
    for y in range(y1, y2):
        for x in range(x1, x2):
            grid_dict['data'][y * w + x] = val


class TestFrontierDetector:

    def setup_method(self):
        self.detector = FrontierDetector(
            min_cluster_size=5,
            obstacle_safety_distance=0.20,
            min_distance_to_goal=0.3,
            max_distance_to_goal=10.0,
            blacklist_radius=0.5,
            blacklist_duration=30.0
        )

    # 1. Frontier cell detection
    def test_frontier_cell_detection(self):
        """Free space bordered by unknown space produces valid frontier cells."""
        grid = create_synthetic_grid(30, 30, resolution=0.05, default_val=-1)
        # 10x10 free space in center (indices 10..20)
        set_rect(grid, 10, 10, 20, 20, val=0)

        goals, clusters = self.detector.detect_and_rank(grid, robot_pose=(0.0, 0.0), current_time=0.0)
        assert len(clusters) > 0, "Should detect frontier clusters around free box perimeter"
        total_cells = sum(c.size for c in clusters)
        assert total_cells >= 10, f"Expected perimeter frontier cells, got {total_cells}"

    # 2. Multiple frontier clusters
    def test_multiple_frontier_clusters(self):
        """Two separated free corridors produce two distinct clusters."""
        grid = create_synthetic_grid(40, 40, resolution=0.05, default_val=-1)
        # Corridor 1 on left
        set_rect(grid, 5, 5, 15, 15, val=0)
        # Corridor 2 on right
        set_rect(grid, 25, 25, 35, 35, val=0)

        goals, clusters = self.detector.detect_and_rank(grid, robot_pose=(0.0, 0.0), current_time=0.0)
        assert len(clusters) >= 2, f"Expected at least 2 distinct clusters, got {len(clusters)}"

    # 3. Minimum cluster filtering
    def test_min_cluster_filtering(self):
        """Small noise clusters below min_cluster_size are discarded."""
        grid = create_synthetic_grid(30, 30, resolution=0.05, default_val=-1)
        # Tiny 2x2 free island (perimeter ~4 cells < 5)
        set_rect(grid, 5, 5, 7, 7, val=0)

        goals, clusters = self.detector.detect_and_rank(grid, robot_pose=(0.0, 0.0), current_time=0.0)
        assert len(clusters) == 0, "Tiny cluster (<5 cells) must be filtered out"

    # 4. Occupied-cell rejection
    def test_occupied_cell_rejection(self):
        """Frontier cells must never be occupied, and cells near obstacles are rejected."""
        grid = create_synthetic_grid(30, 30, resolution=0.05, default_val=-1)
        set_rect(grid, 10, 10, 25, 25, val=0)
        # Add a wall along the right edge of free space
        set_rect(grid, 24, 10, 25, 25, val=100)

        goals, clusters = self.detector.detect_and_rank(grid, robot_pose=(0.0, 0.0), current_time=0.0)
        for cluster in clusters:
            for (gx, gy) in cluster.cells:
                idx = gy * grid['width'] + gx
                assert grid['data'][idx] == 0, "Frontier cell must be FREE, not OCCUPIED or UNKNOWN"

    # 5. Non-convex frontier clusters
    def test_non_convex_frontier_clusters(self):
        """U-shaped frontier cluster extracts a safe goal without landing in unknown center."""
        grid = create_synthetic_grid(40, 40, resolution=0.05, default_val=-1)
        # U-shaped free corridor around unknown center
        set_rect(grid, 10, 10, 15, 30, val=0)  # Left leg
        set_rect(grid, 10, 10, 30, 15, val=0)  # Bottom base
        set_rect(grid, 25, 10, 30, 30, val=0)  # Right leg
        # Center (15..25, 15..30) remains -1 (unknown)

        goals, clusters = self.detector.detect_and_rank(grid, robot_pose=(0.0, 0.0), current_time=0.0)
        assert len(goals) > 0, "Must extract at least one goal for non-convex cluster"
        for g in goals:
            idx = g.grid_y * grid['width'] + g.grid_x
            assert grid['data'][idx] == 0, f"Goal at grid ({g.grid_x}, {g.grid_y}) must be in free space!"

    # 6. Safe goal extraction
    def test_safe_goal_extraction(self):
        """Goal must have clearance >= obstacle_safety_distance."""
        grid = create_synthetic_grid(40, 40, resolution=0.05, default_val=-1)
        set_rect(grid, 10, 10, 30, 30, val=0)
        set_rect(grid, 10, 10, 11, 30, val=100)  # Left obstacle wall

        goals, clusters = self.detector.detect_and_rank(grid, robot_pose=(0.0, 0.0), current_time=0.0)
        assert len(goals) > 0
        for g in goals:
            assert g.obstacle_distance >= self.detector.obstacle_safety_distance - 1e-4

    # 7. Goal ranking
    def test_goal_ranking(self):
        """Closer large clusters rank higher than distant small clusters."""
        grid = create_synthetic_grid(60, 60, resolution=0.05, default_val=-1)
        # Cluster 1: Close to robot (10..22, 10..22)
        set_rect(grid, 10, 10, 22, 22, val=0)
        # Cluster 2: Far from robot (45..55, 45..55)
        set_rect(grid, 45, 45, 55, 55, val=0)

        # Robot at grid (15, 15) -> world (0.75, 0.75) if origin 0
        grid['origin_x'] = 0.0
        grid['origin_y'] = 0.0
        rx = 15 * 0.05
        ry = 15 * 0.05

        goals, clusters = self.detector.detect_and_rank(grid, robot_pose=(rx, ry), current_time=0.0)
        assert len(goals) >= 2
        # First ranked goal should be closer
        assert goals[0].distance_to_robot < goals[-1].distance_to_robot

    # 8. Failed-goal blacklisting
    def test_failed_goal_blacklisting(self):
        """Blacklisted destinations are excluded from ranked goals."""
        grid = create_synthetic_grid(40, 40, resolution=0.05, default_val=-1)
        set_rect(grid, 15, 15, 30, 30, val=0)

        goals, _ = self.detector.detect_and_rank(grid, robot_pose=(0.0, 0.0), current_time=0.0)
        assert len(goals) > 0
        top_goal = goals[0]

        # Blacklist top goal
        self.detector.add_to_blacklist(top_goal.x, top_goal.y, current_time=1.0)
        assert self.detector.is_blacklisted(top_goal.x, top_goal.y, current_time=2.0)

        # Re-detect: top goal must not appear as first choice
        new_goals, _ = self.detector.detect_and_rank(grid, robot_pose=(0.0, 0.0), current_time=2.0)
        for g in new_goals:
            dist = math.hypot(g.x - top_goal.x, g.y - top_goal.y)
            assert dist > self.detector.blacklist_radius, "Blacklisted goal location must be excluded"

    # 9. Blacklist expiry
    def test_blacklist_expiry(self):
        """Blacklist entry expires after blacklist_duration."""
        self.detector.add_to_blacklist(1.0, 2.0, current_time=10.0)
        assert self.detector.is_blacklisted(1.0, 2.0, current_time=20.0)
        # After duration (30s) -> t=41s
        assert not self.detector.is_blacklisted(1.0, 2.0, current_time=45.0)

    # 10. No-frontier handling
    def test_no_frontier_handling(self):
        """Fully explored or fully occupied map returns empty goal list cleanly."""
        grid = create_synthetic_grid(20, 20, resolution=0.05, default_val=0)  # All free, no unknown
        goals, clusters = self.detector.detect_and_rank(grid, robot_pose=(0.0, 0.0), current_time=0.0)
        assert len(goals) == 0
        assert len(clusters) == 0

    # 11. Temporary unreachable frontier handling
    def test_unreachable_frontier_blacklisting_recovery(self):
        """When all current frontiers are blacklisted, clear or retry allows recovery."""
        grid = create_synthetic_grid(30, 30, resolution=0.05, default_val=-1)
        set_rect(grid, 10, 10, 20, 20, val=0)

        goals, _ = self.detector.detect_and_rank(grid, robot_pose=(0.0, 0.0), current_time=0.0)
        assert len(goals) > 0
        for g in goals:
            self.detector.add_to_blacklist(g.x, g.y, current_time=0.0)

        # All blacklisted -> goals empty
        goals_bl, _ = self.detector.detect_and_rank(grid, robot_pose=(0.0, 0.0), current_time=5.0)
        assert len(goals_bl) == 0

        # After clear or expiry, goals become available again
        self.detector.clear_blacklist()
        goals_rec, _ = self.detector.detect_and_rank(grid, robot_pose=(0.0, 0.0), current_time=5.0)
        assert len(goals_rec) > 0

    # 12. Grid-to-world coordinate conversion
    def test_grid_to_world_conversion(self):
        """Mathematical roundtrip between grid and world frames."""
        origin_x = -5.0
        origin_y = -3.0
        res = 0.05
        gx, gy = 120, 80

        wx, wy = self.detector.grid_to_world(gx, gy, origin_x, origin_y, res)
        rgx, rgy = self.detector.world_to_grid(wx, wy, origin_x, origin_y, res)

        assert gx == rgx and gy == rgy, f"Grid ({gx},{gy}) -> World ({wx:.3f},{wy:.3f}) -> Grid ({rgx},{rgy})"

    # 13. Map origin and resolution handling
    def test_map_origin_and_resolution(self):
        """Handles arbitrary resolutions and negative origins accurately."""
        res = 0.10
        ox, oy = -10.0, -8.0
        grid = create_synthetic_grid(50, 50, resolution=res, origin_x=ox, origin_y=oy, default_val=-1)
        set_rect(grid, 20, 20, 30, 30, val=0)

        goals, _ = self.detector.detect_and_rank(grid, robot_pose=(ox, oy), current_time=0.0)
        assert len(goals) > 0
        for g in goals:
            assert g.x >= ox and g.y >= oy

    # 14. Invalid occupancy grids
    def test_invalid_occupancy_grids(self):
        """Handles None, empty, or mismatched dimension grids gracefully without exceptions."""
        assert self.detector.detect_and_rank(None, (0, 0), 0.0) == ([], [])
        assert self.detector.detect_and_rank({}, (0, 0), 0.0) == ([], [])
        bad_grid = {'width': 10, 'height': 10, 'resolution': 0.05, 'data': [0] * 50}  # 50 != 100
        assert self.detector.detect_and_rank(bad_grid, (0, 0), 0.0) == ([], [])


if __name__ == '__main__':
    pytest.main(['-v', __file__])

