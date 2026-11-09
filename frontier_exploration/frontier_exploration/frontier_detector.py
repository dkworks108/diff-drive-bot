#!/usr/bin/env python3
"""
Frontier Detection and Exploration Planning Engine.

This module implements:
1. Fast 8-connected frontier cell detection on nav_msgs/OccupancyGrid.
2. Obstacle-aware safety filtering via Euclidean Distance Transforms.
3. Connected-component clustering of contiguous frontier cells.
4. Safe representative goal extraction (centroid projection into collision-free space).
5. Frontier goal utility ranking balancing travel distance, cluster size, and obstacle clearance.
6. Temporal spatial blacklisting for failed or unreachable goals.
"""

import math
import os
os.environ.setdefault('PYTHONNOUSERSITE', '1')

import math
import collections
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict
import numpy as np
from scipy import ndimage


@dataclass
class FrontierGoal:
    """Represents a validated, ranked exploration destination in world coordinates."""
    x: float
    y: float
    yaw: float
    cluster_size: int
    obstacle_distance: float
    distance_to_robot: float = 0.0
    cost: float = 0.0
    grid_x: int = 0
    grid_y: int = 0


@dataclass
class FrontierCluster:
    """Represents a grouped cluster of frontier cells."""
    cells: List[Tuple[int, int]]
    size: int
    centroid_grid: Tuple[float, float]
    unknown_neighbors: List[Tuple[int, int]] = field(default_factory=list)
    goal: Optional[FrontierGoal] = None


@dataclass
class BlacklistEntry:
    """Records a failed destination and its suppression window."""
    x: float
    y: float
    added_time: float
    expiry_time: float
    failure_count: int = 1


def quaternion_to_yaw(q) -> float:
    """Extract 2D yaw angle from geometry_msgs/Quaternion or object with x,y,z,w."""
    if hasattr(q, 'x'):
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
    elif isinstance(q, (list, tuple)) and len(q) == 4:
        qx, qy, qz, qw = q[0], q[1], q[2], q[3]
    else:
        return 0.0
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class FrontierDetector:
    """
    Modular, high-performance frontier extraction and goal selection engine.
    Independent of ROS middleware for maximum testability.
    """

    def __init__(
        self,
        min_cluster_size: int = 5,
        obstacle_safety_distance: float = 0.30,
        min_distance_to_goal: float = 0.5,
        max_distance_to_goal: float = 12.0,
        weight_distance: float = 1.0,
        weight_size: float = 0.05,
        weight_clearance: float = 0.2,
        blacklist_radius: float = 0.6,
        blacklist_duration: float = 60.0,
        free_threshold: int = 25,
        occupied_threshold: int = 50
    ):
        self.min_cluster_size = min_cluster_size
        self.obstacle_safety_distance = obstacle_safety_distance
        self.min_distance_to_goal = min_distance_to_goal
        self.max_distance_to_goal = max_distance_to_goal
        self.weight_distance = weight_distance
        self.weight_size = weight_size
        self.weight_clearance = weight_clearance
        self.blacklist_radius = blacklist_radius
        self.blacklist_duration = blacklist_duration
        self.free_threshold = free_threshold
        self.occupied_threshold = occupied_threshold

        # Spatial blacklist: list of BlacklistEntry
        self.blacklist: List[BlacklistEntry] = []

    def grid_to_world(
        self,
        gx: float,
        gy: float,
        origin_x: float,
        origin_y: float,
        resolution: float,
        origin_yaw: float = 0.0
    ) -> Tuple[float, float]:
        """Convert continuous/discrete grid coordinates to global world frame coordinates."""
        dx = (gx + 0.5) * resolution
        dy = (gy + 0.5) * resolution
        cos_yaw = math.cos(origin_yaw)
        sin_yaw = math.sin(origin_yaw)
        wx = origin_x + (cos_yaw * dx - sin_yaw * dy)
        wy = origin_y + (sin_yaw * dx + cos_yaw * dy)
        return wx, wy

    def world_to_grid(
        self,
        wx: float,
        wy: float,
        origin_x: float,
        origin_y: float,
        resolution: float,
        origin_yaw: float = 0.0
    ) -> Tuple[int, int]:
        """Convert global world coordinates to discrete grid cell indices."""
        dx = wx - origin_x
        dy = wy - origin_y
        cos_yaw = math.cos(-origin_yaw)
        sin_yaw = math.sin(-origin_yaw)
        gx = int(math.floor((cos_yaw * dx - sin_yaw * dy) / resolution))
        gy = int(math.floor((sin_yaw * dx + cos_yaw * dy) / resolution))
        return gx, gy

    def is_blacklisted(self, x: float, y: float, current_time: float) -> bool:
        """Check if a world coordinate falls within any active blacklist radius."""
        for entry in self.blacklist:
            if current_time <= entry.expiry_time:
                dist = math.hypot(x - entry.x, y - entry.y)
                if dist <= self.blacklist_radius:
                    return True
        return False

    def add_to_blacklist(self, x: float, y: float, current_time: float):
        """Register a destination into the blacklist or extend existing nearby entry."""
        for entry in self.blacklist:
            if math.hypot(x - entry.x, y - entry.y) <= self.blacklist_radius:
                entry.expiry_time = current_time + self.blacklist_duration * (entry.failure_count + 1)
                entry.failure_count += 1
                return

        self.blacklist.append(
            BlacklistEntry(
                x=x,
                y=y,
                added_time=current_time,
                expiry_time=current_time + self.blacklist_duration,
                failure_count=1
            )
        )

    def prune_blacklist(self, current_time: float):
        """Purge expired blacklist entries."""
        self.blacklist = [e for e in self.blacklist if current_time <= e.expiry_time]

    def clear_blacklist(self):
        """Manually clear the blacklist."""
        self.blacklist.clear()

    def parse_occupancy_grid(self, grid_msg) -> Optional[Tuple[np.ndarray, float, float, float, float]]:
        """
        Extract numpy 2D array and metadata from nav_msgs/OccupancyGrid or dict.
        Returns: (grid_array [height, width], resolution, origin_x, origin_y, origin_yaw)
        """
        if grid_msg is None:
            return None

        if hasattr(grid_msg, 'info') and hasattr(grid_msg, 'data'):
            width = grid_msg.info.width
            height = grid_msg.info.height
            resolution = grid_msg.info.resolution
            origin_x = grid_msg.info.origin.position.x
            origin_y = grid_msg.info.origin.position.y
            origin_yaw = quaternion_to_yaw(grid_msg.info.origin.orientation)
            data = grid_msg.data
        elif isinstance(grid_msg, dict):
            width = grid_msg.get('width', 0)
            height = grid_msg.get('height', 0)
            resolution = grid_msg.get('resolution', 0.05)
            origin_x = grid_msg.get('origin_x', 0.0)
            origin_y = grid_msg.get('origin_y', 0.0)
            origin_yaw = grid_msg.get('origin_yaw', 0.0)
            data = grid_msg.get('data', [])
        else:
            return None

        if width <= 0 or height <= 0 or len(data) != width * height or resolution <= 0.0:
            return None

        # Convert to 2D numpy array with shape [height, width] (row-major: y, x)
        arr = np.array(data, dtype=np.int8).reshape((height, width))
        return arr, resolution, origin_x, origin_y, origin_yaw

    def find_frontier_cells(
        self,
        grid_2d: np.ndarray,
        resolution: float
    ) -> Tuple[List[Tuple[int, int]], np.ndarray]:
        """
        Identify all free cells adjacent to unknown space while maintaining clearance.
        Returns:
          - list of (x, y) frontier cells (x=col, y=row)
          - distance_map (float array in meters to nearest obstacle)
        """
        height, width = grid_2d.shape

        free_mask = (grid_2d >= 0) & (grid_2d <= self.free_threshold)
        unknown_mask = (grid_2d == -1)
        occupied_mask = (grid_2d > self.occupied_threshold)

        # Compute Euclidean Distance Transform from occupied cells
        # EDT of non-occupied space: 0 at obstacles, increasing with distance
        if np.any(occupied_mask):
            dist_cells = ndimage.distance_transform_edt(~occupied_mask)
            dist_meters = dist_cells * resolution
        else:
            # If no obstacles mapped yet, clearance is infinite
            dist_meters = np.full((height, width), 999.0, dtype=np.float32)

        # Safe clearance mask
        safe_mask = dist_meters >= self.obstacle_safety_distance

        # Identify free cells that have at least one 8-neighbor unknown
        # Binary dilation of unknown mask with 3x3 structuring element
        struct = np.ones((3, 3), dtype=bool)
        unknown_dilated = ndimage.binary_dilation(unknown_mask, structure=struct)

        # Frontier cells are: FREE AND touches UNKNOWN AND SAFE distance from obstacles
        frontier_mask = free_mask & unknown_dilated & safe_mask

        # Additional safety: ensure NO direct 8-neighbor is occupied
        occupied_dilated = ndimage.binary_dilation(occupied_mask, structure=struct)
        frontier_mask = frontier_mask & (~occupied_dilated)

        # Extract (x, y) coordinates: y=row, x=col
        rows, cols = np.where(frontier_mask)
        frontier_cells = list(zip(cols.tolist(), rows.tolist()))

        return frontier_cells, dist_meters

    def cluster_frontiers(
        self,
        frontier_cells: List[Tuple[int, int]],
        grid_shape: Tuple[int, int]
    ) -> List[FrontierCluster]:
        """
        Group individual frontier cells into spatial clusters using 8-connected BFS.
        Filters out clusters smaller than min_cluster_size.
        """
        if not frontier_cells:
            return []

        height, width = grid_shape
        cell_set = set(frontier_cells)
        visited = set()
        clusters: List[FrontierCluster] = []

        neighbors_8 = [
            (-1, -1), (0, -1), (1, -1),
            (-1,  0),          (1,  0),
            (-1,  1), (0,  1), (1,  1)
        ]

        for cell in frontier_cells:
            if cell in visited:
                continue

            # Start BFS for a new cluster
            cluster_cells = []
            queue = collections.deque([cell])
            visited.add(cell)

            while queue:
                curr = queue.popleft()
                cluster_cells.append(curr)
                cx, cy = curr

                for dx, dy in neighbors_8:
                    nx, ny = cx + dx, cy + dy
                    n_cell = (nx, ny)
                    if n_cell in cell_set and n_cell not in visited:
                        visited.add(n_cell)
                        queue.append(n_cell)

            # Filter small noise clusters
            if len(cluster_cells) >= self.min_cluster_size:
                avg_x = sum(c[0] for c in cluster_cells) / len(cluster_cells)
                avg_y = sum(c[1] for c in cluster_cells) / len(cluster_cells)

                clusters.append(
                    FrontierCluster(
                        cells=cluster_cells,
                        size=len(cluster_cells),
                        centroid_grid=(avg_x, avg_y)
                    )
                )

        return clusters

    def extract_safe_goal(
        self,
        cluster: FrontierCluster,
        grid_2d: np.ndarray,
        dist_meters: np.ndarray,
        resolution: float,
        origin_x: float,
        origin_y: float,
        origin_yaw: float
    ) -> Optional[FrontierGoal]:
        """
        Extract a valid, collision-free goal inside known free space for a cluster.
        Computes goal orientation pointing toward the cluster's adjacent unknown space.
        """
        height, width = grid_2d.shape
        cx, cy = cluster.centroid_grid

        # 1. Select the cell in the cluster that is:
        #    a) Free space
        #    b) Has highest clearance from obstacles
        #    c) Closest to the cluster centroid
        best_cell = None
        best_score = -float('inf')

        for (gx, gy) in cluster.cells:
            clearance = dist_meters[gy, gx]
            if clearance < self.obstacle_safety_distance:
                continue

            dist_to_centroid = math.hypot(gx - cx, gy - cy)
            # Score favors clearance and central placement
            score = clearance * 2.0 - dist_to_centroid * resolution
            if score > best_score:
                best_score = score
                best_cell = (gx, gy)

        # Fallback: if no cell in cluster exceeds safety distance, search within 3 cells around centroid in free space
        if best_cell is None:
            icx, icy = int(round(cx)), int(round(cy))
            search_radius = 4
            for dy in range(-search_radius, search_radius + 1):
                for dx in range(-search_radius, search_radius + 1):
                    nx, ny = icx + dx, icy + dy
                    if 0 <= nx < width and 0 <= ny < height:
                        if 0 <= grid_2d[ny, nx] <= self.free_threshold:
                            clearance = dist_meters[ny, nx]
                            if clearance >= self.obstacle_safety_distance:
                                dist_to_centroid = math.hypot(nx - cx, ny - cy)
                                score = clearance * 2.0 - dist_to_centroid * resolution
                                if score > best_score:
                                    best_score = score
                                    best_cell = (nx, ny)

        if best_cell is None:
            return None

        gx, gy = best_cell

        # 2. Find unknown neighbors around the cluster to compute orientation
        # Vector pointing from chosen goal toward local center of unknown cells
        unknown_cells = []
        for (fcx, fcy) in cluster.cells:
            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (1,-1), (-1,1), (1,1)]:
                ux, uy = fcx + dx, fcy + dy
                if 0 <= ux < width and 0 <= uy < height:
                    if grid_2d[uy, ux] == -1:
                        unknown_cells.append((ux, uy))

        if unknown_cells:
            avg_ux = sum(u[0] for u in unknown_cells) / len(unknown_cells)
            avg_uy = sum(u[1] for u in unknown_cells) / len(unknown_cells)
            # Vector in grid coordinates
            grid_vec_x = avg_ux - gx
            grid_vec_y = avg_uy - gy
            # Convert direction to world angle (adding origin_yaw)
            raw_yaw = math.atan2(grid_vec_y, grid_vec_x)
            goal_yaw = (raw_yaw + origin_yaw + math.pi) % (2.0 * math.pi) - math.pi
        else:
            goal_yaw = 0.0

        # 3. Convert to world coordinates
        wx, wy = self.grid_to_world(gx, gy, origin_x, origin_y, resolution, origin_yaw)
        clearance = float(dist_meters[gy, gx])

        return FrontierGoal(
            x=wx,
            y=wy,
            yaw=goal_yaw,
            cluster_size=cluster.size,
            obstacle_distance=clearance,
            grid_x=gx,
            grid_y=gy
        )

    def rank_goals(
        self,
        candidate_goals: List[FrontierGoal],
        robot_pose: Tuple[float, float],
        current_time: float
    ) -> List[FrontierGoal]:
        """
        Rank candidate goals based on travel distance, cluster size, obstacle clearance,
        and blacklist suppression.
        Returns sorted list with lowest cost (highest utility) first.
        """
        self.prune_blacklist(current_time)
        rx, ry = robot_pose

        valid_ranked: List[FrontierGoal] = []

        for goal in candidate_goals:
            # Check blacklist
            if self.is_blacklisted(goal.x, goal.y, current_time):
                continue

            dist = math.hypot(goal.x - rx, goal.y - ry)
            goal.distance_to_robot = dist

            # Filter out goals directly beneath the robot (e.g. < 0.5m) to avoid thrashing
            if dist < self.min_distance_to_goal:
                # Still allowed if cluster is large, but heavily penalized
                dist_penalty = (self.min_distance_to_goal - dist) * 5.0
            elif dist > self.max_distance_to_goal:
                dist_penalty = (dist - self.max_distance_to_goal) * 2.0
            else:
                dist_penalty = 0.0

            # Cost function: Lower cost = better destination
            # Cost = w_dist * Distance - w_size * ClusterSize - w_clearance * ObstacleClearance + Penalty
            cost = (
                self.weight_distance * dist -
                self.weight_size * goal.cluster_size -
                self.weight_clearance * goal.obstacle_distance +
                dist_penalty
            )
            goal.cost = cost
            valid_ranked.append(goal)

        # Sort ascending by cost
        valid_ranked.sort(key=lambda g: g.cost)
        return valid_ranked

    def detect_and_rank(
        self,
        grid_msg,
        robot_pose: Tuple[float, float],
        current_time: float
    ) -> Tuple[List[FrontierGoal], List[FrontierCluster]]:
        """
        Full pipeline: parse grid -> detect frontiers -> cluster -> extract safe goals -> rank.
        Returns:
          - ranked_goals: List of FrontierGoal sorted by priority
          - clusters: List of valid FrontierCluster objects
        """
        parsed = self.parse_occupancy_grid(grid_msg)
        if parsed is None:
            return [], []

        grid_2d, resolution, origin_x, origin_y, origin_yaw = parsed

        # 1. Detect frontier cells
        frontier_cells, dist_meters = self.find_frontier_cells(grid_2d, resolution)
        if not frontier_cells:
            return [], []

        # 2. Cluster frontier cells
        clusters = self.cluster_frontiers(frontier_cells, grid_2d.shape)
        if not clusters:
            return [], []

        # 3. Extract safe goals
        candidate_goals: List[FrontierGoal] = []
        for cluster in clusters:
            goal = self.extract_safe_goal(
                cluster,
                grid_2d,
                dist_meters,
                resolution,
                origin_x,
                origin_y,
                origin_yaw
            )
            if goal is not None:
                cluster.goal = goal
                candidate_goals.append(goal)

        if not candidate_goals:
            return [], clusters

        # 4. Rank goals
        ranked_goals = self.rank_goals(candidate_goals, robot_pose, current_time)
        return ranked_goals, clusters

