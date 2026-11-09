#!/usr/bin/env python3
"""
Autonomous Frontier Exploration ROS 2 Node.

Coordinates:
- SLAM occupancy map updates (/map).
- TF2 robot pose tracking (map -> base_footprint).
- Algorithmic frontier detection, clustering, safety filtering, and ranking.
- Nav2 NavigateToPose action goal dispatching and lifecycle monitoring.
- Spatial blacklisting of failed navigation goals.
- Exploration state machine (INIT -> IDLE -> PLANNING -> NAVIGATING -> COMPLETED).
- RViz2 visualization marker publication.
- Pause/resume/stop service interfaces.
- Zero direct /cmd_vel publishing (preserves Nav2 obstacle avoidance safety).
"""

import math
import time
from enum import Enum
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger, SetBool
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

try:
    from nav2_msgs.action import NavigateToPose
except ImportError:
    # If bootstrapping from standard environment
    pass

from frontier_exploration.frontier_detector import (
    FrontierDetector,
    FrontierGoal,
    FrontierCluster,
    quaternion_to_yaw
)


class ExplorationState(Enum):
    INIT = "INIT"
    IDLE = "IDLE"
    PLANNING = "PLANNING"
    NAVIGATING = "NAVIGATING"
    WAITING_RETRY = "WAITING_RETRY"
    PAUSED = "PAUSED"
    COMPLETED = "COMPLETED"
    SHUTDOWN = "SHUTDOWN"


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert a planar yaw angle into a ROS Quaternion."""
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0)
    )


class ExplorerNode(Node):
    """
    Main Autonomous Frontier Exploration Node for ROS 2 Humble.
    """

    def __init__(self):
        super().__init__('frontier_explorer')

        # 1. Declare Parameters safely (handles launch-overridden use_sim_time)
        def safe_declare(name, default):
            if not self.has_parameter(name):
                return self.declare_parameter(name, default)
            return self.get_parameter(name)

        safe_declare('use_sim_time', True)
        safe_declare('map_topic', '/map')
        safe_declare('robot_base_frame', 'base_footprint')
        safe_declare('global_frame', 'map')
        safe_declare('update_interval', 2.0)
        safe_declare('min_frontier_cluster_size', 5)
        safe_declare('min_distance_to_goal', 0.5)
        safe_declare('max_distance_to_goal', 10.0)
        safe_declare('goal_timeout', 45.0)
        safe_declare('blacklist_radius', 0.6)
        safe_declare('blacklist_duration', 60.0)
        safe_declare('obstacle_safety_distance', 0.28)
        safe_declare('weight_distance', 1.0)
        safe_declare('weight_size', 0.05)
        safe_declare('weight_clearance', 0.2)
        safe_declare('auto_start', True)
        safe_declare('max_retries', 5)
        safe_declare('retry_delay', 3.0)

        # Retrieve parameter values
        self.map_topic = self.get_parameter('map_topic').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.global_frame = self.get_parameter('global_frame').value
        self.update_interval = self.get_parameter('update_interval').value
        self.goal_timeout = self.get_parameter('goal_timeout').value
        self.max_retries = self.get_parameter('max_retries').value
        self.retry_delay = self.get_parameter('retry_delay').value
        auto_start = self.get_parameter('auto_start').value

        # 2. Instantiate Algorithmic Detector
        self.detector = FrontierDetector(
            min_cluster_size=self.get_parameter('min_frontier_cluster_size').value,
            obstacle_safety_distance=self.get_parameter('obstacle_safety_distance').value,
            min_distance_to_goal=self.get_parameter('min_distance_to_goal').value,
            max_distance_to_goal=self.get_parameter('max_distance_to_goal').value,
            weight_distance=self.get_parameter('weight_distance').value,
            weight_size=self.get_parameter('weight_size').value,
            weight_clearance=self.get_parameter('weight_clearance').value,
            blacklist_radius=self.get_parameter('blacklist_radius').value,
            blacklist_duration=self.get_parameter('blacklist_duration').value
        )

        # 3. State Tracking
        self.state = ExplorationState.INIT
        self.sim_time: float = 0.0
        self.latest_map: Optional[OccupancyGrid] = None
        self.current_goal: Optional[FrontierGoal] = None
        self.goal_start_time: float = 0.0
        self.active_goal_handle = None
        self.retry_count = 0
        self.last_retry_time: float = 0.0
        self.total_goals_completed = 0
        self.total_goals_failed = 0

        # 4. TF2 Buffer & Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 5. Subscribers
        # Simulation clock
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_cb, 10)

        # Map subscription with TRANSIENT_LOCAL durability to receive latched maps
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_cb,
            map_qos
        )

        # 6. Publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/exploration/frontiers', 10)
        self.goal_marker_pub = self.create_publisher(Marker, '/exploration/current_goal', 10)

        # 7. Services
        self.pause_srv = self.create_service(Trigger, '/exploration/pause', self.pause_cb)
        self.resume_srv = self.create_service(Trigger, '/exploration/resume', self.resume_cb)
        self.stop_srv = self.create_service(Trigger, '/exploration/stop', self.stop_cb)

        # 8. Action Client for Nav2 NavigateToPose
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 9. Main Periodic Control Loop Timer
        self.timer = self.create_timer(self.update_interval, self.control_loop)

        if auto_start:
            self.get_logger().info("Frontier Exploration Node initialized with auto_start=True.")
        else:
            self.state = ExplorationState.PAUSED
            self.get_logger().info("Frontier Exploration Node initialized in PAUSED state.")

    def get_current_time(self) -> float:
        """Return current simulation time if available, else wall clock."""
        if self.sim_time > 0.0:
            return self.sim_time
        return time.time()

    def clock_cb(self, msg: Clock):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def map_cb(self, msg: OccupancyGrid):
        self.latest_map = msg
        if self.state == ExplorationState.INIT:
            self.get_logger().info(
                f"Received initial map ({msg.info.width}x{msg.info.height} @ {msg.info.resolution:.3f}m/cell). Ready to explore."
            )
            self.state = ExplorationState.IDLE

    def get_robot_pose(self) -> Optional[Tuple[float, float, float]]:
        """Look up live robot pose in the global map frame."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_base_frame,
                Time()
            )
            rx = t.transform.translation.x
            ry = t.transform.translation.y
            ryaw = quaternion_to_yaw(t.transform.rotation)
            return rx, ry, ryaw
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            return None

    def control_loop(self):
        """Core state machine execution periodic timer."""
        now = self.get_current_time()

        # Handle PAUSED or COMPLETED states
        if self.state in [ExplorationState.PAUSED, ExplorationState.COMPLETED, ExplorationState.SHUTDOWN]:
            return

        # Handle INIT state: wait for map and TF
        if self.state == ExplorationState.INIT:
            if self.latest_map is not None and self.get_robot_pose() is not None:
                self.state = ExplorationState.IDLE
            return

        # Check if action server is available before proceeding
        if not self.nav_client.server_is_ready():
            if not self.nav_client.wait_for_server(timeout_sec=0.1):
                self.get_logger().warn(
                    "NavigateToPose action server not ready. Waiting...",
                    throttle_duration_sec=5.0
                )
                return

        # Monitor active goal when NAVIGATING
        if self.state == ExplorationState.NAVIGATING:
            # Check timeout
            elapsed = now - self.goal_start_time
            if elapsed > self.goal_timeout:
                self.get_logger().warn(
                    f"Navigation goal to ({self.current_goal.x:.2f}, {self.current_goal.y:.2f}) timed out after {elapsed:.1f}s."
                )
                self.cancel_active_goal()
                if self.current_goal:
                    self.detector.add_to_blacklist(self.current_goal.x, self.current_goal.y, now)
                    self.total_goals_failed += 1
                self.current_goal = None
                self.state = ExplorationState.IDLE
            return

        # Handle WAITING_RETRY state
        if self.state == ExplorationState.WAITING_RETRY:
            if now - self.last_retry_time >= self.retry_delay:
                self.get_logger().info(f"Retrying frontier search (attempt {self.retry_count}/{self.max_retries})...")
                self.state = ExplorationState.PLANNING
            else:
                return

        # If IDLE, plan next goal
        if self.state == ExplorationState.IDLE:
            self.state = ExplorationState.PLANNING

        # PLANNING: Search and dispatch next frontier goal
        if self.state == ExplorationState.PLANNING:
            self.plan_and_dispatch_next_goal()

    def plan_and_dispatch_next_goal(self):
        """Detect frontiers, select highest utility candidate, and dispatch to Nav2."""
        now = self.get_current_time()

        if self.latest_map is None:
            self.state = ExplorationState.IDLE
            return

        pose = self.get_robot_pose()
        if pose is None:
            self.get_logger().warn(
                f"Cannot lookup transform {self.global_frame} -> {self.robot_base_frame}. Retrying...",
                throttle_duration_sec=3.0
            )
            self.state = ExplorationState.IDLE
            return

        rx, ry, ryaw = pose

        # Run frontier detection and ranking
        ranked_goals, clusters = self.detector.detect_and_rank(
            self.latest_map,
            robot_pose=(rx, ry),
            current_time=now
        )

        # Publish visualization markers for RViz2
        self.publish_markers(clusters, ranked_goals)

        if not ranked_goals:
            # No valid reachable frontiers found
            if self.retry_count < self.max_retries:
                self.retry_count += 1
                self.last_retry_time = now
                self.state = ExplorationState.WAITING_RETRY
                # On higher retry counts, prune older blacklists to allow second-chance exploration
                if self.retry_count >= 2:
                    self.detector.clear_blacklist()
                    self.get_logger().info("Cleared blacklist for second-chance frontier exploration.")
                self.get_logger().info(
                    f"No valid frontiers currently reachable. Waiting {self.retry_delay}s before retry ({self.retry_count}/{self.max_retries})."
                )
            else:
                self.get_logger().info(
                    "============================================================"
                )
                self.get_logger().info(
                    f"EXPLORATION COMPLETE: All frontiers explored or verified unreachable!"
                )
                self.get_logger().info(
                    f"Total goals reached: {self.total_goals_completed}, Failed/timed-out: {self.total_goals_failed}"
                )
                self.get_logger().info(
                    "============================================================"
                )
                self.state = ExplorationState.COMPLETED
                self.clear_goal_marker()
            return

        # Reset retry counter on finding valid goals
        self.retry_count = 0

        # Select top-ranked goal
        selected_goal = ranked_goals[0]
        self.dispatch_goal(selected_goal)

    def dispatch_goal(self, goal: FrontierGoal):
        """Send a FrontierGoal to Nav2 NavigateToPose action server."""
        now = self.get_current_time()
        self.current_goal = goal
        self.goal_start_time = now
        self.state = ExplorationState.NAVIGATING

        self.get_logger().info(
            f"Dispatching Goal: target=({goal.x:.2f}, {goal.y:.2f}, yaw={goal.yaw:.2f} rad), "
            f"dist={goal.distance_to_robot:.2f}m, cluster_size={goal.cluster_size}, clearance={goal.obstacle_distance:.2f}m"
        )

        # Build NavigateToPose action goal
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose.header.frame_id = self.global_frame
        nav_goal.pose.header.stamp = self.get_clock().now().to_msg()
        nav_goal.pose.pose.position.x = goal.x
        nav_goal.pose.pose.position.y = goal.y
        nav_goal.pose.pose.position.z = 0.0
        nav_goal.pose.pose.orientation = yaw_to_quaternion(goal.yaw)

        # Publish goal marker
        self.publish_goal_marker(goal)

        # Send goal async
        send_future = self.nav_client.send_goal_async(
            nav_goal,
            feedback_callback=self.action_feedback_cb
        )
        send_future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        """Callback when NavigateToPose server accepts or rejects goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(
                f"Goal to ({self.current_goal.x:.2f}, {self.current_goal.y:.2f}) was REJECTED by Nav2."
            )
            now = self.get_current_time()
            if self.current_goal:
                self.detector.add_to_blacklist(self.current_goal.x, self.current_goal.y, now)
                self.total_goals_failed += 1
            self.current_goal = None
            self.active_goal_handle = None
            self.state = ExplorationState.IDLE
            return

        self.active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_cb)

    def goal_result_cb(self, future):
        """Callback when NavigateToPose action terminates."""
        now = self.get_current_time()
        result = future.result()
        status = result.status

        # GoalStatus enum: STATUS_SUCCEEDED == 4, STATUS_ABORTED == 5, STATUS_CANCELED == 6
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f"Goal at ({self.current_goal.x:.2f}, {self.current_goal.y:.2f}) SUCCEEDED!"
            )
            self.total_goals_completed += 1
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(
                f"Goal at ({self.current_goal.x:.2f}, {self.current_goal.y:.2f}) was CANCELED."
            )
        else:
            self.get_logger().warn(
                f"Goal at ({self.current_goal.x:.2f}, {self.current_goal.y:.2f}) FAILED (status code: {status}). Blacklisting."
            )
            if self.current_goal:
                self.detector.add_to_blacklist(self.current_goal.x, self.current_goal.y, now)
                self.total_goals_failed += 1

        self.current_goal = None
        self.active_goal_handle = None

        if self.state not in [ExplorationState.PAUSED, ExplorationState.COMPLETED, ExplorationState.SHUTDOWN]:
            self.state = ExplorationState.IDLE

    def action_feedback_cb(self, feedback_msg):
        """Monitor navigation progress feedback."""
        pass

    def cancel_active_goal(self):
        """Cancel current active Nav2 navigation goal if executing."""
        if self.active_goal_handle is not None:
            self.get_logger().info("Canceling active Nav2 goal...")
            self.active_goal_handle.cancel_goal_async()
            self.active_goal_handle = None

    # Service callbacks
    def pause_cb(self, request, response):
        """Pause exploration service callback."""
        self.get_logger().info("Exploration PAUSED via service call.")
        self.state = ExplorationState.PAUSED
        self.cancel_active_goal()
        self.clear_goal_marker()
        response.success = True
        response.message = "Exploration paused."
        return response

    def resume_cb(self, request, response):
        """Resume exploration service callback."""
        self.get_logger().info("Exploration RESUMED via service call.")
        self.state = ExplorationState.IDLE
        response.success = True
        response.message = "Exploration resumed."
        return response

    def stop_cb(self, request, response):
        """Stop exploration service callback."""
        self.get_logger().info("Exploration STOPPED via service call.")
        self.state = ExplorationState.COMPLETED
        self.cancel_active_goal()
        self.clear_goal_marker()
        response.success = True
        response.message = "Exploration stopped."
        return response

    # Marker publication
    def publish_markers(self, clusters: list, ranked_goals: list):
        """Publish visualization markers for RViz2."""
        marker_array = MarkerArray()
        now_msg = self.get_clock().now().to_msg()

        # Marker 1: Frontier cluster cells (Spheres)
        m_cells = Marker()
        m_cells.header.frame_id = self.global_frame
        m_cells.header.stamp = now_msg
        m_cells.ns = "frontier_cells"
        m_cells.id = 0
        m_cells.type = Marker.SPHERE_LIST
        m_cells.action = Marker.ADD
        m_cells.scale.x = 0.08
        m_cells.scale.y = 0.08
        m_cells.scale.z = 0.08
        m_cells.color.r = 0.0
        m_cells.color.g = 0.9
        m_cells.color.b = 1.0
        m_cells.color.a = 0.75

        res = self.latest_map.info.resolution
        ox = self.latest_map.info.origin.position.x
        oy = self.latest_map.info.origin.position.y
        oyaw = quaternion_to_yaw(self.latest_map.info.origin.orientation)

        for cluster in clusters:
            for (gx, gy) in cluster.cells:
                wx, wy = self.detector.grid_to_world(gx, gy, ox, oy, res, oyaw)
                p = Point(x=wx, y=wy, z=0.04)
                m_cells.points.append(p)

        marker_array.markers.append(m_cells)

        # Marker 2: Candidate goals (Arrows)
        m_candidates = Marker()
        m_candidates.header.frame_id = self.global_frame
        m_candidates.header.stamp = now_msg
        m_candidates.ns = "candidate_goals"
        m_candidates.id = 1
        m_candidates.type = Marker.SPHERE_LIST
        m_candidates.action = Marker.ADD
        m_candidates.scale.x = 0.22
        m_candidates.scale.y = 0.22
        m_candidates.scale.z = 0.22
        m_candidates.color.r = 1.0
        m_candidates.color.g = 0.8
        m_candidates.color.b = 0.0
        m_candidates.color.a = 0.85

        for g in ranked_goals:
            m_candidates.points.append(Point(x=g.x, y=g.y, z=0.10))

        marker_array.markers.append(m_candidates)
        self.marker_pub.publish(marker_array)

    def publish_goal_marker(self, goal: FrontierGoal):
        """Publish currently selected goal as distinct red marker."""
        m = Marker()
        m.header.frame_id = self.global_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "active_goal"
        m.id = 0
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = goal.x
        m.pose.position.y = goal.y
        m.pose.position.z = 0.20
        m.pose.orientation = yaw_to_quaternion(goal.yaw)
        m.scale.x = 0.35
        m.scale.y = 0.35
        m.scale.z = 0.40
        m.color.r = 1.0
        m.color.g = 0.1
        m.color.b = 0.1
        m.color.a = 0.90
        self.goal_marker_pub.publish(m)

    def clear_goal_marker(self):
        """Remove active goal marker on completion or stop."""
        m = Marker()
        m.header.frame_id = self.global_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "active_goal"
        m.id = 0
        m.action = Marker.DELETE
        self.goal_marker_pub.publish(m)

    def destroy_node(self):
        """Ensure clean shutdown and goal cancellation."""
        self.state = ExplorationState.SHUTDOWN
        self.cancel_active_goal()
        self.clear_goal_marker()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Frontier Explorer shutting down cleanly on SIGINT.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

