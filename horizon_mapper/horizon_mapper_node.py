"""
Horizon Mapper Node
===================

Version: 1.4.1
Author: Mohammed Azab (Mohammed@azab.io)

Description:
------------
ROS2 node for trajectory mapping and predictive horizon generation for MPC-based autonomous vehicles.
This node provides adaptive bounds, visualization, and runtime adjustments.

Features:
- Loads and preprocesses trajectory data
- Publishes reference and constrained trajectories
- Provides visualization capabilities
- Handles runtime bound adjustments
- Validates trajectory and vehicle state

Interfaces:
- nav_msgs.msg.Path, Odometry
- geometry_msgs.msg.PoseStamped, PoseWithCovarianceStamped, Point
- std_msgs.msg.Bool, Header, ColorRGBA
- visualization_msgs.msg.Marker, MarkerArray
- giu_f1t_interfaces.msg.VehicleState, VehicleStateArray, ConstrainedVehicleStateArray, ConstrainedVehicleState, BoundAdjustment, ConstraintStatus

Copyright (c) 2025 GIU-F1Tenth
License: MIT
"""

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
from std_msgs.msg import Bool, Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
from giu_f1t_interfaces.msg import (
    VehicleState,
    VehicleStateArray,
    ConstrainedVehicleStateArray,
    ConstrainedVehicleState,
    BoundAdjustment,
    ConstraintStatus,
)
try:
    from .preprocess_trajectory import preprocess_trajectory
except ImportError:
    def preprocess_trajectory(*args, **kwargs):
        return []
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import csv
import math
import numpy as np
import colorsys
import os
from typing import List, Tuple, Optional
from tf_transformations import euler_from_quaternion
from ament_index_python.packages import get_package_share_directory

# Default configuration class - provides fallback values


class DefaultConfig:
    def __init__(self):
        pkg_share = get_package_share_directory('horizon_mapper')
        # Check multiple possible install locations
        install_data_optimal = os.path.join(pkg_share, 'data', 'optimal_trajectory.csv')
        install_data_reference = os.path.join(pkg_share, 'data', 'ref_trajectory.csv')
        install_optimal = os.path.join(pkg_share, 'optimal_trajectory.csv')
        install_reference = os.path.join(pkg_share, 'ref_trajectory.csv')
        src_optimal = os.path.join(os.path.dirname(__file__), 'optimal_trajectory.csv')
        src_reference = os.path.join(os.path.dirname(__file__), 'ref_trajectory.csv')

        # Prefer install space (data directory), then install space root, then source space
        if os.path.exists(install_data_optimal) and os.path.exists(install_data_reference):
            self.optimal_trajectory_path = install_data_optimal
            self.reference_trajectory_path = install_data_reference
        elif os.path.exists(install_optimal) and os.path.exists(install_reference):
            self.optimal_trajectory_path = install_optimal
            self.reference_trajectory_path = install_reference
        else:
            self.optimal_trajectory_path = src_optimal
            self.reference_trajectory_path = src_reference

        self.enable_trajectory_generation = True
        self.horizon_N = 8
        self.wheelbase = 0.33
        self.max_steering_angle = 0.5
        self.min_speed = 0.1
        self.odom_topic = "car_state/odom"
        self.enable_logging = True
        self.enable_debugging = False


default_config = DefaultConfig()


class HorizonMapperNode(Node):
    """
    Horizon Mapper Node

    This node:
    1. Loads and preprocesses trajectory data
    2. Publishes reference trajectories for MPC horizon
    3. Publishes constrained trajectories with adaptive bounds
    4. Provides visualization capabilities
    5. Handles runtime bound adjustments
    6. Provides trajectory validation and error handling
    """

    def __init__(self):
        super().__init__('horizon_mapper_node')

        # Declare ROS2 parameters
        self._declare_parameters()
        self._load_parameters()

        # Initialize state variables
        self._initialize_state_variables()

        # Initialize TF2 if enhanced mode
        if self.enhanced_mode:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create subscribers
        self._create_subscribers()

        # Create publishers
        self._create_publishers()

        # Create timers
        self._create_timers()

        # Run preprocessing on startup
        self.preprocess_and_load_trajectory()

        self.get_logger().info("Horizon Mapper Node initialized")

    def _declare_parameters(self):
        """Declare all ROS2 parameters with descriptions"""

        self.declare_parameter('enhanced_mode', True, ParameterDescriptor(
            description='Enable enhanced features (constraints, visualization)'))

        # Legacy parameters
        self.declare_parameter('enable_logging', default_config.enable_logging)
        self.declare_parameter('enable_debugging', default_config.enable_debugging)
        self.declare_parameter('optimal_trajectory_path', default_config.optimal_trajectory_path)
        self.declare_parameter('reference_trajectory_path', default_config.reference_trajectory_path)
        self.declare_parameter('horizon_N', default_config.horizon_N)
        self.declare_parameter('horizon_T', 0.5)
        self.declare_parameter('wheelbase', default_config.wheelbase)
        self.declare_parameter('max_steering_angle', default_config.max_steering_angle)
        self.declare_parameter('min_speed', default_config.min_speed)
        self.declare_parameter('odom_topic', default_config.odom_topic)

        # Enhanced parameters
        self.declare_parameter('default_left_bound', 1.0,
                               ParameterDescriptor(description='Default left lateral bound [m]'))
        self.declare_parameter('default_right_bound', 1.0,
                               ParameterDescriptor(description='Default right lateral bound [m]'))
        self.declare_parameter('default_confidence', 0.8,
                               ParameterDescriptor(description='Default confidence [0.0-1.0]'))
        self.declare_parameter('adaptive_bounds', True,
                               ParameterDescriptor(description='Enable adaptive bound adjustments'))
        self.declare_parameter('publish_visualization', True,
                               ParameterDescriptor(description='Enable visualization publishing'))
        self.declare_parameter('velocity_color_scale', 10.0,
                               ParameterDescriptor(description='Maximum velocity for color scale [m/s]'))
        self.declare_parameter('map_frame', 'map',
                               ParameterDescriptor(description='Map frame ID'))

    def _load_parameters(self):
        """Load parameters from parameter server"""
        self.enhanced_mode = True

        # Legacy parameters
        self.enable_logging = self.get_parameter('enable_logging').value
        self.enable_debugging = self.get_parameter('enable_debugging').value
        self.input_path = self.get_parameter('optimal_trajectory_path').value
        self.output_path = self.get_parameter('reference_trajectory_path').value
        self.horizon = self.get_parameter('horizon_N').value
        self.horizon_T = self.get_parameter('horizon_T').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_steering = self.get_parameter('max_steering_angle').value
        self.min_speed = self.get_parameter('min_speed').value
        self.odom_topic = self.get_parameter('odom_topic').value

        # Enhanced parameters
        self.default_left_bound = self.get_parameter('default_left_bound').value
        self.default_right_bound = self.get_parameter('default_right_bound').value
        self.default_confidence = self.get_parameter('default_confidence').value
        self.adaptive_bounds = self.get_parameter('adaptive_bounds').value
        self.publish_visualization = self.get_parameter('publish_visualization').value
        self.velocity_color_scale = self.get_parameter('velocity_color_scale').value
        self.map_frame = self.get_parameter('map_frame').value

    def _initialize_state_variables(self):
        """Initialize state variables"""
        # Legacy state variables
        self.reference_trajectory = []
        self.path_ready = False
        self.current_vehicle_state = VehicleState()
        self.current_vehicle_state.x = 0.0
        self.current_vehicle_state.y = 0.0
        self.current_vehicle_state.v = 0.0
        self.current_vehicle_state.theta = 0.0
        self.current_vehicle_state.delta = 0.0
        self.current_pose_estimate = None
        self.trajectory_index = 0

        # Enhanced state variables
        if self.enhanced_mode:
            self.trajectory_points = []
            self.current_pose = None
            self.current_velocity = 0.0
            self.current_heading = 0.0

            # Bound adjustments
            self.left_bound_adjustment = 0.0
            self.right_bound_adjustment = 0.0
            self.confidence_adjustment = 0.0

            # Status tracking
            self.bounds_violated = False
            self.constraints_active = True

    def _create_subscribers(self):
        """Create ROS2 subscribers"""
        # Legacy subscribers
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odometry_callback, 10)

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.pose_estimate_callback, 10)

        # Enhanced subscribers
        if self.enhanced_mode and self.adaptive_bounds:
            self.bound_adjustment_sub = self.create_subscription(
                BoundAdjustment, '/mpc/bound_adjustments', self._bound_adjustment_callback, 10)

    def _create_publishers(self):
        """Create ROS2 publishers"""
        # Legacy publishers (always available)
        self.path_pub = self.create_publisher(Path, '/horizon_mapper/reference_path', 10)
        self.trajectory_pub = self.create_publisher(VehicleStateArray, '/horizon_mapper/reference_trajectory', 10)
        self.status_pub = self.create_publisher(Bool, '/horizon_mapper/path_ready', 10)

        # Enhanced publishers
        if self.enhanced_mode:
            self.constrained_trajectory_pub = self.create_publisher(
                ConstrainedVehicleStateArray, '/horizon_mapper/constrained_trajectory', 10)
            self.constraint_status_pub = self.create_publisher(
                ConstraintStatus, '/horizon_mapper/constraint_status', 10)

            if self.publish_visualization:
                self.corridor_viz_pub = self.create_publisher(
                    MarkerArray, '/horizon_mapper/corridor_visualization', 10)
                self.velocity_path_pub = self.create_publisher(
                    MarkerArray, '/horizon_mapper/velocity_path', 10)

    def _create_timers(self):
        """Create ROS2 timers"""
        # Main timer for trajectory publishing
        self.publish_timer = self.create_timer(0.1, self.publish_predictive_trajectory)  # 10 Hz for MPC

        # Enhanced visualization timer
        if self.enhanced_mode and self.publish_visualization:
            self.viz_timer = self.create_timer(0.2, self._publish_visualization)  # 5 Hz
        self.enable_debugging = self.get_parameter('enable_debugging').value
        self.input_path = self.get_parameter('optimal_trajectory_path').value
        self.output_path = self.get_parameter('reference_trajectory_path').value
        self.horizon = self.get_parameter('horizon_N').value
        self.horizon_T = self.get_parameter('horizon_T').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_steering = self.get_parameter('max_steering_angle').value
        self.min_speed = self.get_parameter('min_speed').value
        self.odom_topic = self.get_parameter('odom_topic').value

        if self.enable_logging:
            self.get_logger().info("Detailed logging enabled")
        else:
            self.get_logger().info("Detailed logging disabled - only warnings and errors will be shown")

        if not self.input_path or not self.output_path:
            self.get_logger().error("Missing trajectory paths in parameters!")
            return

        self.get_logger().info(f"Input trajectory: {self.input_path}")
        self.get_logger().info(f"Output trajectory: {self.output_path}")
        self.get_logger().info(f"Horizon: {self.horizon} steps")

        # State variables
        self.reference_trajectory = []
        self.path_ready = False
        self.current_vehicle_state = VehicleState()
        # Initialize vehicle state with safe default values
        self.current_vehicle_state.x = 0.0
        self.current_vehicle_state.y = 0.0
        self.current_vehicle_state.v = 0.0
        self.current_vehicle_state.theta = 0.0  # Initialize heading
        self.current_vehicle_state.delta = 0.0
        self.current_pose_estimate = None
        self.trajectory_index = 0

        # Debug: Log initial vehicle state
        self.get_logger().info(f"Initialized vehicle state: "
                               f"x={self.current_vehicle_state.x}, y={self.current_vehicle_state.y}, "
                               f"v={self.current_vehicle_state.v}, theta={self.current_vehicle_state.theta}")

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odometry_callback,
            10)

        self.get_logger().info(f"Subscribing to odometry topic: {self.odom_topic}")

        # Subscribe to RViz 2D Pose Estimate
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.pose_estimate_callback,
            10)

        # Publishers
        self.path_pub = self.create_publisher(
            Path,
            '/horizon_mapper/reference_path',
            10)

        self.trajectory_pub = self.create_publisher(
            VehicleStateArray,
            '/horizon_mapper/reference_trajectory',
            10)

        self.status_pub = self.create_publisher(
            Bool,
            '/horizon_mapper/path_ready',
            10)

        # Timers
        self.publish_timer = self.create_timer(0.1, self.publish_predictive_trajectory)  # 10 Hz for MPC

    def log_info(self, message):
        """Log info messages only if logging is enabled"""
        if self.enable_logging:
            self.get_logger().info(message)

    def log_debug(self, message):
        """Log debug messages only if logging is enabled"""
        if self.enable_debugging:
            self.get_logger().debug(message)

    def odometry_callback(self, msg):
        """Update current vehicle state from odometry"""
        try:
            # Extract position
            self.current_vehicle_state.x = msg.pose.pose.position.x
            self.current_vehicle_state.y = msg.pose.pose.position.y

            # Extract velocity (assuming this is in body frame)
            linear_velocity = msg.twist.twist.linear
            self.current_vehicle_state.v = np.sqrt(linear_velocity.x**2 + linear_velocity.y**2)

            # Extract yaw from quaternion
            orientation_q = msg.pose.pose.orientation
            _, _, yaw = euler_from_quaternion([
                orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
            ])

            # Set the theta (heading angle) - this is critical for MPC!
            self.current_vehicle_state.theta = yaw

            # For now, set delta to 0 (we'll calculate this from trajectory tracking)
            self.current_vehicle_state.delta = 0.0

            # Update enhanced state if in enhanced mode
            if self.enhanced_mode:
                self.current_pose = msg.pose.pose
                self.current_velocity = self.current_vehicle_state.v
                self.current_heading = self.current_vehicle_state.theta

            self.log_debug(f"Odometry update: x={self.current_vehicle_state.x:.2f}, "
                           f"y={self.current_vehicle_state.y:.2f}, "
                           f"v={self.current_vehicle_state.v:.2f}, "
                           f"theta={self.current_vehicle_state.theta:.2f}")

        except Exception as e:
            self.get_logger().error(f"Error processing odometry: {str(e)}")

    def pose_estimate_callback(self, msg):
        """Update pose estimate from RViz 2D Pose Estimate tool"""
        try:
            self.current_pose_estimate = msg

            # Use pose estimate to correct vehicle state
            self.current_vehicle_state.x = msg.pose.pose.position.x
            self.current_vehicle_state.y = msg.pose.pose.position.y

            # Extract yaw from quaternion for heading
            orientation_q = msg.pose.pose.orientation
            _, _, yaw = euler_from_quaternion([
                orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
            ])

            # Set the theta (heading angle) - this is critical for MPC!
            self.current_vehicle_state.theta = yaw

            self.log_info(f"Pose estimate from RViz: x={self.current_vehicle_state.x:.2f}, "
                          f"y={self.current_vehicle_state.y:.2f}, "
                          f"theta={self.current_vehicle_state.theta:.2f}")

        except Exception as e:
            self.get_logger().error(f"Error processing pose estimate: {str(e)}")

    def preprocess_and_load_trajectory(self):
        """Run preprocessing script and load trajectory"""
        try:
            self.log_info("Starting trajectory preprocessing...")

            # Run the preprocessing script with parameters
            success = preprocess_trajectory(
                self.input_path,
                self.output_path,
                wheelbase=self.wheelbase,
                max_steering=self.max_steering,
                min_velocity=self.min_speed
            )

            if success:
                self.log_info("Preprocessing completed successfully")
                self.reference_trajectory = self.load_trajectory_from_csv(self.output_path)
                self.path_ready = True
                self.log_info(f"Reference trajectory loaded: {len(self.reference_trajectory)} points")
            else:
                self.get_logger().error("Preprocessing failed")
                self.path_ready = False

        except Exception as e:
            self.get_logger().error(f"Error during preprocessing: {str(e)}")
            self.path_ready = False

    def load_trajectory_from_csv(self, path):
        """Load processed trajectory from CSV file with theta support"""
        data = []
        try:
            with open(path, 'r') as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                    state = VehicleState()
                    state.x = float(row['x'])
                    state.y = float(row['y'])
                    state.v = float(row['v'])

                    # Handle theta (heading angle) - this is what MPC needs
                    if 'theta' in row:
                        state.theta = float(row['theta'])
                    else:
                        # Calculate theta from the data if not provided
                        state.theta = 0.0  # Will be calculated later

                    # MPC controller computes steering angles, so set to 0.0
                    state.delta = 0.0
                    data.append(state)

            # If theta was not provided, calculate it from consecutive points
            if len(data) > 1 and data[0].theta == 0.0:
                for i in range(len(data)):
                    if i < len(data) - 1:
                        dx = data[i + 1].x - data[i].x
                        dy = data[i + 1].y - data[i].y
                        data[i].theta = math.atan2(dy, dx)
                    else:
                        # For the last point, use the same theta as the previous point
                        data[i].theta = data[i - 1].theta

            # Validate all loaded trajectory points
            valid_points = []
            invalid_count = 0
            for i, state in enumerate(data):
                if self._validate_vehicle_state(state):
                    valid_points.append(state)
                else:
                    invalid_count += 1
                    self.get_logger().warn(f"Invalid trajectory point at index {i}: "
                                           f"x={state.x}, y={state.y}, v={state.v}, theta={state.theta}")

            if invalid_count > 0:
                self.get_logger().warn(f"Removed {invalid_count} invalid trajectory points")

            self.log_info(f"Loaded {len(valid_points)} valid trajectory points (removed {invalid_count} invalid)")
            return valid_points

        except Exception as e:
            self.get_logger().error(f"Failed to load trajectory: {str(e)}")
        return data

    def find_closest_trajectory_point(self):
        """Find the closest point on the reference trajectory to current position"""
        if not self.reference_trajectory:
            return 0

        min_distance = float('inf')
        closest_index = 0

        current_x = self.current_vehicle_state.x
        current_y = self.current_vehicle_state.y

        for i, point in enumerate(self.reference_trajectory):
            distance = np.sqrt((point.x - current_x)**2 + (point.y - current_y)**2)
            if distance < min_distance:
                min_distance = distance
                closest_index = i

        return closest_index

    def create_predictive_trajectory(self):
        """Create predictive trajectory horizon starting from closest reference point"""
        try:
            if not self.reference_trajectory:
                self.get_logger().warn("No reference trajectory available")
                return VehicleStateArray()

            # Validate current vehicle state before using it
            if not self._validate_vehicle_state(self.current_vehicle_state):
                self.get_logger().error(f"Current vehicle state contains invalid values: "
                                        f"x={self.current_vehicle_state.x}, y={self.current_vehicle_state.y}, "
                                        f"v={self.current_vehicle_state.v}, theta={self.current_vehicle_state.theta}")
                return VehicleStateArray()

            # Find closest point on reference trajectory
            closest_index = self.find_closest_trajectory_point()

            # Create trajectory array for MPC horizon
            trajectory_msg = VehicleStateArray()

            # Add reference trajectory points for the prediction horizon
            for i in range(self.horizon):
                ref_index = (closest_index + i) % len(self.reference_trajectory)
                ref_point = self.reference_trajectory[ref_index]

                # Validate reference point before using it
                if not self._validate_vehicle_state(ref_point):
                    self.get_logger().error(
                        f"Reference point {ref_index} contains invalid values: "
                        f"x={ref_point.x}, y={ref_point.y}, v={ref_point.v}, theta={ref_point.theta}")
                    return VehicleStateArray()

                # Create predicted state
                predicted_state = VehicleState()
                predicted_state.x = ref_point.x
                predicted_state.y = ref_point.y
                predicted_state.v = ref_point.v
                predicted_state.theta = ref_point.theta  # Include heading angle - critical for MPC!
                predicted_state.delta = 0.0  # MPC computes steering, not trajectory publisher

                trajectory_msg.states.append(predicted_state)

            return trajectory_msg

        except Exception as e:
            self.get_logger().error(f"Error creating predictive trajectory: {e}")
            import traceback
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")
            return VehicleStateArray()

    def _validate_vehicle_state(self, state):
        """Validate that a vehicle state contains finite, reasonable values"""
        try:
            # Check for NaN or infinite values
            if not (np.isfinite(state.x) and np.isfinite(state.y) and
                    np.isfinite(state.v) and np.isfinite(state.theta)):
                return False

            # Check for reasonable ranges (same as MPC node validation)
            if abs(state.x) > 1000 or abs(state.y) > 1000:  # Position check
                return False

            if state.v < -2 or state.v > 25:  # Velocity check (same as MPC: -2 to 25 m/s)
                return False

            # Theta should be reasonable (same as MPC: allows up to 10 radians)
            if abs(state.theta) > 10:  # Allow some flexibility for unnormalized angles
                return False

            return True

        except Exception as e:
            self.get_logger().error(f"Error validating vehicle state: {e}")
            return False

    def create_path_msg(self):
        """Create Path message for RViz visualization"""
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for state in self.reference_trajectory:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = state.x
            pose.pose.position.y = state.y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # No rotation for simplicity
            path_msg.poses.append(pose)

        return path_msg

    def publish_predictive_trajectory(self):
        """Publish predictive trajectory for MPC and status"""
        # Always publish status
        status_msg = Bool()
        status_msg.data = self.path_ready
        self.status_pub.publish(status_msg)

        # Only publish trajectory data if ready and we have current state
        if self.path_ready and len(self.reference_trajectory) > 0:
            # Publish full reference path for RViz (less frequent)
            if self.get_clock().now().nanoseconds % 1000000000 < 100000000:  # ~10% of the time
                path_msg = self.create_path_msg()
                self.path_pub.publish(path_msg)
                self.log_debug("Published reference path for RViz visualization")

            # Always publish predictive trajectory for MPC
            trajectory_msg = self.create_predictive_trajectory()
            if len(trajectory_msg.states) > 0:
                # Double-check that all states in the trajectory are valid
                valid_trajectory = True
                for i, state in enumerate(trajectory_msg.states):
                    if not self._validate_vehicle_state(state):
                        self.get_logger().error(f"Invalid state at index {i} in trajectory, not publishing")
                        valid_trajectory = False
                        break

                if valid_trajectory:
                    self.trajectory_pub.publish(trajectory_msg)
                    self.log_debug(
                        f"Published valid predictive trajectory: {len(trajectory_msg.states)} points, "
                        f"current pos: ({self.current_vehicle_state.x:.2f}, {self.current_vehicle_state.y:.2f}), "
                        f"current theta: {self.current_vehicle_state.theta:.2f}, "
                        f"closest ref idx: {self.find_closest_trajectory_point()}")

                    # Debug: Log first few trajectory points being sent to MPC
                    if self.enable_logging and len(trajectory_msg.states) > 0:
                        self.log_debug("Sample trajectory states sent to MPC:")
                        for i in range(min(3, len(trajectory_msg.states))):
                            state = trajectory_msg.states[i]
                            self.log_debug(
                                f"  State {i}: x={state.x:.2f}, y={state.y:.2f}, v={state.v:.2f}, theta={state.theta:.2f}")
                else:
                    self.get_logger().warn("Skipping trajectory publication due to invalid states")
            else:
                self.get_logger().warn("Empty trajectory created, not publishing")

    # Enhanced mode methods

    def _bound_adjustment_callback(self, msg: BoundAdjustment):
        """Handle bound adjustment messages from MPC (enhanced mode only)"""
        if not self.enhanced_mode or not self.adaptive_bounds:
            return

        if msg.reset_to_default:
            self.left_bound_adjustment = 0.0
            self.right_bound_adjustment = 0.0
            self.confidence_adjustment = 0.0
            self.get_logger().info("Reset bound adjustments to default")
        else:
            self.left_bound_adjustment = msg.left_bound_adjustment
            self.right_bound_adjustment = msg.right_bound_adjustment
            self.confidence_adjustment = msg.confidence_adjustment

            self.get_logger().info(f"Updated bound adjustments: "
                                   f"left={self.left_bound_adjustment:.3f}, "
                                   f"right={self.right_bound_adjustment:.3f}, "
                                   f"confidence={self.confidence_adjustment:.3f}")

    def _find_closest_point_index(self) -> int:
        """Find the closest trajectory point to current position (enhanced mode)"""
        if not self.enhanced_mode or not hasattr(self, 'trajectory_points'):
            return self.find_closest_trajectory_point()

        if not self.path_ready or self.current_pose is None:
            return 0

        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        min_distance = float('inf')
        closest_index = 0

        for i, point in enumerate(self.trajectory_points):
            distance = math.sqrt((point['x'] - current_x)**2 + (point['y'] - current_y)**2)
            if distance < min_distance:
                min_distance = distance
                closest_index = i

        return closest_index

    def _compute_path_normal(self, index: int) -> Tuple[float, float]:
        """Compute the normal vector to the path at given index"""
        if not hasattr(self, 'trajectory_points') or index >= len(self.trajectory_points) - 1:
            return 0.0, 1.0

        # Use finite differences to compute tangent
        if index > 0:
            dx = self.trajectory_points[index + 1]['x'] - self.trajectory_points[index - 1]['x']
            dy = self.trajectory_points[index + 1]['y'] - self.trajectory_points[index - 1]['y']
        else:
            dx = self.trajectory_points[index + 1]['x'] - self.trajectory_points[index]['x']
            dy = self.trajectory_points[index + 1]['y'] - self.trajectory_points[index]['y']

        # Normalize tangent
        length = math.sqrt(dx**2 + dy**2)
        if length > 0:
            dx /= length
            dy /= length

        # Normal is perpendicular to tangent (rotated 90 degrees)
        return -dy, dx

    def _compute_adaptive_bounds(self, index: int, velocity: float) -> Tuple[float, float, float]:
        """Compute adaptive bounds based on velocity and curvature"""
        base_left = self.default_left_bound + self.left_bound_adjustment
        base_right = self.default_right_bound + self.right_bound_adjustment
        base_confidence = self.default_confidence + self.confidence_adjustment

        # Clamp confidence to valid range
        base_confidence = max(0.0, min(1.0, base_confidence))

        # Velocity-based scaling (tighter bounds at higher speeds)
        velocity_factor = min(velocity / 5.0, 1.0)  # Scale up to 5 m/s

        # Curvature-based scaling
        curvature_factor = 1.0
        if hasattr(self, 'trajectory_points') and index > 0 and index < len(self.trajectory_points) - 1:
            # Approximate curvature using three points
            p1 = self.trajectory_points[index - 1]
            p2 = self.trajectory_points[index]
            p3 = self.trajectory_points[index + 1]

            # Compute approximate curvature
            dx1, dy1 = p2['x'] - p1['x'], p2['y'] - p1['y']
            dx2, dy2 = p3['x'] - p2['x'], p3['y'] - p2['y']

            cross_product = dx1 * dy2 - dy1 * dx2
            curvature = abs(cross_product) / (math.sqrt(dx1**2 + dy1**2) * math.sqrt(dx2**2 + dy2**2) + 1e-6)

            # Scale bounds tighter for high curvature
            curvature_factor = max(0.5, 1.0 - curvature * 2.0)

        # Apply scaling factors
        left_bound = base_left * curvature_factor * (1.0 + 0.3 * velocity_factor)
        right_bound = base_right * curvature_factor * (1.0 + 0.3 * velocity_factor)
        confidence = base_confidence * (1.0 + 0.2 * velocity_factor)

        return left_bound, right_bound, confidence

    def _publish_enhanced_trajectory(self):
        """Publish enhanced trajectory with constraints"""
        if not self.enhanced_mode or not hasattr(self, 'trajectory_points') or len(self.trajectory_points) == 0:
            return

        # Find starting index
        start_index = self._find_closest_point_index()

        # Create constrained trajectory message
        constrained_msg = ConstrainedVehicleStateArray()
        constrained_msg.header.stamp = self.get_clock().now().to_msg()
        constrained_msg.header.frame_id = self.map_frame

        # Generate horizon points
        for i in range(self.horizon):
            point_index = (start_index + i) % len(self.trajectory_points)
            traj_point = self.trajectory_points[point_index]

            # Create constrained state
            state = ConstrainedVehicleState()
            state.x = traj_point['x']
            state.y = traj_point['y']
            state.theta = traj_point['theta']
            state.v_ref = max(traj_point['v'], self.min_speed)

            # Compute adaptive bounds
            left_bound, right_bound, confidence = self._compute_adaptive_bounds(
                point_index, state.v_ref)

            state.left_bound = left_bound
            state.right_bound = right_bound
            state.confidence = confidence

            constrained_msg.states.append(state)

        # Publish constrained trajectory
        self.constrained_trajectory_pub.publish(constrained_msg)

        # Publish constraint status
        self._publish_constraint_status(constrained_msg.states)

    def _publish_constraint_status(self, states: List[ConstrainedVehicleState]):
        """Publish constraint status"""
        if not self.enhanced_mode:
            return

        status_msg = ConstraintStatus()
        status_msg.header.stamp = self.get_clock().now().to_msg()
        status_msg.header.frame_id = self.map_frame

        status_msg.constraints_active = self.constraints_active
        status_msg.bounds_violated = self.bounds_violated
        status_msg.adapting = self.adaptive_bounds

        if states:
            status_msg.avg_left_margin = sum(s.left_bound for s in states) / len(states)
            status_msg.avg_right_margin = sum(s.right_bound for s in states) / len(states)
            status_msg.avg_confidence = sum(s.confidence for s in states) / len(states)

        status_msg.status_message = f"Active: {status_msg.constraints_active}, " \
            f"Violated: {status_msg.bounds_violated}, " \
            f"Adapting: {status_msg.adapting}"

        self.constraint_status_pub.publish(status_msg)

    def _publish_visualization(self):
        """Publish advanced visualization markers (enhanced mode only)"""
        if not self.enhanced_mode or not self.publish_visualization or not hasattr(self, 'trajectory_points'):
            return

        if not self.path_ready or len(self.trajectory_points) == 0:
            return

        self._publish_corridor_visualization()
        self._publish_velocity_visualization()

    def _publish_corridor_visualization(self):
        """Publish corridor visualization with bounds"""
        marker_array = MarkerArray()

        # Find current trajectory segment
        start_index = self._find_closest_point_index()

        # Create corridor markers
        for i in range(min(self.horizon, len(self.trajectory_points) - start_index)):
            point_index = start_index + i
            traj_point = self.trajectory_points[point_index]

            # Compute bounds
            left_bound, right_bound, confidence = self._compute_adaptive_bounds(
                point_index, traj_point['v'])

            # Compute normal vector
            normal_x, normal_y = self._compute_path_normal(point_index)

            # Left boundary marker
            left_marker = Marker()
            left_marker.header.stamp = self.get_clock().now().to_msg()
            left_marker.header.frame_id = self.map_frame
            left_marker.ns = "corridor_left"
            left_marker.id = i
            left_marker.type = Marker.SPHERE
            left_marker.action = Marker.ADD

            left_marker.pose.position.x = traj_point['x'] + normal_x * left_bound
            left_marker.pose.position.y = traj_point['y'] + normal_y * left_bound
            left_marker.pose.position.z = 0.0

            left_marker.scale.x = 0.1
            left_marker.scale.y = 0.1
            left_marker.scale.z = 0.1

            left_marker.color.r = 1.0
            left_marker.color.g = 0.0
            left_marker.color.b = 0.0
            left_marker.color.a = confidence

            marker_array.markers.append(left_marker)

            # Right boundary marker
            right_marker = Marker()
            right_marker.header.stamp = self.get_clock().now().to_msg()
            right_marker.header.frame_id = self.map_frame
            right_marker.ns = "corridor_right"
            right_marker.id = i
            right_marker.type = Marker.SPHERE
            right_marker.action = Marker.ADD

            right_marker.pose.position.x = traj_point['x'] - normal_x * right_bound
            right_marker.pose.position.y = traj_point['y'] - normal_y * right_bound
            right_marker.pose.position.z = 0.0

            right_marker.scale = left_marker.scale
            right_marker.color.r = 0.0
            right_marker.color.g = 0.0
            right_marker.color.b = 1.0
            right_marker.color.a = confidence

            marker_array.markers.append(right_marker)

            # Centerline marker
            center_marker = Marker()
            center_marker.header.stamp = self.get_clock().now().to_msg()
            center_marker.header.frame_id = self.map_frame
            center_marker.ns = "centerline"
            center_marker.id = i
            center_marker.type = Marker.SPHERE
            center_marker.action = Marker.ADD

            center_marker.pose.position.x = traj_point['x']
            center_marker.pose.position.y = traj_point['y']
            center_marker.pose.position.z = 0.0

            center_marker.scale.x = 0.05
            center_marker.scale.y = 0.05
            center_marker.scale.z = 0.05

            center_marker.color.r = 0.0
            center_marker.color.g = 1.0
            center_marker.color.b = 0.0
            center_marker.color.a = 1.0

            marker_array.markers.append(center_marker)

        self.corridor_viz_pub.publish(marker_array)

    def _velocity_to_color(self, velocity: float) -> ColorRGBA:
        """Convert velocity to HSV color gradient"""
        # Normalize velocity to [0, 1]
        normalized_vel = min(velocity / self.velocity_color_scale, 1.0)

        # Map to HSV: blue (240°) for low speed, red (0°) for high speed
        hue = (1.0 - normalized_vel) * 240.0 / 360.0  # Convert to [0, 1] range
        saturation = 1.0
        value = 1.0

        # Convert HSV to RGB
        r, g, b = colorsys.hsv_to_rgb(hue, saturation, value)

        color = ColorRGBA()
        color.r = float(r)
        color.g = float(g)
        color.b = float(b)
        color.a = 1.0

        return color

    def _publish_velocity_visualization(self):
        """Publish velocity-colored path visualization"""
        marker_array = MarkerArray()

        # Create line strip marker for velocity path
        line_marker = Marker()
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.header.frame_id = self.map_frame
        line_marker.ns = "velocity_path"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD

        line_marker.scale.x = 0.1  # Line width
        line_marker.color.r = 1.0
        line_marker.color.g = 1.0
        line_marker.color.b = 1.0
        line_marker.color.a = 1.0

        # Add points with velocity-based height offset
        for i, point in enumerate(self.trajectory_points[::5]):  # Sample every 5th point
            p = Point()
            p.x = point['x']
            p.y = point['y']
            p.z = point['v'] * 0.1  # Height proportional to velocity
            line_marker.points.append(p)

            # Add color for each point
            color = self._velocity_to_color(point['v'])
            line_marker.colors.append(color)

        marker_array.markers.append(line_marker)

        # Create sphere markers for better visibility
        for i, point in enumerate(self.trajectory_points[::10]):  # Sample every 10th point
            sphere_marker = Marker()
            sphere_marker.header.stamp = self.get_clock().now().to_msg()
            sphere_marker.header.frame_id = self.map_frame
            sphere_marker.ns = "velocity_spheres"
            sphere_marker.id = i
            sphere_marker.type = Marker.SPHERE
            sphere_marker.action = Marker.ADD

            sphere_marker.pose.position.x = point['x']
            sphere_marker.pose.position.y = point['y']
            sphere_marker.pose.position.z = point['v'] * 0.1

            sphere_marker.scale.x = 0.2
            sphere_marker.scale.y = 0.2
            sphere_marker.scale.z = 0.2

            sphere_marker.color = self._velocity_to_color(point['v'])

            marker_array.markers.append(sphere_marker)

        self.velocity_path_pub.publish(marker_array)

    def publish_predictive_trajectory(self):
        """Unified trajectory publishing method"""
        # Always publish legacy trajectory for backward compatibility
        self._publish_legacy_trajectory()

        # Publish enhanced trajectory if in enhanced mode
        if self.enhanced_mode:
            self._publish_enhanced_trajectory()

    def _publish_legacy_trajectory(self):
        """Publish legacy trajectory (original horizon mapper behavior)"""
        # Always publish status
        status_msg = Bool()
        status_msg.data = self.path_ready
        self.status_pub.publish(status_msg)

        # Only publish trajectory data if ready and we have current state
        if self.path_ready and len(self.reference_trajectory) > 0:
            # Publish full reference path for RViz (less frequent)
            if self.get_clock().now().nanoseconds % 1000000000 < 100000000:  # ~10% of the time
                path_msg = self.create_path_msg()
                self.path_pub.publish(path_msg)
                self.log_debug("Published reference path for RViz visualization")

            # Always publish predictive trajectory for MPC
            trajectory_msg = self.create_predictive_trajectory()
            if len(trajectory_msg.states) > 0:
                # Double-check that all states in the trajectory are valid
                valid_trajectory = True
                for i, state in enumerate(trajectory_msg.states):
                    if not self._validate_vehicle_state(state):
                        self.get_logger().error(f"Invalid state at index {i} in trajectory, not publishing")
                        valid_trajectory = False
                        break

                if valid_trajectory:
                    self.trajectory_pub.publish(trajectory_msg)
                    self.log_debug(
                        f"Published valid predictive trajectory: {len(trajectory_msg.states)} points, "
                        f"current pos: ({self.current_vehicle_state.x:.2f}, {self.current_vehicle_state.y:.2f}), "
                        f"current theta: {self.current_vehicle_state.theta:.2f}, "
                        f"closest ref idx: {self.find_closest_trajectory_point()}")

                    # Debug: Log first few trajectory points being sent to MPC
                    if self.enable_logging and len(trajectory_msg.states) > 0:
                        self.log_debug("Sample trajectory states sent to MPC:")
                        for i in range(min(3, len(trajectory_msg.states))):
                            state = trajectory_msg.states[i]
                            self.log_debug(
                                f"  State {i}: x={state.x:.2f}, y={state.y:.2f}, v={state.v:.2f}, theta={state.theta:.2f}")
                else:
                    self.get_logger().warn("Skipping trajectory publication due to invalid states")
            else:
                self.get_logger().warn("Empty trajectory created, not publishing")
        else:
            if not self.path_ready:
                self.log_debug("Path not ready, not publishing trajectory")
            if len(self.reference_trajectory) == 0:
                self.log_debug("No reference trajectory loaded, not publishing")


def main(args=None):
    rclpy.init(args=args)
    node = HorizonMapperNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.log_info("Horizon Mapper node shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    main()
