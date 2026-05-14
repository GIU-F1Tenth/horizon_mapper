from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Bool, Header
from tf2_ros import Buffer, TransformListener
from std_srvs.srv import Trigger
from std_msgs.msg import Int16
from giu_f1t_interfaces.msg import (
    VehicleState,
    VehicleStateArray,
    BoundAdjustment,
)
from sensor_msgs.msg import LaserScan
from .preprocess_trajectory import preprocess_trajectory
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import csv
import math
import numpy as np
import colorsys
import os
from typing import List, Tuple, Optional
try:
    from tf_transformations import euler_from_quaternion, quaternion_from_euler
except ImportError:
    def euler_from_quaternion(quat):
        """Fallback conversion: quaternion [x, y, z, w] -> roll, pitch, yaw."""
        x, y, z, w = quat

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def quaternion_from_euler(roll, pitch, yaw):
        """Fallback conversion: roll, pitch, yaw -> quaternion [x, y, z, w]."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return [x, y, z, w]
from ament_index_python.packages import get_package_share_directory
from .hm_viz import HorizonMapperViz

# Default configuration class - provides fallback values


class DefaultConfig:
    def __init__(self):
        self.enable_trajectory_generation = True
        self.horizon_N = 8
        self.wheelbase = 0.33
        self.max_steering_angle = 0.5
        self.min_speed = 0.1
        self.odom_topic = "car_state/odom"
        self.path_topic = "/path"
        self.enable_logging = True
        self.enable_debugging = False
        self.use_csv_path = False
        self.csv_input_path = ""
        self.csv_output_path = ""


default_config = DefaultConfig()


class HorizonMapperNode(Node):
    """
    Horizon Mapper Node

    Publishes reference trajectories with predictive horizon for model-based controllers.
    Supports constrained trajectories with adaptive bounds, visualization, and runtime adjustments.
    """

    def __init__(self):
        super().__init__('horizon_mapper_node')

        # Declare ROS2 parameters
        self._declare_parameters()
        self._load_parameters()

        self._viz = HorizonMapperViz(self) if self.publish_visualization else None

        # Initialize state variables
        self._initialize_state_variables()

        # Initialize TF2 for visualization
        if self.publish_visualization:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create subscribers
        self._create_subscribers()

        # Create publishers
        self._create_publishers()

        # Create timers
        self._create_timers()

        # Service: allow external controller to request trajectory reload/resample
        self.override_service = self.create_service(Trigger, '/horizon_mapper/override_path', self._override_service_cb)

        if self.use_csv_path:
            self.get_logger().info("Horizon Mapper Node initialized - path source: CSV file")
        else:
            self.get_logger().info(f"Horizon Mapper Node initialized - waiting for {self.path_topic} topic")

    def _declare_parameters(self):
        """Declare all ROS2 parameters with descriptions"""

        # Core parameters
        self.declare_parameter('enable_logging', default_config.enable_logging)
        self.declare_parameter('enable_debugging', default_config.enable_debugging)
        self.declare_parameter('path_topic', default_config.path_topic,
                               ParameterDescriptor(description='Topic to subscribe for input trajectory path'))
        self.declare_parameter('left_boundary_topic', '/left_boundary',
                               ParameterDescriptor(description='Topic for left boundary path'))
        self.declare_parameter('right_boundary_topic', '/right_boundary',
                               ParameterDescriptor(description='Topic for right boundary path'))
        self.declare_parameter('horizon_N', default_config.horizon_N)
        self.declare_parameter('horizon_T', 0.5)
        self.declare_parameter('wheelbase', default_config.wheelbase)
        self.declare_parameter('max_steering_angle', default_config.max_steering_angle)
        self.declare_parameter('min_speed', default_config.min_speed)
        self.declare_parameter('odom_topic', default_config.odom_topic)

        # Topic parameters
        self.declare_parameter('reference_path_topic', '/horizon_mapper/reference_path',
                               ParameterDescriptor(description='Topic for publishing reference path'))
        self.declare_parameter('reference_trajectory_topic', '/horizon_mapper/reference_trajectory',
                               ParameterDescriptor(description='Topic for publishing reference trajectory'))
        self.declare_parameter('status_topic', '/horizon_mapper/path_ready',
                               ParameterDescriptor(description='Topic for publishing path ready status'))
        self.declare_parameter('constrained_trajectory_topic', '/horizon_mapper/constrained_trajectory',
                               ParameterDescriptor(description='Topic for publishing constrained trajectory'))
        self.declare_parameter('constraint_status_topic', '/horizon_mapper/constraint_status',
                               ParameterDescriptor(description='Topic for publishing constraint status'))
        self.declare_parameter('corridor_viz_topic', '/horizon_mapper/corridor_visualization',
                               ParameterDescriptor(description='Topic for corridor visualization'))
        self.declare_parameter('velocity_viz_topic', '/horizon_mapper/velocity_path',
                               ParameterDescriptor(description='Topic for velocity visualization'))
        self.declare_parameter('bound_adjustment_topic', '/ctrl/bound_adjustments',
                               ParameterDescriptor(description='Topic for receiving bound adjustments'))
        self.declare_parameter('scan_topic', '/scan',
                               ParameterDescriptor(description='Topic for LIDAR scan data'))
        self.declare_parameter('initialpose_topic', '/initialpose',
                               ParameterDescriptor(description='Topic for initial pose estimates'))
        self.declare_parameter('horizon_topic', 'ctrl/dynamic/horizon_n',
                               ParameterDescriptor(description='Topic for dynamic horizon updates'))

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
        self.declare_parameter('qos_depth', 10,
                               ParameterDescriptor(description='QoS queue depth for topics'))
        self.declare_parameter('reverse_direction', False,
                               ParameterDescriptor(description='If true, traverse the trajectory in reverse (counter-direction)'))

        # Adaptive track geometry – tune per map
        self.declare_parameter('track_width', 0.0,
                               ParameterDescriptor(
                                   description='Total driveable track width [m]. '
                                               'When > 0 overrides default_left_bound and default_right_bound '
                                               '(sets each to track_width / 2). Use 0 to keep manual bounds.'))
        # CSV path source parameters
        self.declare_parameter('use_csv_path', default_config.use_csv_path,
                               ParameterDescriptor(description='If true, load trajectory from CSV file instead of subscribing to path topic'))
        self.declare_parameter('csv_input_path', default_config.csv_input_path,
                               ParameterDescriptor(description='Path to input CSV trajectory file (used when use_csv_path=true)'))
        self.declare_parameter('csv_output_path', default_config.csv_output_path,
                               ParameterDescriptor(description='Path to write preprocessed CSV trajectory (used when use_csv_path=true)'))

    def _load_parameters(self):
        """Load parameters from parameter server"""
        # Core parameters
        self.enable_logging = self.get_parameter('enable_logging').value
        self.enable_debugging = self.get_parameter('enable_debugging').value
        self.path_topic = self.get_parameter('path_topic').value
        self.left_boundary_topic = self.get_parameter('left_boundary_topic').value
        self.right_boundary_topic = self.get_parameter('right_boundary_topic').value
        self.horizon = self.get_parameter('horizon_N').value
        self.horizon_T = self.get_parameter('horizon_T').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_steering = self.get_parameter('max_steering_angle').value
        self.min_speed = self.get_parameter('min_speed').value
        self.odom_topic = self.get_parameter('odom_topic').value

        # Topic parameters
        self.reference_path_topic = self.get_parameter('reference_path_topic').value
        self.reference_trajectory_topic = self.get_parameter('reference_trajectory_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.constrained_trajectory_topic = self.get_parameter('constrained_trajectory_topic').value
        self.constraint_status_topic = self.get_parameter('constraint_status_topic').value
        self.corridor_viz_topic = self.get_parameter('corridor_viz_topic').value
        self.velocity_viz_topic = self.get_parameter('velocity_viz_topic').value
        self.bound_adjustment_topic = self.get_parameter('bound_adjustment_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.initialpose_topic = self.get_parameter('initialpose_topic').value
        self.horizon_topic = self.get_parameter('horizon_topic').value

        # Constraint and visualization parameters
        self.default_left_bound = self.get_parameter('default_left_bound').value
        self.default_right_bound = self.get_parameter('default_right_bound').value
        self.default_confidence = self.get_parameter('default_confidence').value
        self.adaptive_bounds = self.get_parameter('adaptive_bounds').value
        self.publish_visualization = self.get_parameter('publish_visualization').value
        self.velocity_color_scale = self.get_parameter('velocity_color_scale').value
        self.map_frame = self.get_parameter('map_frame').value
        self.qos_depth = self.get_parameter('qos_depth').value
        self.reverse_direction = self.get_parameter('reverse_direction').value

        # Adaptive track geometry
        track_width = self.get_parameter('track_width').value
        if track_width > 0.0:
            half = track_width / 2.0
            self.default_left_bound = half
            self.default_right_bound = half

        # CSV path source parameters
        self.use_csv_path = self.get_parameter('use_csv_path').value
        self.csv_input_path = self.get_parameter('csv_input_path').value
        self.csv_output_path = self.get_parameter('csv_output_path').value

        self.csv_input_path = self._resolve_csv_path(self.csv_input_path)
        self.csv_output_path = self._resolve_csv_path(self.csv_output_path, prefer_output=True)

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
        self.csv_retry_timer = None
        
        # Boundary arrays from external source
        self.received_left_bounds = []
        self.received_right_bounds = []
        self.use_received_bounds = False

        # Visualization and constraint state variables
        if self.publish_visualization:
            self.trajectory_points = []
            # Bound adjustments
            self.left_bound_adjustment = 0.0
            self.right_bound_adjustment = 0.0
            self.confidence_adjustment = 0.0

            # Status tracking
            self.bounds_violated = False
            self.constraints_active = True

    def _create_subscribers(self):
        """Create ROS2 subscribers"""        
        # Core subscribers
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odometry_callback, qos_profile_sensor_data)

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, self.initialpose_topic, self.pose_estimate_callback, self.qos_depth)

        self.lidar_scan = None
        self.lidar_scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self._lidar_callback, self.qos_depth)
        
        self.horizon_sub = self.create_subscription(
            Int16, self.horizon_topic, self.horizon_callback, self.qos_depth)
        
        if not self.use_csv_path:
            self.path_sub = self.create_subscription(
                Path, self.path_topic, self._path_callback, self.qos_depth)

        self.left_boundary_sub = self.create_subscription(
            Path, self.left_boundary_topic, self._left_boundary_callback, self.qos_depth)
        
        self.right_boundary_sub = self.create_subscription(
            Path, self.right_boundary_topic, self._right_boundary_callback, self.qos_depth)

        # Adaptive bounds subscriber
        if self.publish_visualization and self.adaptive_bounds:
            self.bound_adjustment_sub = self.create_subscription(
                BoundAdjustment, self.bound_adjustment_topic, self._bound_adjustment_callback, self.qos_depth)
        
        self.get_logger().info(f"Subscribed to topics:")
        if not self.use_csv_path:
            self.get_logger().info(f"  - Reference path: {self.path_topic}")
        else:
            self.get_logger().info(f"  - Reference path: CSV file ({self.csv_input_path})")
        self.get_logger().info(f"  - Left boundary: {self.left_boundary_topic}")
        self.get_logger().info(f"  - Right boundary: {self.right_boundary_topic}")
        self.get_logger().info(f"  - Odometry: {self.odom_topic}")

    def _lidar_callback(self, msg):
        """Store latest LIDAR scan"""
        self.lidar_scan = msg
    
    def _path_callback(self, msg: Path):
        """Callback to process incoming Path message and convert to reference trajectory"""
        try:
            if len(msg.poses) == 0:
                self.get_logger().warn("Received empty path message")
                return
            
            self.get_logger().info(f"Received path with {len(msg.poses)} poses")
            
            # Convert Path message to reference trajectory format
            new_trajectory = []
            for i, pose_stamped in enumerate(msg.poses):
                state = VehicleState()
                state.x = pose_stamped.pose.position.x
                state.y = pose_stamped.pose.position.y
                
                # Extract theta from quaternion
                orientation = pose_stamped.pose.orientation
                _, _, yaw = euler_from_quaternion([
                    orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w
                ])
                state.theta = yaw
                
                # Calculate velocity from spacing (simple approximation)
                if i > 0:
                    prev_state = new_trajectory[-1]
                    dx = state.x - prev_state.x
                    dy = state.y - prev_state.y
                    distance = math.sqrt(dx**2 + dy**2)
                    # Assume constant time step or use min_speed as default
                    state.v = max(distance / self.horizon_T, self.min_speed)
                else:
                    state.v = self.min_speed
                
                state.delta = 0.0  # Steering will be computed by MPC
                new_trajectory.append(state)
            
            # Update reference trajectory
            self.reference_trajectory = new_trajectory
            
            # Check if we have matching boundaries
            self._validate_boundaries()
            
            # Update trajectory points for visualization
            if self.publish_visualization:
                self.trajectory_points = []
                for state in new_trajectory:
                    point = {
                        'x': state.x,
                        'y': state.y,
                        'v': state.v,
                        'theta': state.theta
                    }
                    self.trajectory_points.append(point)
            
            self.path_ready = True
            boundary_status = 'received' if self.use_received_bounds else 'adaptive'
            self.get_logger().info(f"Updated reference trajectory: {len(self.reference_trajectory)} points, "
                                  f"boundaries: {boundary_status}")
            
        except Exception as e:
            self.get_logger().error(f"Error processing path message: {e}")
    
    def _left_boundary_callback(self, msg: Path):
        """Callback for left boundary path"""
        try:
            if len(msg.poses) == 0:
                self.get_logger().warn("Received empty left boundary path")
                return
            
            # Extract lateral distances from boundary path
            self.received_left_bounds = []
            
            if not self.reference_trajectory:
                # If no reference trajectory yet, store raw distances
                for pose_stamped in msg.poses:
                    # Calculate distance from origin or use provided value
                    x = pose_stamped.pose.position.x
                    y = pose_stamped.pose.position.y
                    distance = math.sqrt(x**2 + y**2)
                    self.received_left_bounds.append(distance)
            else:
                # Calculate perpendicular distance from reference trajectory
                for i, pose_stamped in enumerate(msg.poses):
                    if i < len(self.reference_trajectory):
                        # Get boundary point
                        bound_x = pose_stamped.pose.position.x
                        bound_y = pose_stamped.pose.position.y
                        
                        # Get corresponding reference point
                        ref_x = self.reference_trajectory[i].x
                        ref_y = self.reference_trajectory[i].y
                        
                        # Calculate perpendicular distance
                        distance = math.sqrt((bound_x - ref_x)**2 + (bound_y - ref_y)**2)
                        self.received_left_bounds.append(distance)
                    else:
                        # Fallback for mismatched lengths
                        self.received_left_bounds.append(self.default_left_bound)
            
            self.get_logger().info(f"Received left boundary: {len(self.received_left_bounds)} points")
            self._validate_boundaries()
            
        except Exception as e:
            self.get_logger().error(f"Error processing left boundary: {e}")
    
    def _right_boundary_callback(self, msg: Path):
        """Callback for right boundary path"""
        try:
            if len(msg.poses) == 0:
                self.get_logger().warn("Received empty right boundary path")
                return
            
            # Extract lateral distances from boundary path
            self.received_right_bounds = []
            
            if not self.reference_trajectory:
                # If no reference trajectory yet, store raw distances
                for pose_stamped in msg.poses:
                    # Calculate distance from origin or use provided value
                    x = pose_stamped.pose.position.x
                    y = pose_stamped.pose.position.y
                    distance = math.sqrt(x**2 + y**2)
                    self.received_right_bounds.append(distance)
            else:
                # Calculate perpendicular distance from reference trajectory
                for i, pose_stamped in enumerate(msg.poses):
                    if i < len(self.reference_trajectory):
                        # Get boundary point
                        bound_x = pose_stamped.pose.position.x
                        bound_y = pose_stamped.pose.position.y
                        
                        # Get corresponding reference point
                        ref_x = self.reference_trajectory[i].x
                        ref_y = self.reference_trajectory[i].y
                        
                        # Calculate perpendicular distance
                        distance = math.sqrt((bound_x - ref_x)**2 + (bound_y - ref_y)**2)
                        self.received_right_bounds.append(distance)
                    else:
                        # Fallback for mismatched lengths
                        self.received_right_bounds.append(self.default_right_bound)
            
            self.get_logger().info(f"Received right boundary: {len(self.received_right_bounds)} points")
            self._validate_boundaries()
            
        except Exception as e:
            self.get_logger().error(f"Error processing right boundary: {e}")
    
    def _validate_boundaries(self):
        """Validate that trajectory and boundaries have matching lengths"""
        if not self.reference_trajectory:
            self.use_received_bounds = False
            return
        
        traj_len = len(self.reference_trajectory)
        left_len = len(self.received_left_bounds)
        right_len = len(self.received_right_bounds)
        
        if left_len == traj_len and right_len == traj_len:
            self.use_received_bounds = True
            self.get_logger().info(f"Boundaries validated: {traj_len} points match")
        elif left_len > 0 or right_len > 0:
            self.use_received_bounds = False
            self.get_logger().warn(f"Boundary length mismatch - trajectory: {traj_len}, "
                                  f"left: {left_len}, right: {right_len}. Using adaptive bounds.")
        else:
            self.use_received_bounds = False

    def preprocess_and_load_trajectory(self):
        """Run preprocessing on CSV file and load trajectory (used when use_csv_path=true)"""
        if not self.csv_input_path or not self.csv_output_path:
            self.get_logger().error("use_csv_path is true but csv_input_path or csv_output_path is not set")
            return

        if not os.path.isfile(self.csv_input_path):
            self.get_logger().warn(
                f"CSV input not found yet: {self.csv_input_path}. Waiting for file generation..."
            )
            if self.csv_retry_timer is None:
                self.csv_retry_timer = self.create_timer(1.0, self._retry_csv_load)
            self.path_ready = False
            return

        try:
            self.log_info(f"Loading trajectory from CSV: {self.csv_input_path}")
            success = preprocess_trajectory(
                self.csv_input_path,
                self.csv_output_path,
                wheelbase=self.wheelbase,
                max_steering=self.max_steering,
                min_velocity=self.min_speed
            )

            if success:
                self.reference_trajectory = self._load_trajectory_from_csv(self.csv_output_path)
                self.path_ready = len(self.reference_trajectory) > 0
                self.log_info(f"CSV trajectory loaded: {len(self.reference_trajectory)} points")
                if self.csv_retry_timer is not None:
                    self.csv_retry_timer.cancel()
                    self.csv_retry_timer = None
                if self.publish_visualization:
                    self.trajectory_points = [
                        {'x': s.x, 'y': s.y, 'v': s.v, 'theta': s.theta}
                        for s in self.reference_trajectory
                    ]
            else:
                self.get_logger().error("CSV preprocessing failed")
                self.path_ready = False

        except Exception as e:
            self.get_logger().error(f"Error loading CSV trajectory: {e}")
            self.path_ready = False

    def _resolve_csv_path(self, path, prefer_output=False):
        """Resolve CSV paths relative to the package share directory when needed."""
        if not path:
            return path

        expanded_path = os.path.expanduser(path)
        if os.path.isabs(expanded_path) and os.path.exists(expanded_path):
            return expanded_path

        package_share = get_package_share_directory('horizon_mapper')
        candidate_names = []

        if os.path.isabs(expanded_path):
            candidate_names.append(expanded_path.lstrip(os.sep))

        candidate_names.extend([
            expanded_path,
            os.path.basename(expanded_path),
        ])

        for candidate_name in candidate_names:
            if not candidate_name:
                continue

            candidate_paths = []
            if os.path.isabs(candidate_name):
                candidate_paths.append(candidate_name)
            else:
                candidate_paths.extend([
                    os.path.join(package_share, candidate_name),
                    os.path.join(package_share, 'config', candidate_name),
                ])

            for candidate_path in candidate_paths:
                if os.path.exists(candidate_path):
                    return candidate_path

        if os.path.isabs(expanded_path):
            if expanded_path.startswith('/path_planner/'):
                return os.path.join(package_share, os.path.basename(expanded_path))
            return expanded_path

        if prefer_output:
            return os.path.join(package_share, expanded_path)

        return os.path.join(package_share, expanded_path)

    def _retry_csv_load(self):
        """Retry loading CSV trajectory once input file becomes available."""
        if self.path_ready:
            if self.csv_retry_timer is not None:
                self.csv_retry_timer.cancel()
                self.csv_retry_timer = None
            return
        self.preprocess_and_load_trajectory()

    def _load_trajectory_from_csv(self, path):
        """Load preprocessed trajectory from CSV file"""
        data = []
        try:
            with open(path, 'r') as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                    state = VehicleState()
                    state.x = float(row['x'])
                    state.y = float(row['y'])
                    state.v = float(row['v'])
                    state.theta = float(row['theta']) if 'theta' in row else 0.0
                    state.delta = float(row['delta']) if 'delta' in row else 0.0
                    data.append(state)

            # Calculate theta from consecutive points if not provided
            if len(data) > 1 and data[0].theta == 0.0:
                for i in range(len(data)):
                    if i < len(data) - 1:
                        dx = data[i + 1].x - data[i].x
                        dy = data[i + 1].y - data[i].y
                        data[i].theta = math.atan2(dy, dx)
                    else:
                        data[i].theta = data[i - 1].theta

            valid_points = [s for s in data if self._validate_vehicle_state(s)]
            removed = len(data) - len(valid_points)
            if removed > 0:
                self.get_logger().warn(f"Removed {removed} invalid trajectory points from CSV")
            return valid_points

        except Exception as e:
            self.get_logger().error(f"Failed to load CSV trajectory: {e}")
            return data

    def _create_publishers(self):
        """Create ROS2 publishers"""
        # Core publishers (always available)
        self.path_pub = self.create_publisher(Path, self.reference_path_topic, self.qos_depth)
        self.trajectory_pub = self.create_publisher(VehicleStateArray, self.reference_trajectory_topic, self.qos_depth)
        self.status_pub = self.create_publisher(Bool, self.status_topic, self.qos_depth)

        # Visualization and constraint publishers
        if self._viz:
            self._viz.create_publishers()

    def _create_timers(self):
        """Create ROS2 timers"""
        # Main timer for trajectory publishing
        self.publish_timer = self.create_timer(0.1, self.publish_predictive_trajectory)

        # Visualization timer
        if self._viz:
            self.viz_timer = self.create_timer(0.2, self._viz.publish_visualization)

        # If using CSV source, load trajectory immediately at startup
        if self.use_csv_path:
            self.preprocess_and_load_trajectory()

    def log_info(self, message):
        """Log info messages only if logging is enabled"""
        if self.enable_logging:
            self.get_logger().info(message)

    def log_debug(self, message):
        """Log debug messages only if logging is enabled"""
        if self.enable_debugging:
            self.get_logger().debug(message)

    def horizon_callback(self, msg):
        """Update current horizon_N variable from dynamic topic"""
        try:
            new_horizon = int(msg.data)
            if new_horizon > 0:
                self.horizon = new_horizon
                self.get_logger().info(f"Dynamic horizon_N updated to {self.horizon}")
            else:
                self.get_logger().warn(f"Ignored invalid horizon_N value: {new_horizon}")
        except Exception as e:
            self.get_logger().error(f"Error updating horizon_N: {e}")

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

            self.current_vehicle_state.delta = 0.0

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

    def _override_service_cb(self, request, response):
        """Service callback to reset trajectory state.

        This allows external controllers to request a trajectory reset.
        """
        try:
            self.trajectory_index = 0
            self.get_logger().info("Trajectory state reset via service call")
            response.success = True
            response.message = "Trajectory state reset"
        except Exception as e:
            self.get_logger().error(f"Error resetting trajectory: {e}")
            response.success = False
            response.message = str(e)
        return response

    def find_closest_trajectory_point(self):
        """Find the closest point on the reference trajectory to current position.

        Performs a full O(n) scan to guarantee the globally closest point is
        found regardless of how far trajectory_index has drifted — critical on
        compact closed-loop tracks where a windowed search can lock onto the
        wrong nearby point.

        Among geometrically tied candidates (within TIE_RADIUS of the global
        minimum) the one requiring the fewest forward steps from the current
        trajectory_index is preferred, preventing cross-track jumps at places
        where two track sections run close together.
        """
        if not self.reference_trajectory:
            return 0

        n = len(self.reference_trajectory)
        car_x = self.current_vehicle_state.x
        car_y = self.current_vehicle_state.y

        # Full scan – for typical trajectory sizes (< 2000 pts) this is fast at 10 Hz
        distances = []
        min_dist = float('inf')
        for pt in self.reference_trajectory:
            d = math.sqrt((pt.x - car_x) ** 2 + (pt.y - car_y) ** 2)
            distances.append(d)
            if d < min_dist:
                min_dist = d

        # Collect all points within TIE_RADIUS of the global minimum.
        # Among them prefer the one needing the fewest forward steps from
        # trajectory_index so we don't jump backwards across the track.
        TIE_RADIUS = 0.30  # metres
        candidates = [i for i, d in enumerate(distances) if d <= min_dist + TIE_RADIUS]

        def forward_steps(i):
            return (i - self.trajectory_index + n) % n

        if self.reverse_direction:
            best = min(candidates, key=lambda i: (n - forward_steps(i)) % n)
        else:
            best = min(candidates, key=forward_steps)

        self.trajectory_index = best
        return best

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
                step = -i if self.reverse_direction else i
                ref_index = (closest_index + step) % len(self.reference_trajectory)
                ref_point = self.reference_trajectory[ref_index]

                # Skip individual invalid points rather than aborting the entire horizon
                if not self._validate_vehicle_state(ref_point):
                    self.get_logger().warn(
                        f"Skipping invalid reference point {ref_index}: "
                        f"x={ref_point.x}, y={ref_point.y}, v={ref_point.v}, theta={ref_point.theta}")
                    continue

                # Create predicted state
                predicted_state = VehicleState()
                predicted_state.x = ref_point.x
                predicted_state.y = ref_point.y
                predicted_state.v = ref_point.v
                predicted_state.theta = ref_point.theta  # Include heading angle - critical for MPC!
                predicted_state.delta = ref_point.delta  # bicycle-model reference steering

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

    # Visualization and constraint methods

    def _bound_adjustment_callback(self, msg: BoundAdjustment):
        """Handle bound adjustment messages"""
        if not self.publish_visualization or not self.adaptive_bounds:
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


    def publish_predictive_trajectory(self):
        """Unified trajectory publishing method"""
        # Publish basic trajectory for controller
        self._publish_basic_trajectory()

        # Publish constrained trajectory with visualization
        if self._viz:
            self._viz.publish_constrained_trajectory()

    def _publish_basic_trajectory(self):
        """Publish basic trajectory for model-based controller"""
        # Publish status
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
