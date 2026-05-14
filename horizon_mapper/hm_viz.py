"""
Horizon Mapper Visualization Helpers
"""

import colorsys
import math
from typing import List, Tuple

import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from giu_f1t_interfaces.msg import (
    ConstrainedVehicleStateArray,
    ConstrainedVehicleState,
    ConstraintStatus,
)

try:
    from tf_transformations import quaternion_from_euler
except ImportError:
    def quaternion_from_euler(roll, pitch, yaw):
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


class HorizonMapperViz:
    def __init__(self, node):
        self._node = node
        self._constrained_trajectory_pub = None
        self._constraint_status_pub = None
        self._corridor_viz_pub = None
        self._velocity_path_pub = None
        self._car_viz_pub = None

    def create_publishers(self):
        n = self._node
        self._constrained_trajectory_pub = n.create_publisher(
            ConstrainedVehicleStateArray, n.constrained_trajectory_topic, n.qos_depth)
        self._constraint_status_pub = n.create_publisher(
            ConstraintStatus, n.constraint_status_topic, n.qos_depth)
        self._corridor_viz_pub = n.create_publisher(
            MarkerArray, n.corridor_viz_topic, n.qos_depth)
        self._velocity_path_pub = n.create_publisher(
            MarkerArray, n.velocity_viz_topic, n.qos_depth)
        self._car_viz_pub = n.create_publisher(
            MarkerArray, n.car_viz_topic, n.qos_depth)

    def publish_constrained_trajectory(self):
        n = self._node
        if not n.publish_visualization or not hasattr(n, 'trajectory_points') or len(n.trajectory_points) == 0:
            return
        if not n.path_ready:
            return

        start_index = self._find_closest_point_index()

        constrained_msg = ConstrainedVehicleStateArray()
        constrained_msg.header.stamp = n.get_clock().now().to_msg()
        constrained_msg.header.frame_id = n.map_frame

        for i in range(n.horizon):
            step = -i if n.reverse_direction else i
            point_index = (start_index + step) % len(n.trajectory_points)
            traj_point = n.trajectory_points[point_index]

            def get_value(point, key):
                if isinstance(point, dict):
                    return point[key]
                return getattr(point, key)

            state = ConstrainedVehicleState()
            state.x = get_value(traj_point, 'x')
            state.y = get_value(traj_point, 'y')
            state.theta = get_value(traj_point, 'theta')
            state.v_ref = max(get_value(traj_point, 'v'), n.min_speed)

            if n.use_received_bounds and point_index < len(n.received_left_bounds):
                left_bound = n.received_left_bounds[point_index]
                right_bound = n.received_right_bounds[point_index]
                confidence = n.default_confidence
            else:
                left_bound, right_bound, confidence = self._compute_adaptive_bounds(
                    point_index, state.v_ref)

            state.left_bound = left_bound
            state.right_bound = right_bound
            state.confidence = confidence

            constrained_msg.states.append(state)

        self._constrained_trajectory_pub.publish(constrained_msg)
        self._publish_constraint_status(constrained_msg.states)

    def publish_visualization(self):
        n = self._node
        if not n.publish_visualization or not hasattr(n, 'trajectory_points'):
            return
        if not n.path_ready or len(n.trajectory_points) == 0:
            return

        self._publish_corridor_visualization()
        self._publish_velocity_visualization()
        self._publish_car_visualization()

    def _find_closest_point_index(self) -> int:
        return self._node.find_closest_trajectory_point()

    def _compute_path_normal(self, index: int) -> Tuple[float, float]:
        n = self._node
        if not hasattr(n, 'trajectory_points') or len(n.trajectory_points) == 0:
            return 0.0, 1.0

        n_points = len(n.trajectory_points)

        def get_xy(point):
            if isinstance(point, dict):
                return point['x'], point['y']
            return point.x, point.y

        prev_idx = (index - 1) % n_points
        next_idx = (index + 1) % n_points
        x1, y1 = get_xy(n.trajectory_points[prev_idx])
        x2, y2 = get_xy(n.trajectory_points[next_idx])
        dx = x2 - x1
        dy = y2 - y1

        length = math.sqrt(dx ** 2 + dy ** 2)
        if length > 1e-9:
            dx /= length
            dy /= length

        return -dy, dx

    def _compute_adaptive_bounds(self, index: int, velocity: float) -> Tuple[float, float, float]:
        n = self._node
        base_left = n.default_left_bound + n.left_bound_adjustment
        base_right = n.default_right_bound + n.right_bound_adjustment
        base_confidence = n.default_confidence + n.confidence_adjustment

        base_confidence = max(0.0, min(1.0, base_confidence))
        velocity_factor = min(velocity / 5.0, 1.0)

        curvature_factor = 1.0
        if hasattr(n, 'trajectory_points') and len(n.trajectory_points) > 0 and 0 < index < len(n.trajectory_points) - 1:
            def get_xy(point):
                if isinstance(point, dict):
                    return point['x'], point['y']
                return point.x, point.y

            x1, y1 = get_xy(n.trajectory_points[index - 1])
            x2, y2 = get_xy(n.trajectory_points[index])
            x3, y3 = get_xy(n.trajectory_points[index + 1])

            dx1, dy1 = x2 - x1, y2 - y1
            dx2, dy2 = x3 - x2, y3 - y2

            cross_product = dx1 * dy2 - dy1 * dx2
            curvature = abs(cross_product) / (math.sqrt(dx1**2 + dy1**2) * math.sqrt(dx2**2 + dy2**2) + 1e-6)
            curvature_factor = max(0.5, 1.0 - curvature * 2.0)

        left_bound = base_left * curvature_factor * (1.0 + 0.3 * velocity_factor)
        right_bound = base_right * curvature_factor * (1.0 + 0.3 * velocity_factor)
        confidence = base_confidence * (1.0 + 0.2 * velocity_factor)

        return left_bound, right_bound, confidence

    def _publish_constraint_status(self, states: List[ConstrainedVehicleState]):
        n = self._node
        if not n.publish_visualization:
            return

        status_msg = ConstraintStatus()
        status_msg.header.stamp = n.get_clock().now().to_msg()
        status_msg.header.frame_id = n.map_frame

        status_msg.constraints_active = n.constraints_active
        status_msg.bounds_violated = n.bounds_violated
        status_msg.adapting = n.adaptive_bounds

        if states:
            status_msg.avg_left_margin = sum(s.left_bound for s in states) / len(states)
            status_msg.avg_right_margin = sum(s.right_bound for s in states) / len(states)
            status_msg.avg_confidence = sum(s.confidence for s in states) / len(states)

        status_msg.status_message = (
            f"Active: {status_msg.constraints_active}, "
            f"Violated: {status_msg.bounds_violated}, "
            f"Adapting: {status_msg.adapting}"
        )

        self._constraint_status_pub.publish(status_msg)

    def _publish_car_visualization(self):
        n = self._node
        if not n.current_pose:
            return
        if not n.path_ready or len(n.trajectory_points) == 0:
            return

        marker_array = MarkerArray()
        stamp = n.get_clock().now().to_msg()

        car_x = n.current_vehicle_state.x
        car_y = n.current_vehicle_state.y

        def get_value(pt, key):
            return pt[key] if isinstance(pt, dict) else getattr(pt, key)

        closest_idx = self._find_closest_point_index()
        n_points = len(n.trajectory_points)
        step_sign = -1 if n.reverse_direction else 1
        target_idx = (closest_idx + step_sign * n.viz_lookahead_steps) % n_points
        target_pt = n.trajectory_points[target_idx]

        tx = get_value(target_pt, 'x')
        ty = get_value(target_pt, 'y')
        lookahead_dist = math.sqrt((tx - car_x) ** 2 + (ty - car_y) ** 2)

        circle_marker = Marker()
        circle_marker.header.stamp = stamp
        circle_marker.header.frame_id = n.map_frame
        circle_marker.ns = "lookahead_circle"
        circle_marker.id = 0
        circle_marker.type = Marker.LINE_STRIP
        circle_marker.action = Marker.ADD
        circle_marker.pose.orientation.w = 1.0
        circle_marker.scale.x = 0.03
        circle_marker.color.r = 0.0
        circle_marker.color.g = 1.0
        circle_marker.color.b = 1.0
        circle_marker.color.a = 0.8

        num_segments = 48
        for j in range(num_segments + 1):
            angle = 2.0 * math.pi * j / num_segments
            pt = Point()
            pt.x = car_x + lookahead_dist * math.cos(angle)
            pt.y = car_y + lookahead_dist * math.sin(angle)
            pt.z = 0.05
            circle_marker.points.append(pt)
        marker_array.markers.append(circle_marker)

        direction_marker = Marker()
        direction_marker.header.stamp = stamp
        direction_marker.header.frame_id = n.map_frame
        direction_marker.ns = "direction_arrow"
        direction_marker.id = 0
        direction_marker.type = Marker.ARROW
        direction_marker.action = Marker.ADD

        start_pt = Point()
        start_pt.x = car_x
        start_pt.y = car_y
        start_pt.z = 0.2

        end_pt = Point()
        end_pt.x = tx
        end_pt.y = ty
        end_pt.z = 0.2

        direction_marker.points = [start_pt, end_pt]
        direction_marker.scale.x = 0.05
        direction_marker.scale.y = 0.12
        direction_marker.scale.z = 0.15
        direction_marker.color.r = 1.0
        direction_marker.color.g = 0.5
        direction_marker.color.b = 0.0
        direction_marker.color.a = 1.0
        marker_array.markers.append(direction_marker)

        self._car_viz_pub.publish(marker_array)

    def _publish_corridor_visualization(self):
        n = self._node
        if not n.path_ready or len(n.trajectory_points) == 0:
            return

        marker_array = MarkerArray()

        danger_thresh = 0.5
        caution_thresh = 1.0

        start_index = self._find_closest_point_index()

        def get_value(point, key):
            if isinstance(point, dict):
                return point[key]
            return getattr(point, key)

        def get_lidar_risk(x, y):
            if n.lidar_scan is None:
                return None
            dx = x - n.current_vehicle_state.x
            dy = y - n.current_vehicle_state.y
            r = math.sqrt(dx**2 + dy**2)
            angle = math.atan2(dy, dx)
            scan = n.lidar_scan
            angle_min = scan.angle_min
            angle_max = scan.angle_max
            angle_inc = scan.angle_increment
            idx = int((angle - angle_min) / angle_inc)
            if idx < 0 or idx >= len(scan.ranges):
                return None
            scan_range = scan.ranges[idx]
            if not np.isfinite(scan_range):
                return None
            return scan_range - r

        n_traj = len(n.trajectory_points)
        for i in range(min(n.horizon, n_traj)):
            step = -i if n.reverse_direction else i
            point_index = (start_index + step) % n_traj
            traj_point = n.trajectory_points[point_index]

            px = get_value(traj_point, 'x')
            py = get_value(traj_point, 'y')
            pv = get_value(traj_point, 'v')

            left_bound, right_bound, confidence = self._compute_adaptive_bounds(
                point_index, pv)

            normal_x, normal_y = self._compute_path_normal(point_index)

            left_x = px + normal_x * left_bound
            left_y = py + normal_y * left_bound
            left_marker = Marker()
            left_marker.header.stamp = n.get_clock().now().to_msg()
            left_marker.header.frame_id = n.map_frame
            left_marker.ns = "corridor_left"
            left_marker.id = i
            left_marker.type = Marker.SPHERE
            left_marker.action = Marker.ADD

            left_marker.pose.position.x = left_x
            left_marker.pose.position.y = left_y
            left_marker.pose.position.z = 0.0

            left_marker.scale.x = 0.1
            left_marker.scale.y = 0.1
            left_marker.scale.z = 0.1

            risk = get_lidar_risk(left_x, left_y)
            if risk is not None and risk < danger_thresh:
                left_marker.color.r = 1.0
                left_marker.color.g = 0.0
                left_marker.color.b = 0.0
            elif risk is not None and risk < caution_thresh:
                left_marker.color.r = 1.0
                left_marker.color.g = 1.0
                left_marker.color.b = 0.0
            else:
                left_marker.color.r = 0.0
                left_marker.color.g = 0.0
                left_marker.color.b = 1.0
            left_marker.color.a = confidence

            marker_array.markers.append(left_marker)

            right_x = px - normal_x * right_bound
            right_y = py - normal_y * right_bound
            right_marker = Marker()
            right_marker.header.stamp = n.get_clock().now().to_msg()
            right_marker.header.frame_id = n.map_frame
            right_marker.ns = "corridor_right"
            right_marker.id = i
            right_marker.type = Marker.SPHERE
            right_marker.action = Marker.ADD

            right_marker.pose.position.x = right_x
            right_marker.pose.position.y = right_y
            right_marker.pose.position.z = 0.0

            right_marker.scale = left_marker.scale

            risk = get_lidar_risk(right_x, right_y)
            if risk is not None and risk < danger_thresh:
                right_marker.color.r = 1.0
                right_marker.color.g = 0.0
                right_marker.color.b = 0.0
            elif risk is not None and risk < caution_thresh:
                right_marker.color.r = 1.0
                right_marker.color.g = 1.0
                right_marker.color.b = 0.0
            else:
                right_marker.color.r = 0.0
                right_marker.color.g = 0.0
                right_marker.color.b = 1.0
            right_marker.color.a = confidence

            marker_array.markers.append(right_marker)

            center_marker = Marker()
            center_marker.header.stamp = n.get_clock().now().to_msg()
            center_marker.header.frame_id = n.map_frame
            center_marker.ns = "centerline"
            center_marker.id = i
            center_marker.type = Marker.SPHERE
            center_marker.action = Marker.ADD

            center_marker.pose.position.x = px
            center_marker.pose.position.y = py
            center_marker.pose.position.z = 0.0

            center_marker.scale.x = 0.05
            center_marker.scale.y = 0.05
            center_marker.scale.z = 0.05

            center_marker.color.r = 0.0
            center_marker.color.g = 1.0
            center_marker.color.b = 0.0
            center_marker.color.a = 1.0

            marker_array.markers.append(center_marker)

        self._corridor_viz_pub.publish(marker_array)

    def _velocity_to_color(self, velocity: float) -> ColorRGBA:
        normalized_vel = min(velocity / self._node.velocity_color_scale, 1.0)

        hue = (1.0 - normalized_vel) * 240.0 / 360.0
        saturation = 1.0
        value = 1.0

        r, g, b = colorsys.hsv_to_rgb(hue, saturation, value)

        color = ColorRGBA()
        color.r = float(r)
        color.g = float(g)
        color.b = float(b)
        color.a = 1.0

        return color

    def _publish_velocity_visualization(self):
        n = self._node
        if not n.path_ready or len(n.trajectory_points) == 0:
            return

        marker_array = MarkerArray()

        def get_value(point, key):
            if isinstance(point, dict):
                return point[key]
            return getattr(point, key)

        for i, point in enumerate(n.trajectory_points[::3]):
            cylinder_marker = Marker()
            cylinder_marker.header.stamp = n.get_clock().now().to_msg()
            cylinder_marker.header.frame_id = n.map_frame
            cylinder_marker.ns = "velocity_heatmap"
            cylinder_marker.id = i
            cylinder_marker.type = Marker.CYLINDER
            cylinder_marker.action = Marker.ADD

            cylinder_marker.pose.position.x = get_value(point, 'x')
            cylinder_marker.pose.position.y = get_value(point, 'y')
            cylinder_marker.pose.position.z = 0.01

            base_radius = 0.15
            velocity_scale = min(get_value(point, 'v') / n.velocity_color_scale, 1.0)
            radius = base_radius + (velocity_scale * 0.1)

            cylinder_marker.scale.x = radius * 2
            cylinder_marker.scale.y = radius * 2
            cylinder_marker.scale.z = 0.02

            cylinder_marker.color = self._velocity_to_color(get_value(point, 'v'))
            cylinder_marker.color.a = 0.7

            marker_array.markers.append(cylinder_marker)

        elevation_marker = Marker()
        elevation_marker.header.stamp = n.get_clock().now().to_msg()
        elevation_marker.header.frame_id = n.map_frame
        elevation_marker.ns = "velocity_elevation"
        elevation_marker.id = 0
        elevation_marker.type = Marker.TRIANGLE_LIST
        elevation_marker.action = Marker.ADD

        elevation_marker.scale.x = 1.0
        elevation_marker.scale.y = 1.0
        elevation_marker.scale.z = 1.0

        width = 0.3
        height_scale = 0.5

        for i in range(len(n.trajectory_points) - 1):
            if i % 2 != 0:
                continue

            p1 = n.trajectory_points[i]
            p2 = n.trajectory_points[i + 1]

            dx = get_value(p2, 'x') - get_value(p1, 'x')
            dy = get_value(p2, 'y') - get_value(p1, 'y')
            length = math.sqrt(dx**2 + dy**2)
            if length > 0:
                normal_x = -dy / length * width / 2
                normal_y = dx / length * width / 2
            else:
                normal_x = normal_y = 0

            h1 = get_value(p1, 'v') * height_scale
            h2 = get_value(p2, 'v') * height_scale

            pt1 = Point()
            pt1.x = get_value(p1, 'x') - normal_x
            pt1.y = get_value(p1, 'y') - normal_y
            pt1.z = 0.0
            elevation_marker.points.append(pt1)
            elevation_marker.colors.append(self._velocity_to_color(get_value(p1, 'v')))

            pt2 = Point()
            pt2.x = get_value(p1, 'x') - normal_x
            pt2.y = get_value(p1, 'y') - normal_y
            pt2.z = h1
            elevation_marker.points.append(pt2)
            elevation_marker.colors.append(self._velocity_to_color(get_value(p1, 'v')))

            pt3 = Point()
            pt3.x = get_value(p2, 'x') - normal_x
            pt3.y = get_value(p2, 'y') - normal_y
            pt3.z = 0.0
            elevation_marker.points.append(pt3)
            elevation_marker.colors.append(self._velocity_to_color(get_value(p2, 'v')))

            pt4 = Point()
            pt4.x = get_value(p1, 'x') - normal_x
            pt4.y = get_value(p1, 'y') - normal_y
            pt4.z = h1
            elevation_marker.points.append(pt4)
            elevation_marker.colors.append(self._velocity_to_color(get_value(p1, 'v')))

            pt5 = Point()
            pt5.x = get_value(p2, 'x') - normal_x
            pt5.y = get_value(p2, 'y') - normal_y
            pt5.z = h2
            elevation_marker.points.append(pt5)
            elevation_marker.colors.append(self._velocity_to_color(get_value(p2, 'v')))

            pt6 = Point()
            pt6.x = get_value(p2, 'x') - normal_x
            pt6.y = get_value(p2, 'y') - normal_y
            pt6.z = 0.0
            elevation_marker.points.append(pt6)
            elevation_marker.colors.append(self._velocity_to_color(get_value(p2, 'v')))

        marker_array.markers.append(elevation_marker)

        for i, point in enumerate(n.trajectory_points[::8]):
            arrow_marker = Marker()
            arrow_marker.header.stamp = n.get_clock().now().to_msg()
            arrow_marker.header.frame_id = n.map_frame
            arrow_marker.ns = "velocity_arrows"
            arrow_marker.id = i
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD

            arrow_marker.pose.position.x = get_value(point, 'x')
            arrow_marker.pose.position.y = get_value(point, 'y')
            arrow_marker.pose.position.z = 0.1

            quat = quaternion_from_euler(0, 0, get_value(point, 'theta'))
            arrow_marker.pose.orientation.x = quat[0]
            arrow_marker.pose.orientation.y = quat[1]
            arrow_marker.pose.orientation.z = quat[2]
            arrow_marker.pose.orientation.w = quat[3]

            velocity_scale = min(get_value(point, 'v') / n.velocity_color_scale, 1.0)
            arrow_length = 0.2 + (velocity_scale * 0.3)
            arrow_marker.scale.x = arrow_length
            arrow_marker.scale.y = 0.05
            arrow_marker.scale.z = 0.05

            arrow_marker.color = self._velocity_to_color(get_value(point, 'v'))
            arrow_marker.color.a = 0.8

            marker_array.markers.append(arrow_marker)

        self._velocity_path_pub.publish(marker_array)
