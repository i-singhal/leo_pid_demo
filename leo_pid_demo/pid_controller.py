"""
PID Waypoint Follower — RViz + rqt Version
============================================
Uses Gazebo ground truth for control AND visualization.
Broadcasts ground truth as tf: world -> base_footprint
so RViz can display everything correctly.

REQUIRED — run in separate terminals:

  # T1: Gazebo
  ros2 launch leo_gz_bringup leo_gz.launch.py

  # T2: Ground truth bridge
  ros2 run ros_gz_bridge parameter_bridge \
    /world/leo_empty/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V

  # T3: This node
  ros2 run leo_pid_demo pid_waypoint_follower

  # T4: RViz (set Fixed Frame to "world")
  rviz2 -d waypoint_follower.rviz

  # T5 (optional): rqt parameter tuning
  ros2 run rqt_reconfigure rqt_reconfigure
"""

import math
import threading

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped, Point, TransformStamped
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from builtin_interfaces.msg import Duration


# ── Utility functions ────────────────────────────────────────────

def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, lower, upper):
    return max(min(value, upper), lower)


# ── The frame everything is published in ─────────────────────────
WORLD_FRAME = 'world'


class PIDWaypointFollower(Node):
    def __init__(self):
        super().__init__('pid_waypoint_follower')

        # ── cmd_vel publisher ────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── tf broadcaster: world -> base_footprint ──────────────
        # This lets RViz know the robot's true position in the
        # world frame. No localization needed — it's ground truth.
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── Ground truth from Gazebo ─────────────────────────────
        self.gt_sub = self.create_subscription(
            TFMessage,
            '/world/leo_empty/dynamic_pose/info',
            self.ground_truth_callback,
            10
        )

        # ── Odom (fallback only) ─────────────────────────────────
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # ── RViz "2D Nav Goal" click ─────────────────────────────
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.rviz_goal_callback, 10
        )

        # ── RViz visualization publishers ────────────────────────
        self.marker_pub = self.create_publisher(
            MarkerArray, '/waypoint_markers', 10
        )
        self.path_pub = self.create_publisher(Path, '/robot_path', 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = WORLD_FRAME

        self.goal_marker_pub = self.create_publisher(
            Marker, '/goal_marker', 10
        )

        # ── Timers ───────────────────────────────────────────────
        self.control_hz = 20.0
        self.timer = self.create_timer(1.0 / self.control_hz, self.control_loop)
        self.viz_timer = self.create_timer(0.2, self.publish_visualization)

        # ── Robot state ──────────────────────────────────────────
        self._pose_lock = threading.Lock()
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0

        self.gt_available = False
        self.odom_ready = False
        self.pose_ready = False

        # ── Goal / path state ────────────────────────────────────
        self.goal_x = None
        self.goal_y = None
        self.pending_goal = None
        self.new_goal_requested = False

        self.waypoints = []
        self.current_wp_idx = 0
        self.path_generated = False
        self.finished = True
        self.phase = 'move'
        self.wp_spacing = 0.5

        # ── PID parameters ───────────────────────────────────────
        self.declare_parameter('kp_lin', 0.6)
        self.declare_parameter('ki_lin', 0.0)
        self.declare_parameter('kd_lin', 0.05)
        self.declare_parameter('kp_ang', 1.2)
        self.declare_parameter('ki_ang', 0.0)
        self.declare_parameter('kd_ang', 0.15)
        self.declare_parameter('max_lin', 0.12)
        self.declare_parameter('max_ang', 0.6)
        self.declare_parameter('dist_tolerance', 0.10)
        self.declare_parameter('final_dist_tolerance', 0.05)
        self.declare_parameter('turn_in_place_threshold', 0.35)
        self.declare_parameter('align_heading_tolerance', 0.08)
        self.declare_parameter('wp_spacing', 0.5)

        self._sync_params()
        self.add_on_set_parameters_callback(self._on_param_change)

        # ── Derivative filter ────────────────────────────────────
        self.deriv_filter_alpha = 0.3
        self.filtered_ang_deriv = 0.0
        self.filtered_lin_deriv = 0.0

        # ── PID limits & memory ──────────────────────────────────
        self.integral_limit_lin = 0.5
        self.integral_limit_ang = 0.5
        self.lin_integral = 0.0
        self.ang_integral = 0.0
        self.prev_dist_error = None
        self.prev_heading_error = None
        self.prev_time = None

        # ── Thread safety ────────────────────────────────────────
        self.goal_lock = threading.Lock()

        # ── Terminal input ───────────────────────────────────────
        self.input_thread = threading.Thread(
            target=self.goal_input_loop, daemon=True
        )
        self.input_thread.start()

        self.get_logger().info('PID waypoint follower started.')
        self.get_logger().info(
            'Set goals: RViz "2D Goal Pose" click  OR  type "x y" here'
        )
        self.get_logger().info(
            f'RViz fixed frame: {WORLD_FRAME}'
        )

    # ── Parameters ───────────────────────────────────────────────

    def _sync_params(self):
        self.kp_lin = self.get_parameter('kp_lin').value
        self.ki_lin = self.get_parameter('ki_lin').value
        self.kd_lin = self.get_parameter('kd_lin').value
        self.kp_ang = self.get_parameter('kp_ang').value
        self.ki_ang = self.get_parameter('ki_ang').value
        self.kd_ang = self.get_parameter('kd_ang').value
        self.max_lin = self.get_parameter('max_lin').value
        self.max_ang = self.get_parameter('max_ang').value
        self.dist_tolerance = self.get_parameter('dist_tolerance').value
        self.final_dist_tolerance = self.get_parameter('final_dist_tolerance').value
        self.turn_in_place_threshold = self.get_parameter('turn_in_place_threshold').value
        self.align_heading_tolerance = self.get_parameter('align_heading_tolerance').value
        self.wp_spacing = self.get_parameter('wp_spacing').value

    def _on_param_change(self, params):
        for p in params:
            if hasattr(self, p.name):
                setattr(self, p.name, p.value)
                self.get_logger().info(f'Param: {p.name} = {p.value}')
        return SetParametersResult(successful=True)

    # ── Sensor callbacks ─────────────────────────────────────────

    def ground_truth_callback(self, msg):
        if not msg.transforms:
            return

        best = None
        best_score = -1.0
        for t in msg.transforms:
            tz = t.transform.translation.z
            horiz = abs(t.transform.translation.x) + abs(t.transform.translation.y)
            if abs(tz) < 0.05 and horiz > best_score:
                best_score = horiz
                best = t

        if best is None:
            best = msg.transforms[0]

        x = best.transform.translation.x
        y = best.transform.translation.y
        yaw = quaternion_to_yaw(best.transform.rotation)

        with self._pose_lock:
            self._x = x
            self._y = y
            self._yaw = yaw

        # ── Broadcast tf: world -> base_footprint ────────────────
        # This is what makes RViz work — it knows where the robot
        # is in the world frame, so markers line up with reality.
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = WORLD_FRAME
        tf_msg.child_frame_id = 'base_footprint'
        tf_msg.transform = best.transform
        self.tf_broadcaster.sendTransform(tf_msg)

        if not self.gt_available:
            self.gt_available = True
            self.pose_ready = True
            self.get_logger().info(
                f'GROUND TRUTH ONLINE  pos=({x:.3f},{y:.3f})  '
                f'yaw={math.degrees(yaw):.1f}deg  '
                f'Broadcasting tf: {WORLD_FRAME} -> base_footprint'
            )

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)

        with self._pose_lock:
            self._odom_x = x
            self._odom_y = y
            self._odom_yaw = yaw
            if not self.gt_available:
                self._x = x
                self._y = y
                self._yaw = yaw

        if not self.odom_ready:
            self.odom_ready = True
            if not self.gt_available:
                self.pose_ready = True
                self.get_logger().warn('Using ODOM (ground truth bridge not running).')

    # ── RViz click goal ──────────────────────────────────────────

    def rviz_goal_callback(self, msg):
        gx = msg.pose.position.x
        gy = msg.pose.position.y

        with self.goal_lock:
            was_moving = not self.finished
            self.pending_goal = (gx, gy)
            self.new_goal_requested = True

        action = 'Preempting ->' if was_moving else 'New'
        self.get_logger().info(
            f'RViz goal: {action} ({gx:.2f}, {gy:.2f})'
        )

    # ── Terminal goal input ──────────────────────────────────────

    def goal_input_loop(self):
        while True:
            try:
                raw = input('\nEnter goal as: x y  (or q to quit): ').strip()
                if raw.lower() == 'q':
                    break
                parts = raw.split()
                if len(parts) != 2:
                    print('Enter exactly 2 numbers, e.g. 2.0 1.0')
                    continue
                gx, gy = float(parts[0]), float(parts[1])
                with self.goal_lock:
                    was_moving = not self.finished
                    self.pending_goal = (gx, gy)
                    self.new_goal_requested = True
                if was_moving:
                    print(f'Preempting -> ({gx:.2f}, {gy:.2f})')
                else:
                    print(f'Queued goal: ({gx:.2f}, {gy:.2f})')
            except ValueError:
                print('Invalid input.')
            except (EOFError, KeyboardInterrupt):
                break

    # ── Pose access ──────────────────────────────────────────────

    def _get_pose(self):
        with self._pose_lock:
            return self._x, self._y, self._yaw

    # ── RViz visualization ───────────────────────────────────────

    def publish_visualization(self):
        if not self.pose_ready:
            return

        now = self.get_clock().now().to_msg()
        cx, cy, cyaw = self._get_pose()

        # ── Robot path trail ─────────────────────────────────────
        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = WORLD_FRAME
        pose.pose.position.x = cx
        pose.pose.position.y = cy
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(cyaw / 2.0)
        pose.pose.orientation.w = math.cos(cyaw / 2.0)

        self.path_msg.header.stamp = now
        self.path_msg.poses.append(pose)
        if len(self.path_msg.poses) > 2000:
            self.path_msg.poses = self.path_msg.poses[-2000:]
        self.path_pub.publish(self.path_msg)

        # ── Waypoint markers ─────────────────────────────────────
        marker_array = MarkerArray()

        if self.waypoints and not self.finished:
            for i, (wx, wy) in enumerate(self.waypoints):
                is_final = (i == len(self.waypoints) - 1)
                is_reached = (i < self.current_wp_idx)
                is_current = (i == self.current_wp_idx)

                m = Marker()
                m.header.frame_id = WORLD_FRAME
                m.header.stamp = now
                m.ns = 'waypoints'
                m.id = i
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = wx
                m.pose.position.y = wy
                m.pose.position.z = 0.05

                if is_final:
                    m.scale.x = m.scale.y = m.scale.z = 0.80
                else:
                    m.scale.x = m.scale.y = m.scale.z = 0.50

                if is_reached:
                    m.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.4)
                elif is_current:
                    m.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
                elif is_final:
                    m.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
                else:
                    m.color = ColorRGBA(r=0.2, g=0.4, b=1.0, a=0.8)

                m.lifetime = Duration(sec=0, nanosec=0)
                marker_array.markers.append(m)

            # Line connecting waypoints
            line = Marker()
            line.header.frame_id = WORLD_FRAME
            line.header.stamp = now
            line.ns = 'waypoint_line'
            line.id = 0
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.08
            line.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.5)
            line.lifetime = Duration(sec=0, nanosec=0)

            for wx, wy in self.waypoints:
                p = Point()
                p.x = wx
                p.y = wy
                p.z = 0.02
                line.points.append(p)
            marker_array.markers.append(line)

        else:
            clear = Marker()
            clear.header.frame_id = WORLD_FRAME
            clear.header.stamp = now
            clear.ns = 'waypoints'
            clear.action = Marker.DELETEALL
            marker_array.markers.append(clear)

            clear_line = Marker()
            clear_line.header.frame_id = WORLD_FRAME
            clear_line.header.stamp = now
            clear_line.ns = 'waypoint_line'
            clear_line.action = Marker.DELETEALL
            marker_array.markers.append(clear_line)

        self.marker_pub.publish(marker_array)

        # ── Goal arrow ───────────────────────────────────────────
        goal_m = Marker()
        goal_m.header.frame_id = WORLD_FRAME
        goal_m.header.stamp = now
        goal_m.ns = 'goal'
        goal_m.id = 0
        goal_m.type = Marker.ARROW
        goal_m.lifetime = Duration(sec=0, nanosec=0)

        if self.goal_x is not None and not self.finished:
            goal_m.action = Marker.ADD
            goal_m.pose.position.x = self.goal_x
            goal_m.pose.position.y = self.goal_y
            goal_m.pose.position.z = 0.15
            goal_m.pose.orientation.y = 0.707
            goal_m.pose.orientation.w = 0.707
            goal_m.scale.x = 0.3
            goal_m.scale.y = 0.08
            goal_m.scale.z = 0.08
            goal_m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.9)
        else:
            goal_m.action = Marker.DELETE

        self.goal_marker_pub.publish(goal_m)

    # ── Waypoints ────────────────────────────────────────────────

    def generate_waypoints(self):
        sx, sy, _ = self._get_pose()
        total_dist = math.hypot(self.goal_x - sx, self.goal_y - sy)
        if total_dist < 1e-6:
            self.waypoints = [(self.goal_x, self.goal_y)]
        else:
            n = max(1, round(total_dist / self.wp_spacing))
            self.waypoints = []
            for i in range(1, n + 1):
                a = i / n
                self.waypoints.append((
                    sx + a * (self.goal_x - sx),
                    sy + a * (self.goal_y - sy)
                ))
        self.current_wp_idx = 0
        self.path_generated = True
        self.get_logger().info(
            f'Generated {len(self.waypoints)} waypoints ({total_dist:.2f}m)'
        )

    # ── Helpers ──────────────────────────────────────────────────

    def stop_robot(self):
        try:
            self.cmd_pub.publish(Twist())
        except Exception:
            pass

    def reset_pid(self):
        self.lin_integral = 0.0
        self.ang_integral = 0.0
        self.prev_dist_error = None
        self.prev_heading_error = None
        self.prev_time = None
        self.filtered_ang_deriv = 0.0
        self.filtered_lin_deriv = 0.0

    # ── Control loop ─────────────────────────────────────────────

    def control_loop(self):
        if not self.pose_ready:
            return

        with self.goal_lock:
            if self.new_goal_requested and self.pending_goal is not None:
                if not self.finished:
                    self.stop_robot()
                    self.get_logger().info(
                        f'Preempting goal ({self.goal_x:.2f},{self.goal_y:.2f})'
                    )
                self.goal_x, self.goal_y = self.pending_goal
                self.pending_goal = None
                self.new_goal_requested = False
                self.finished = False
                self.path_generated = False
                self.current_wp_idx = 0
                self.phase = 'move'
                self.reset_pid()

                sx, sy, syaw = self._get_pose()
                gdir = math.atan2(self.goal_y - sy, self.goal_x - sx)
                self.get_logger().info(
                    f'NEW GOAL ({self.goal_x:.2f},{self.goal_y:.2f})  '
                    f'from ({sx:.2f},{sy:.2f}) yaw={math.degrees(syaw):.1f}  '
                    f'err={math.degrees(wrap_to_pi(gdir-syaw)):.1f}deg'
                )
                self.generate_waypoints()

        if self.finished or not self.path_generated:
            return
        if self.current_wp_idx >= len(self.waypoints):
            self.stop_robot()
            self.finished = True
            self.phase = 'move'
            self.get_logger().info('All waypoints reached.')
            return

        cx, cy, cyaw = self._get_pose()
        gx, gy = self.waypoints[self.current_wp_idx]
        is_final = (self.current_wp_idx == len(self.waypoints) - 1)

        dx, dy = gx - cx, gy - cy
        dist_error = math.hypot(dx, dy)
        target_heading = math.atan2(dy, dx)
        heading_error = wrap_to_pi(target_heading - cyaw)

        now = self.get_clock().now()
        if self.prev_time is None:
            self.prev_time = now
            self.prev_dist_error = dist_error
            self.prev_heading_error = heading_error
            return
        dt = max((now - self.prev_time).nanoseconds / 1e9, 1e-4)
        self.prev_time = now

        # ── ALIGN ────────────────────────────────────────────────
        if self.phase == 'align':
            ni = self.current_wp_idx + 1
            if ni >= len(self.waypoints):
                self.finished = True
                self.phase = 'move'
                self.stop_robot()
                return
            nx, ny = self.waypoints[ni]
            ae = wrap_to_pi(math.atan2(ny - cy, nx - cx) - cyaw)
            if abs(ae) < self.align_heading_tolerance:
                self.current_wp_idx = ni
                self.phase = 'move'
                self.reset_pid()
                self.stop_robot()
                return
            if self.prev_heading_error is not None:
                rd = wrap_to_pi(ae - self.prev_heading_error) / dt
                self.filtered_ang_deriv = (
                    self.deriv_filter_alpha * rd
                    + (1.0 - self.deriv_filter_alpha) * self.filtered_ang_deriv
                )
            else:
                self.filtered_ang_deriv = 0.0
            ac = clamp(
                self.kp_ang * ae + self.kd_ang * self.filtered_ang_deriv,
                -self.max_ang, self.max_ang
            )
            cmd = Twist()
            cmd.angular.z = ac
            self.cmd_pub.publish(cmd)
            self.prev_heading_error = ae
            return

        # ── MOVE ─────────────────────────────────────────────────
        tol = self.final_dist_tolerance if is_final else self.dist_tolerance
        if dist_error < tol:
            self.stop_robot()
            if is_final:
                self.finished = True
                self.phase = 'move'
                self.get_logger().info('Final waypoint reached.')
                return
            self.phase = 'align'
            self.reset_pid()
            return

        # Linear PID
        self.lin_integral = clamp(
            self.lin_integral + dist_error * dt,
            -self.integral_limit_lin, self.integral_limit_lin
        )
        if self.prev_dist_error is not None:
            rld = (dist_error - self.prev_dist_error) / dt
            self.filtered_lin_deriv = (
                self.deriv_filter_alpha * rld
                + (1.0 - self.deriv_filter_alpha) * self.filtered_lin_deriv
            )
        self.prev_dist_error = dist_error
        lin_cmd = (self.kp_lin * dist_error
                   + self.ki_lin * self.lin_integral
                   + self.kd_lin * self.filtered_lin_deriv)

        # Angular PID
        self.ang_integral = clamp(
            self.ang_integral + heading_error * dt,
            -self.integral_limit_ang, self.integral_limit_ang
        )
        if self.prev_heading_error is not None:
            rad = wrap_to_pi(heading_error - self.prev_heading_error) / dt
            self.filtered_ang_deriv = (
                self.deriv_filter_alpha * rad
                + (1.0 - self.deriv_filter_alpha) * self.filtered_ang_deriv
            )
        self.prev_heading_error = heading_error
        ang_cmd = (self.kp_ang * heading_error
                   + self.ki_ang * self.ang_integral
                   + self.kd_ang * self.filtered_ang_deriv)

        # Speed scaling
        if abs(heading_error) > self.turn_in_place_threshold:
            lin_cmd = 0.0
        else:
            lin_cmd *= max(0.0, math.cos(heading_error))
        if is_final and dist_error < 0.20:
            lin_cmd *= (dist_error / 0.20)

        ml = 0.08 if is_final else self.max_lin
        lin_cmd = clamp(lin_cmd, 0.0, ml)
        ang_cmd = clamp(ang_cmd, -self.max_ang, self.max_ang)

        cmd = Twist()
        cmd.linear.x = lin_cmd
        cmd.angular.z = ang_cmd
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PIDWaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop_robot()
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
