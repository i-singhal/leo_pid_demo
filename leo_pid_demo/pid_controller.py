"""
PID Waypoint Follower — Gazebo Ground Truth Version
====================================================
Uses /world/leo_empty/dynamic_pose/info for ground truth position + yaw.

REQUIRED: run this bridge in a separate terminal BEFORE starting:
  ros2 run ros_gz_bridge parameter_bridge \
    /world/leo_empty/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V
"""

import math
import threading
import subprocess

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage


# ── Utility functions ────────────────────────────────────────────

def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, lower, upper):
    return max(min(value, upper), lower)


# ── Node ─────────────────────────────────────────────────────────

class PIDWaypointFollower(Node):
    def __init__(self):
        super().__init__('pid_waypoint_follower')

        # ── ROS interfaces ───────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Ground truth from Gazebo dynamic_pose
        self.gt_sub = self.create_subscription(
            TFMessage,
            '/world/leo_empty/dynamic_pose/info',
            self.ground_truth_callback,
            10
        )

        # Odom — fallback + drift comparison
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        self.control_hz = 20.0
        self.timer = self.create_timer(1.0 / self.control_hz, self.control_loop)

        # ── Gazebo world name (for marker spawning) ──────────────
        self.world_name = 'leo_empty'

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

        # ── PID parameters (tuneable at runtime) ─────────────────
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

        # ── Thread safety / markers ──────────────────────────────
        self.goal_lock = threading.Lock()
        self.marker_batch_id = 0

        # ── Input thread ─────────────────────────────────────────
        self.input_thread = threading.Thread(
            target=self.goal_input_loop, daemon=True
        )
        self.input_thread.start()

        self.get_logger().info('PID waypoint follower started.')
        self.get_logger().info(
            'Type a new goal ANY TIME to preempt the current one.'
        )
        self.get_logger().info(
            'Waiting for ground truth on '
            '/world/leo_empty/dynamic_pose/info ...'
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
        """Extract robot base pose from dynamic_pose/info.

        The first transform in the TFMessage is the model's world-frame
        pose.  Remaining transforms are individual links (wheels etc).
        We identify the robot as the entry closest to ground (z ≈ 0)
        with the largest horizontal displacement from origin.
        """
        if not msg.transforms:
            return

        # Pick the first transform with z near ground level
        # (the model root, not a wheel or rocker link)
        best = None
        best_score = -1.0
        for t in msg.transforms:
            tz = t.transform.translation.z
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            horiz = abs(tx) + abs(ty)
            # Robot base: z close to 0, large horizontal displacement
            if abs(tz) < 0.05 and horiz > best_score:
                best_score = horiz
                best = t

        if best is None:
            # Fallback: just use index 0
            best = msg.transforms[0]

        x = best.transform.translation.x
        y = best.transform.translation.y
        yaw = quaternion_to_yaw(best.transform.rotation)

        with self._pose_lock:
            self._x = x
            self._y = y
            self._yaw = yaw

        if not self.gt_available:
            self.gt_available = True
            self.pose_ready = True
            self.get_logger().info(
                f'=== GROUND TRUTH ONLINE ===  '
                f'pos=({x:.3f},{y:.3f})  yaw={math.degrees(yaw):.1f}deg'
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
                self.get_logger().warn(
                    'Ground truth NOT received yet — using ODOM (will drift!).'
                )

    # ── Pose access ──────────────────────────────────────────────

    def _get_pose(self):
        with self._pose_lock:
            return self._x, self._y, self._yaw

    def _get_odom_pose(self):
        with self._pose_lock:
            return self._odom_x, self._odom_y, self._odom_yaw

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
                    print(f'Preempting -> new goal: ({gx:.2f}, {gy:.2f})')
                else:
                    print(f'Queued goal: ({gx:.2f}, {gy:.2f})')
            except ValueError:
                print('Invalid input.')
            except (EOFError, KeyboardInterrupt):
                break

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

    # ── Gazebo markers ───────────────────────────────────────────

    def spawn_marker(self, name, x, y, z, radius, r, g, b):
        sdf = (
            f'<?xml version="1.0" ?><sdf version="1.7">'
            f'<model name="{name}"><static>true</static>'
            f'<pose>{x} {y} {z} 0 0 0</pose>'
            f'<link name="link"><visual name="visual">'
            f'<geometry><sphere><radius>{radius}</radius></sphere></geometry>'
            f'<material><ambient>{r} {g} {b} 1</ambient>'
            f'<diffuse>{r} {g} {b} 1</diffuse>'
            f'</material></visual></link></model></sdf>'
        )
        service = f'/world/{self.world_name}/create'
        req_sdf = sdf.replace('"', '\\"')
        try:
            subprocess.run([
                'gz', 'service', '-s', service,
                '--reqtype', 'gz.msgs.EntityFactory',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '1000',
                '--req', f'sdf: "{req_sdf}"',
            ], capture_output=True, text=True, check=False)
        except Exception:
            pass

    def spawn_waypoint_markers(self):
        self.marker_batch_id += 1
        b = self.marker_batch_id
        for i, (wx, wy) in enumerate(self.waypoints):
            final = (i == len(self.waypoints) - 1)
            name = f'goal_{b}_final' if final else f'goal_{b}_wp_{i}'
            self.spawn_marker(
                name, wx, wy,
                0.08 if final else 0.06,
                0.08 if final else 0.05,
                1 if final else 0, 0, 1 if not final else 0
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
                    f'=== NEW GOAL ({self.goal_x:.2f},{self.goal_y:.2f}) ===  '
                    f'from ({sx:.2f},{sy:.2f}) yaw={math.degrees(syaw):.1f}  '
                    f'err={math.degrees(wrap_to_pi(gdir-syaw)):.1f}deg  '
                    f'src={"GT" if self.gt_available else "ODOM"}'
                )
                self.generate_waypoints()
                self.spawn_waypoint_markers()

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

        # Logging with drift comparison
        ox, oy, oy_yaw = self._get_odom_pose()
        pd = math.hypot(cx - ox, cy - oy)
        yd = math.degrees(wrap_to_pi(cyaw - oy_yaw))

        '''self.get_logger().info(
            f'MOVE wp={self.current_wp_idx}{"(F)" if is_final else ""}  '
            f'pos=({cx:.2f},{cy:.2f}) yaw={math.degrees(cyaw):.1f}  '
            f'err={math.degrees(heading_error):.1f}  '
            f'd={dist_error:.3f}  '
            f'v={lin_cmd:.3f} w={ang_cmd:.3f}  '
            f'drift:pos={pd:.2f}m yaw={yd:.1f}deg'
        )'''


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
