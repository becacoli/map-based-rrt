import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math, random


def normalize_angle(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quat(q) -> float:
    """Convert quaternion to yaw angle."""
    w, x, y, z = q.w, q.x, q.y, q.z
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class RRTPlanner(Node):
    """RRT-based planner and point-to-point controller."""

    def __init__(self):
        super().__init__('rrt_planner')

        # ROS I/O
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.timer    = self.create_timer(0.05, self.step)

        # Goal parameters (goal inside tilted corridor)
        self.declare_parameter('goal_x', 3.0)
        self.declare_parameter('goal_y', 1.9)
        gx = self.get_parameter('goal_x').value
        gy = self.get_parameter('goal_y').value
        self.goal = np.array([gx, gy], dtype=float)
        self.get_logger().info(f"Goal set to ({gx:.2f}, {gy:.2f})")

        # Robot state
        self.pos  = np.array([0.0, 0.0], dtype=float)
        self.yaw  = 0.0
        self.path = []

        # Control gains and limits
        self.k_w = 2.0
        self.k_v = 0.8
        self.w_max = 2.0
        self.v_max = 0.35
        self.theta_tol = 0.25
        self.wp_tol    = 0.12
        self.goal_tol  = 0.25
        self.reached_goal = False

        # Obstacles
        self.setup_obstacles()

    def setup_obstacles(self):
        """Define rectangular walls, tilted walls (segments) and circles."""
        r = 0.16  # approx TB3 radius

        # Axis-aligned rectangles: straight lower corridor
        rects = [
            {
                'xmin': -2.25 - r, 'xmax': 1.25 + r,
                'ymin': 0.80 - 0.05 - r, 'ymax': 0.80 + 0.05 + r
            },
            {
                'xmin': -2.25 - r, 'xmax': 1.25 + r,
                'ymin': -0.80 - 0.05 - r, 'ymax': -0.80 + 0.05 + r
            },
        ]
        self.rect_obstacles = rects

        # Tilted corridor walls as line segments
        # These numbers are based on your SDF: length ~2.5, yaw ~0.6 rad
        wall_thickness = 0.10
        eff_rad = wall_thickness * 0.5 + r

        self.segment_obstacles = [
            # left tilted wall: from (1.17, 0.89) to (3.23, 2.31)
            {
                'x1': 1.17, 'y1': 0.89,
                'x2': 3.23, 'y2': 2.31,
                'rad': eff_rad
            },
            # right tilted wall: from (1.37, 0.39) to (3.43, 1.81)
            {
                'x1': 1.37, 'y1': 0.39,
                'x2': 3.43, 'y2': 1.81,
                'rad': eff_rad
            },
        ]

        # Cylinders as circles
        self.circle_obstacles = [
            {'cx': -1.0, 'cy': 2.0, 'rad': 0.35 + r},
            {'cx':  2.8, 'cy': 0.0, 'rad': 0.35 + r},
        ]

    def odom_cb(self, msg: Odometry):
        """Update robot position and yaw from odometry."""
        self.pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ], dtype=float)
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)

    def rrt(self, start: np.ndarray, goal: np.ndarray, step=0.25, n_iter=1500):
        """Compute an RRT path from start to goal with biased sampling."""
        start = np.array(start, dtype=float)
        goal  = np.array(goal,  dtype=float)
        nodes = [start]
        parent = {tuple(start): None}

        for _ in range(n_iter):
            rnum = random.random()

            if rnum < 0.10:
                # 10%: sample exactly at the goal
                rnd = goal.copy()
            elif rnum < 0.90:
                # 80%: biased to tilted corridor region
                rnd = np.array([
                    random.uniform(1.2, 3.6),
                    random.uniform(0.8, 2.4)
                ], dtype=float)
            else:
                # 10%: uniform over full map
                rnd = np.array([
                    random.uniform(-3.0, 5.5),
                    random.uniform(-2.5, 2.8)
                ], dtype=float)

            nearest = min(nodes, key=lambda n: np.linalg.norm(rnd - n))
            v = rnd - nearest
            nrm = np.linalg.norm(v) + 1e-9
            new = nearest + (step * v / nrm)

            if not self.in_collision(new):
                nodes.append(new)
                parent[tuple(new)] = nearest
                if np.linalg.norm(new - goal) < 0.40:
                    path = [goal]
                    p = new
                    while p is not None:
                        path.append(p)
                        p = parent.get(tuple(p))
                    path.reverse()
                    return path

        return []

    def point_to_segment_dist2(self, x, y, seg):
        """Squared distance from point to line segment."""
        x1, y1 = seg['x1'], seg['y1']
        x2, y2 = seg['x2'], seg['y2']
        dx = x2 - x1
        dy = y2 - y1
        if dx == 0.0 and dy == 0.0:
            return (x - x1) ** 2 + (y - y1) ** 2
        t = ((x - x1) * dx + (y - y1) * dy) / (dx * dx + dy * dy)
        t = max(0.0, min(1.0, t))
        px = x1 + t * dx
        py = y1 + t * dy
        return (x - px) ** 2 + (y - py) ** 2

    def in_collision(self, p: np.ndarray) -> bool:
        """Check if a point lies inside any obstacle."""
        x, y = float(p[0]), float(p[1])

        # Rectangles
        for obs in self.rect_obstacles:
            if (obs['xmin'] <= x <= obs['xmax'] and
                obs['ymin'] <= y <= obs['ymax']):
                return True

        # Tilted walls (segments)
        for seg in self.segment_obstacles:
            d2 = self.point_to_segment_dist2(x, y, seg)
            if d2 <= seg['rad'] ** 2:
                return True

        # Circles
        for c in self.circle_obstacles:
            dx = x - c['cx']
            dy = y - c['cy']
            if dx * dx + dy * dy <= c['rad'] * c['rad']:
                return True

        return False

    def step(self):
        """Main loop: plan if needed and track the path."""
        if self.reached_goal:
            stop = Twist()
            self.cmd_pub.publish(stop)
            return

        if not self.path:
            self.path = self.rrt(self.pos, self.goal)
            if not self.path:
                self.get_logger().warn('RRT failed to find path')
                return
            self.get_logger().info(f'Path found with {len(self.path)} points')

        goal_dist = float(np.linalg.norm(self.goal - self.pos))
        if goal_dist < self.goal_tol:
            self.get_logger().info('GOAL REACHED! Stopping robot.')
            self.reached_goal = True
            stop = Twist()
            self.cmd_pub.publish(stop)
            return

        target = self.path[0]
        dx, dy = float(target[0] - self.pos[0]), float(target[1] - self.pos[1])
        dist   = math.hypot(dx, dy)
        desired = math.atan2(dy, dx)
        err = normalize_angle(desired - self.yaw)

        cmd = Twist()
        cmd.angular.z = max(-self.w_max, min(self.k_w * err, self.w_max))

        if abs(err) > self.theta_tol:
            cmd.linear.x = 0.0
        else:
            v = min(self.k_v * dist, self.v_max)
            cmd.linear.x = v

        if dist < self.wp_tol and len(self.path) > 1:
            self.path.pop(0)

        self.cmd_pub.publish(cmd)


def main():
    """Node entry point."""
    rclpy.init()
    node = RRTPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
