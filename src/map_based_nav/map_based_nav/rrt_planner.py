import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math, random

def normalize_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def yaw_from_quat(q) -> float:
    w, x, y, z = q.w, q.x, q.y, q.z
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

class RRTPlanner(Node):
    def __init__(self):
        super().__init__('rrt_planner')
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.timer    = self.create_timer(0.05, self.step)  


        self.goal = np.array([2.5, 0.0], dtype=float)


        self.pos  = np.array([0.0, 0.0], dtype=float)
        self.yaw  = 0.0
        self.path = []  


        self.k_w = 2.0
        self.k_v = 0.8
        self.w_max = 2.0
        self.v_max = 0.35
        self.theta_tol = 0.25  
        self.wp_tol    = 0.12 

    def odom_cb(self, msg: Odometry):
        self.pos = np.array(
            [msg.pose.pose.position.x, msg.pose.pose.position.y],
            dtype=float
        )
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)

    def rrt(self, start: np.ndarray, goal: np.ndarray, step=0.25, n_iter=800):
        start = np.array(start, dtype=float)
        goal  = np.array(goal,  dtype=float)
        nodes = [start]
        parent = {tuple(start): None}

        for _ in range(n_iter):
            rnd = np.array([random.uniform(-3, 3), random.uniform(-3, 3)], dtype=float)
            nearest = min(nodes, key=lambda n: np.linalg.norm(rnd - n))
            v = rnd - nearest
            nrm = np.linalg.norm(v) + 1e-9
            new = nearest + (step * v / nrm)

            if not self.in_collision(new):
                nodes.append(new)
                parent[tuple(new)] = nearest
                if np.linalg.norm(new - goal) < 0.35:
                    path = [goal]
                    p = new
                    while p is not None:
                        path.append(p)
                        p = parent.get(tuple(p))
                    path.reverse()
                    return path

        return []

    def in_collision(self, p: np.ndarray) -> bool:
        # TODO: 
        return False

    def step(self):
       
        if not self.path:
            self.path = self.rrt(self.pos, self.goal)
            if not self.path:
                self.get_logger().warn('RRT failed to find path')
                return
            self.get_logger().info(f'Path found with {len(self.path)} points')

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

        if dist < self.wp_tol:
            self.path.pop(0)

        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = RRTPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
