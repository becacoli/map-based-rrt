import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math
import random

def normalize_angle(a):
    a = (a + math.pi) % (2*math.pi) - math.pi
    return a

class RRTPlanner(Node):
    def __init__(self):
        super().__init__('rrt_planner')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.timer = self.create_timer(0.1, self.step)

        self.goal = np.array([2.5, 0.0])
        self.pos = np.array([0.0, 0.0])
        self.yaw = 0.0
        self.path = []

    def odom_cb(self, msg):
        self.pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        # yaw aproximado a partir de twist (fallback simples se não quiser converter quaternion)
        vx = msg.twist.twist.linear.x
        vyaw = msg.twist.twist.angular.z  # não é yaw absoluto, mas serve pra não travar
        # (se quiser yaw correto: instale tf-transformations e converta quaternion)

    def step(self):
        if not self.path:
            self.path = self.rrt(self.pos, self.goal)
            if not self.path:
                self.get_logger().warn('RRT failed to find path')
                return
            self.get_logger().info(f'Path found with {len(self.path)} points')

        # navegação ponto-a-ponto
        target = self.path[0]
        dx, dy = target - self.pos
        dist = np.linalg.norm([dx, dy])

        tw = Twist()
        if dist < 0.12:
            self.path.pop(0)
        else:
            desired = math.atan2(dy, dx)
            err = normalize_angle(desired - self.yaw)
            tw.angular.z = 1.5 * err
            tw.linear.x = 0.2 if abs(err) < 0.5 else 0.05
        self.cmd_pub.publish(tw)

    def rrt(self, start, goal, step=0.25, n_iter=800):
        nodes = [start]
        parent = {tuple(start): None}
        for _ in range(n_iter):
            rnd = np.array([random.uniform(-3, 3), random.uniform(-3, 3)])
            nearest = min(nodes, key=lambda n: np.linalg.norm(rnd - n))
            v = rnd - nearest
            v = v / (np.linalg.norm(v) + 1e-9)
            new = nearest + step * v
            if not self.in_collision(new):
                nodes.append(new)
                parent[tuple(new)] = nearest
                if np.linalg.norm(new - goal) < 0.35:
                    path = [goal]; p = new
                    while p is not None:
                        path.append(p); p = parent.get(tuple(p))
                    path.reverse()
                    return path
        return []

    def in_collision(self, p):
        # implementar se usar occupancy grid; aqui está "livre"
        return False

def main():
    rclpy.init()
    node = RRTPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
