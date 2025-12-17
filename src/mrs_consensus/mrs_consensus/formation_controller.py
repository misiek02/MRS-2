import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
import numpy as np
import math

class FormationController(Node):
    def __init__(self):
        super().__init__('formation_controller')
        
        self.num_robots = 4
        self.k_p = 1.0       # Formation gain
        self.k_sep = 1.5     # Separation gain
        self.safe_dist = 0.5 # Safety distance
        
        # Target position for the formation center (Virtual Leader)
        self.target_pos = np.array([2.0, 2.0]) 
        
        # Triangle Formation Offsets (xi)
        # Robot 1: Top, Robot 2: Bottom Left, Robot 3: Bottom Right, Robot 4: Center
        self.offsets = np.array([
            [0.0, 0.5],   # R1
            [-0.5, -0.5],  # R2
            [0.5, -0.5],   # R3
            [0.0, 0.0]    # R4
        ])

        self.positions = np.zeros((self.num_robots, 2))
        self.position_received = [False] * self.num_robots

        # Adjacency Matrix: Fully connected for stable formation 
        self.A = np.ones((self.num_robots, self.num_robots)) - np.eye(self.num_robots)

        self.pubs = []
        self.subs = []
        self.path_pubs = []
        self.paths = [Path() for _ in range(self.num_robots)]

        for i in range(self.num_robots):
            robot_id = i + 1
            self.subs.append(self.create_subscription(
                PoseStamped, f'/cf_{robot_id}/pose',
                lambda msg, rid=i: self.pose_callback(msg, rid), 10))
            
            self.pubs.append(self.create_publisher(Twist, f'/cf_{robot_id}/cmd_vel', 10))
            
            self.path_pubs.append(self.create_publisher(Path, f'/cf_{robot_id}/path', 10))
            self.paths[i].header.frame_id = "world"

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Triangle Formation Controller Started")

    def pose_callback(self, msg, robot_idx):
        self.positions[robot_idx] = [msg.pose.position.x, msg.pose.position.y]
        self.position_received[robot_idx] = True
        
        # Path visualization
        p = PoseStamped()
        p.header = msg.header
        p.pose = msg.pose
        self.paths[robot_idx].poses.append(p)
        if len(self.paths[robot_idx].poses) > 500:
            self.paths[robot_idx].poses.pop(0)
        self.path_pubs[robot_idx].publish(self.paths[robot_idx])

    def control_loop(self):
        if not all(self.position_received):
            return

        for i in range(self.num_robots):
            u_x, u_y = 0.0, 0.0
            
            # 1. Formation Consensus Logic 
            for j in range(self.num_robots):
                if self.A[i][j] == 1:
                    # (x_j - x_i) - (xi_j - xi_i)
                    u_x += (self.positions[j][0] - self.positions[i][0]) - (self.offsets[j][0] - self.offsets[i][0])
                    u_y += (self.positions[j][1] - self.positions[i][1]) - (self.offsets[j][1] - self.offsets[i][1])
            
            # 2. Virtual Leader Logic: Move formation center to target 
            # We treat the formation as a "stubborn robot" following the target_pos
            u_x += (self.target_pos[0] - self.positions[i][0]) + self.offsets[i][0]
            u_y += (self.target_pos[1] - self.positions[i][1]) + self.offsets[i][1]

            # 3. Collision Avoidance 
            col_x, col_y = 0.0, 0.0
            for j in range(self.num_robots):
                if i != j:
                    dx = self.positions[i][0] - self.positions[j][0]
                    dy = self.positions[i][1] - self.positions[j][1]
                    dist = math.sqrt(dx**2 + dy**2)
                    if dist < self.safe_dist and dist > 0.01:
                        force = self.k_sep * (1.0/dist - 1.0/self.safe_dist)
                        col_x += (dx/dist) * force
                        col_y += (dy/dist) * force

            # Combine and Saturate
            final_vx = (u_x * self.k_p) + col_x
            final_vy = (u_y * self.k_p) + col_y
            
            v_norm = math.sqrt(final_vx**2 + final_vy**2)
            if v_norm > 0.6: 
                final_vx = (final_vx / v_norm) * 0.6
                final_vy = (final_vy / v_norm) * 0.6

            msg = Twist()
            msg.linear.x = float(final_vx)
            msg.linear.y = float(final_vy)
            self.pubs[i].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FormationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
