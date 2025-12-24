import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from crazyflie_interfaces.msg import LogDataGeneric
import numpy as np
import math
from typing import Dict, List

# importing helper functions
from mrs_consensus_improved.utils.helper import *

class FormationController(Node):
    def __init__(self):
        super().__init__('formation_contoller')

        # Declaring parameters to ROS
        self.declare_parameter('num_robots', 4)
        self.declare_parameter('publish_rate', 10.0) # in Hz
        self.declare_parameter('max_acc', 2.0)
        self.declare_parameter('k_p', 0.8)
        self.declare_parameter('k_d', 0.8)
        self.declare_parameter('k_separation', 1.0)
        self.declare_parameter('safe_dist', 0.5)
        self.declare_parameter('spacing', 0.5)
        self.declare_parameter('target_pos_x', 1.5)
        self.declare_parameter('target_pos_y', 1.5)
        self.declare_parameter('vleader_x', 0.0)
        self.declare_parameter('vleader_y', 0.0)
        self.declare_parameter('desired_shape', 'L')

        # Load parameters
        self.num_robots = self.get_parameter('num_robots').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.max_acc = self.get_parameter('max_acc').value
        self.k_p = self.get_parameter('k_p').value
        self.k_d = self.get_parameter('k_d').value
        self.k_separation = self.get_parameter('k_separation').value
        self.safe_dist = self.get_parameter('safe_dist').value
        self.spacing = self.get_parameter('spacing').value
        self.target_pos = np.array([self.get_parameter('target_pos_x').value, self.get_parameter('target_pos_y').value])
        self.vleader_pos = [self.get_parameter('vleader_x').value, self.get_parameter('vleader_y').value]
        self.desired_shape = self.get_parameter('desired_shape').value

        # ATTRIBUTES
        self.formation_offsets = get_formation_offset_matrix(self.desired_shape, self.num_robots, vleader_pos=self.vleader_pos, spacing=self.spacing)
        if not isinstance(self.formation_offsets, np.ndarray):
            self.get_logger().info("Wrong formation provided (or wrong formation to robot number) compatibility. Please check params file for restrictions.\nSupplying zeros for offsets...")
            self.formation_offsets = np.zeros((self.num_robots, 2))
        self.positions = np.zeros((self.num_robots, 2))
        self.velocities = np.zeros((self.num_robots, 2))
        self.position_received = [False] * self.num_robots  # Track which robots have sent at least one position update
        self.velocity_received = [False] * self.num_robots  # Track which robots have sent at least one velocity update
        self.robot_poses: Dict[int, List[np.ndarray]] = {}  # Dictionary to append the robot poses, to be plotted at the end of the simulation run
        self.A = np.ones((self.num_robots, self.num_robots)) - np.eye(self.num_robots)  # # Adjacency Matrix: Fully connected for stable formation 

        # PUBLISHERS & SUBSCRIBERS
        self.vel_pubs = []
        self.vel_subs = []
        self.path_pubs = []
        self.subs = []
        self.paths = [Path() for _ in range(self.num_robots)] # for RViz path visualisation
        self.fixed_frame = "world"   # for RViz path visualisation    
        self.max_path_points = 2000        # Limit to avoid RViz slowdown/memory growth

        for i in range(self.num_robots):
            robot_id = i + 1    # Convert 0-indexed loop variable to 1-indexed robot ID (for topic naming)
            self.subs.append(self.create_subscription(PoseStamped, f'/cf_{robot_id}/pose', lambda msg, rid=i: self.pose_callback(msg, rid), 10)) # Robot pose subscription (rid -> robot id, with starting index=0)
            self.vel_pubs.append(self.create_publisher(Twist, f'/cf_{robot_id}/cmd_vel', 10))   # Robot velocity publisher
            self.vel_subs.append(self.create_subscription(LogDataGeneric, f'cf_{robot_id}/velocity', lambda msg, rid=i: self.vel_callback(msg, rid), 10)) # Robot velocity subsctiption (rid -> robot id, with starting index=0)
            self.path_pubs.append(self.create_publisher(Path, f'/cf_{robot_id}/path', 10))  # Robot paths for RViz visualisation
            self.paths[i].header.frame_id = self.fixed_frame    # setting the frame for robots' path publisher

        # TIMERS
        self.connection_timer = self.create_timer(3, self.connect_wait) # wait to allow publisher/subscriber connection

    # Startup timer callback function
    def connect_wait(self):
        self.destroy_timer(self.connection_timer) # stopping the 3-second wait
        self.timer = self.create_timer(1.0 / self.publish_rate, self.control_loop)
        self.get_logger().info(f"Formation Controller node started with {self.num_robots} robots with formation {self.desired_shape}")

    def pose_callback(self, msg:PoseStamped, rid:int):
        """
            Pose callback, with added functionality to publish path to RViz

            Parameters:
                msg: (PoseStamped) msg containing the pose of crazyflie with index 'rid'
                rid: (int) robot index (0 to num_robots-1) - cuz the control loop accesses each robot with index starting with 0
        """

        self.positions[rid] = [msg.pose.position.x, msg.pose.position.y]    # NOTE: We're controlling just x and y positions, because height is controlled and kept (to hover) at a set value (1.0m in crazyflie config)
        self.position_received[rid] = True  # Marking this robot as having sent at least one position update. This flag prevents the control loop from starting until all robots report position_received==True

        # Add pose to path list, and publish Rviz (so the robot path can be visualized)
        self.publish_path(msg, rid)

    def vel_callback(self, msg:LogDataGeneric, rid:int):
        self.velocities[rid] = [msg.values[0], msg.values[1]]   # # NOTE: We're controlling just x and y velocities. z velocity is controlled (to maintain 1.0m altitude) by vel_mux.py and crazyflie_server.py
        self.velocity_received[rid] = True

    # Main control loop
    def control_loop(self):
        # Safety check: Wait until position and velocity updates have been received from all robots
        # before starting the control algorithm. This ensures all states are initialized.
        if not (all(self.position_received) and all(self.velocity_received)):
            self.get_logger().info("Not all robot positions and velocities have been received. Awaiting before start...")
            return

        LOCAL_FORMATION_THRESHOLD = 1e-1

        # Process each robot independently
        for i in range(self.num_robots):
            a_x, a_y = 0.0, 0.0     # Initialising acceleration value
            local_err = 0.0
            local_count = 0


            # Compute acceleration based consensus formula
            for j in range(self.num_robots):
                # Only consider robots in communication range (defined by adjacency matrix A)
                # A[i,j] == 1 means robot i should track information from robot j
                if self.A[i][j] == 1:
                    a_x += (self.positions[j][0] - self.positions[i][0]) - (self.formation_offsets[j][0] - self.formation_offsets[i][0])
                    a_y += (self.positions[j][1] - self.positions[i][1]) - (self.formation_offsets[j][1] - self.formation_offsets[i][1])

                    local_err += ((self.positions[j][0] - self.positions[i][0]) - (self.formation_offsets[j][0] - self.formation_offsets[i][0]))**2 + ((self.positions[j][1] - self.positions[i][1]) - (self.formation_offsets[j][1] - self.formation_offsets[i][1]))**2
                    local_count += 1
            local_ready = (local_err / max(local_count, 1)) < LOCAL_FORMATION_THRESHOLD

            # Virtual Leader Logic: Move formation center to target 
            # We treat the formation as a "stubborn robot" following the target_pos
            if i == 0 and local_ready:
                desired_i = self.target_pos + self.formation_offsets[i]
                a_x += desired_i[0] - self.positions[i][0]
                a_y += desired_i[1] - self.positions[i][1]


            # Proportional gain
            a_x = a_x * self.k_p
            a_y = a_y * self.k_p

            # Add collision avoidance component
            col_acc_x, col_acc_y  = 0.0, 0.0
            for j in range(self.num_robots):
                if i == j:
                    continue    # Skip self-repulsion (a robot doesn't repel from itself)
                
                # Calculate distance towards neighboring robot
                dx = self.positions[i][0] - self.positions[j][0]
                dy = self.positions[i][1] - self.positions[j][1]

                sep_x, sep_y = compute_separation_acc(dx, dy, self.safe_dist, self.k_separation)

                col_acc_x += sep_x
                col_acc_y += sep_y                

            # Combine and cap acceleration components
            a_x = a_x + col_acc_x 
            a_y = a_y + col_acc_y 

            final_ax, final_ay = cap_acceleration(a_x, a_y, self.max_acc)

            # Compute damping velocity
            d_ax = self.k_d * self.velocities[i][0]                
            d_ay = self.k_d * self.velocities[i][1]

            # PUBLISH VELOCITY
            final_vx = self.velocities[i][0] + (final_ax * (1.0/self.publish_rate)) - d_ax
            final_vy = self.velocities[i][1] + (final_ay * (1.0/self.publish_rate)) - d_ay

            self.publish_vel(final_vx, final_vy, i)

    def publish_vel(self, vx, vy, rid):
        # Create ROS2 Twist message with supplied velocity commands
        msg = Twist()
        msg.linear.x = float(vx)   # Velocity in x direction (forward/backward)
        msg.linear.y = float(vy)   # Velocity in y direction (left/right)

        self.vel_pubs[rid].publish(msg)    # publish velocity to robot with index rid

    def publish_path(self, pose_msg:PoseStamped, rid:int):
        p = PoseStamped()
        p.header.stamp = pose_msg.header.stamp
        p.header.frame_id = self.fixed_frame
        p.pose = pose_msg.pose

        self.paths[rid].header.stamp = pose_msg.header.stamp
        self.paths[rid].poses.append(p)

        # Keep only the last max_path_points poses to avoid unlimited growth
        if len(self.paths[rid].poses) > self.max_path_points:
            self.paths[rid].poses.pop(0)

        self.path_pubs[rid].publish(self.paths[rid])


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