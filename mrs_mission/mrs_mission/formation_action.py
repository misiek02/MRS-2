import rclpy
from rclpy.node import Node
from rclpy.action.server import ServerGoalHandle
from geometry_msgs.msg import PoseStamped, Twist, Point
from nav_msgs.msg import Path
from crazyflie_interfaces.msg import LogDataGeneric
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import asyncio
from typing import Dict, List
from mrs_interfaces.action import Formation
# from mrs_interfaces.msg import for

# importing helper functions
from mrs_consensus_improved.utils.helper import *

class FormationAction(Node):    # Formation Action Server (can handle multiple requests)
    def __init__(self):
        super().__init__('formation_action_node')

        # Declaring parameters to ROS
        self.declare_parameter('num_robots', 4)
        self.declare_parameter('publish_rate', 10.0) # in Hz
        self.declare_parameter('max_acc', 2.0)
        self.declare_parameter('k_p', 0.8)
        self.declare_parameter('k_d', 0.8)
        self.declare_parameter('k_separation', 1.0)
        self.declare_parameter('safe_dist', 0.5)

        # Load parameters
        self.num_robots = self.get_parameter('num_robots').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.max_acc = self.get_parameter('max_acc').value
        self.k_p = self.get_parameter('k_p').value
        self.k_d = self.get_parameter('k_d').value
        self.k_separation = self.get_parameter('k_separation').value
        self.safe_dist = self.get_parameter('safe_dist').value

        # ATTRIBUTES
        self._actionname = 'formation_action_node'  # same as node name declared above
        self.controller_timer = None
        self._goal_handle = None
        self.spacing = None
        self.formation_center = None
        self.desired_shape = None
        self.num_formation_robots = None
        self.robot_ids = None
        self.formation_complete = False
        self.robot_curr_positions = []
        self.formation_offsets = None
        self.A = None   # adjency matrix
        self.MIN_DIFF = 1e-2    # minimum distance to desired position in offset matrix to allow for formation completion

        self.positions = np.zeros((self.num_robots, 2))
        self.velocities = np.zeros((self.num_robots, 2))
        self.position_received = [False] * self.num_robots  # Track which robots have sent at least one position update
        self.velocity_received = [False] * self.num_robots  # Track which robots have sent at least one velocity update
        
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

        # ACTION
        self._action_server = ActionServer(self, Formation, self._actionname, self.execute_callback, callback_group=ReentrantCallbackGroup(), goal_callback=self.goal_callback, cancel_callback=self.cancel_callback)     # if sync issues occur, check out the qos thingy

        # TIMERS
        self.connection_timer = self.create_timer(3, self.connect_wait) # wait to allow publisher/subscriber connection


    # added this function because it was used in example :>)
    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    # Startup timer callback function
    def connect_wait(self):
        self.destroy_timer(self.connection_timer) # stopping the 3-second wait
        self.get_logger().info(f"Formation Action Server started and awaiting a formation request.")

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

    # Execute callback function for Formation action
    async def execute_callback(self, goal_handle: ServerGoalHandle[Formation]):
        # Check if formation has been made.
        if self.formation_complete is True:
            goal_handle.canceled()
            self.get_logger().info("Formation has been reached")
            res = Formation.Result()
            res.formation_complete = True
            return res

        """Execute a goal."""
        self.get_logger().info('Executing New Formation action...')

        # Create feedback message object
        self.feedback_msg = Formation.Feedback()
        self.feedback_msg.robot_curr_positions = []
        self.feedback_msg.robot_ids = []

        # Result instance
        self._result = Formation.Result()

        # Get parameters for formation request
        self._goal_handle = goal_handle
        self.spacing = goal_handle.request.spacing
        self.formation_center = [goal_handle.request.formation_center_x, goal_handle.request.formation_center_y]
        self.desired_shape = goal_handle.request.desired_shape
        self.robot_ids = np.array(goal_handle.request.robot_ids) - 1  # (list to array) robot ids which are part of the requested formation
        self.num_formation_robots = len(self.robot_ids)

        self.A = np.ones((self.num_formation_robots, self.num_formation_robots)) - np.eye(self.num_formation_robots)  # Adjacency Matrix: Fully connected for stable formation 
        self.formation_offsets = get_formation_offset_matrix(self.desired_shape, self.num_formation_robots, vleader_pos=self.formation_center, spacing=self.spacing)
        if not isinstance(self.formation_offsets, np.ndarray):
            self.get_logger().info("Wrong formation provided (or wrong formation to robot number) compatibility. Please check Formation.action file for restrictions.\nSupplying zeros for offsets...")
            self.formation_offsets = np.zeros((self.num_formation_robots, 2))

        # Start the formation control loop
        self.controller_timer = self.create_timer(1.0 / self.publish_rate, self.control_loop, callback_group=ReentrantCallbackGroup())

        # Wait for completion (non-blocking with async)
        while not self.formation_complete:
            await asyncio.sleep(0.1)

        # Return result
        return self._result

    # Main control loop
    def control_loop(self):
        # Safety check: Wait until position and velocity updates have been received from all robots
        # before starting the control algorithm. This ensures all states are initialized.
        if not (all(self.position_received) and all(self.velocity_received)):
            self.get_logger().info("Not all robot positions and velocities have been received. Awaiting before start...")
            return
        
        if self._goal_handle.is_cancel_requested:
            self._goal_handle.canceled()
            self.controller_timer.cancel()
            self.get_logger().info('Goal canceled')
            return
        
        # Check if formation is completed
        if self.formation_complete is True:
            self._goal_handle.succeed()
            self.controller_timer.cancel()
            return

        self.formation_complete = False
        # Process each robot independently
        for index, element in enumerate(self.robot_ids):    # enumerate is used here because of varying sizes of arrays that have robot info
            a_x, a_y = 0.0, 0.0     # Initialising acceleration value


            # Compute acceleration based on formation control consensus formula
            for jindex, jelement in enumerate(self.robot_ids):
                # Only consider robots in communication range (defined by adjacency matrix A)
                # A[i,j] == 1 means robot i should track information from robot j
                if self.A[index][jindex] == 1:
                    a_x += (self.positions[jelement][0] - self.positions[element][0]) - (self.formation_offsets[jindex][0] - self.formation_offsets[index][0])
                    a_y += (self.positions[jelement][1] - self.positions[element][1]) - (self.formation_offsets[jindex][1] - self.formation_offsets[index][1])

            # Proportional gain
            a_x = a_x * self.k_p
            a_y = a_y * self.k_p

            # Add collision avoidance component
            col_acc_x, col_acc_y  = 0.0, 0.0
            for jindex, jelement in enumerate(self.robot_ids):
                if element == jelement:
                    continue    # Skip self-repulsion (a robot doesn't repel from itself)
                
                # Calculate distance towards neighboring robot
                dx = self.positions[element][0] - self.positions[jelement][0]
                dy = self.positions[element][1] - self.positions[jelement][1]

                sep_x, sep_y = compute_separation_acc(dx, dy, self.safe_dist, self.k_separation)

                col_acc_x += sep_x
                col_acc_y += sep_y                

            # Combine and cap acceleration components
            a_x = a_x + col_acc_x 
            a_y = a_y + col_acc_y 

            final_ax, final_ay = cap_acceleration(a_x, a_y, self.max_acc)

            # Compute damping velocity
            d_ax = self.k_d * self.velocities[element][0]                
            d_ay = self.k_d * self.velocities[element][1]

            # PUBLISH VELOCITY
            final_vx = self.velocities[element][0] + (final_ax * (1.0/self.publish_rate)) - d_ax
            final_vy = self.velocities[element][1] + (final_ay * (1.0/self.publish_rate)) - d_ay

            self.publish_vel(final_vx, final_vy, element)

            # Create and publish action feedback
            for r in self.robot_ids:
                p = Point()
                p.x = self.positions[r][0]
                p.y = self.positions[r][1]
                self.feedback_msg.robot_curr_positions.append(p)
                self.feedback_msg.robot_ids.append(r)
            self._goal_handle.publish_feedback(self.feedback_msg)

        # Check if formation has been achieved
        indexed_poses = self.positions[self.robot_ids]
        if np.linalg.norm(np.linalg.norm(indexed_poses - self.formation_offsets)) <= self.MIN_DIFF:
            self.formation_complete = True
            self._result.formation_complete = True
            self.get_logger().info("Formation completed!")
            

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
    
    try:
        with rclpy.init(args=args):
            node = FormationAction()

            # Use a MultiThreadedExecutor to enable processing goals concurrently
            executor = MultiThreadedExecutor()

            rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:    # remove in case of issues....
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()