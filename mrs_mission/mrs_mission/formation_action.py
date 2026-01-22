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
        
        # FIXED: Replaced single-goal tracking with dictionary for multiple concurrent goals
        # OLD: self.prev_task_id, self._goal_handle, self.formation_complete, etc. were shared
        # NEW: Each goal gets its own state dictionary stored in self.active_goals
        self.active_goals = {}  # Dictionary to track multiple concurrent goals with per-goal state
        self.controller_timer = None
        
        # Shared robot state (position and velocity are shared across all goals)
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
        self.get_logger().info(f'Received goal request for task_id: {goal_request.task_id}')
        
        # ADDED: Check if this task_id is already active to prevent duplicate processing
        if goal_request.task_id in self.active_goals:
            self.get_logger().warn(f'Task {goal_request.task_id} already active, rejecting')
            return GoalResponse.REJECT
            
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
    async def execute_callback(self, goal_handle: ServerGoalHandle):
        """Execute a goal."""
        # CHANGED: Removed old single-goal checks (formation_complete, prev_task_id)
        # Now using per-goal state management via self.active_goals dictionary
        
        task_id = goal_handle.request.task_id
        self.get_logger().info(f'Executing New Formation action for task_id: {task_id}...')

        # ADDED: Create goal-specific state dictionary to track this particular formation request
        # This allows multiple goals to run concurrently without interfering with each other
        goal_state = {
            'task_id': task_id,
            'spacing': goal_handle.request.spacing,
            'formation_center': [goal_handle.request.formation_center_x, 
                                goal_handle.request.formation_center_y],
            'desired_shape': goal_handle.request.desired_shape,
            'robot_ids': np.array(goal_handle.request.robot_ids) - 1,  # (list to array) robot ids which are part of the requested formation
            'num_formation_robots': len(goal_handle.request.robot_ids),
            'formation_complete': False,
            'goal_handle': goal_handle,


            'MIN_DIFF': 0.25    # minimum distance to desired position in offset matrix to allow for formation completion
        }

        num_robots = goal_state['num_formation_robots']
        
        # Create adjacency matrix: Fully connected for stable formation
        goal_state['A'] = np.ones((num_robots, num_robots)) - np.eye(num_robots)  # Adjacency Matrix: Fully connected for stable formation 
        
        # Get formation offsets
        if goal_state['desired_shape'] == 'S':
            goal_state['formation_offsets'] = get_formation_offset_matrix_square(
                goal_state['desired_shape'], 
                num_robots, 
                vleader_pos=goal_state['formation_center'], 
                spacing=goal_state['spacing'])
        else: # other shapes
            goal_state['formation_offsets'] = get_formation_offset_matrix_mission(
                goal_state['desired_shape'], 
                num_robots, 
                vleader_pos=goal_state['formation_center'], 
                spacing=goal_state['spacing'])
            
        if not isinstance(goal_state['formation_offsets'], np.ndarray):
            self.get_logger().info("Wrong formation provided (or wrong formation to robot number) compatibility. Please check Formation.action file for restrictions.\nSupplying zeros for offsets...")
            # CHANGED: Instead of continuing with zeros, abort the goal
            result = Formation.Result()
            result.formation_complete = False
            goal_handle.abort()
            return result

        # ADDED: Store this goal's state in the active goals dictionary
        self.active_goals[task_id] = goal_state

        # CHANGED: Start control loop only if not already running (shared across all goals)
        # The control loop will process all active goals in self.active_goals
        if self.controller_timer is None:
            self.controller_timer = self.create_timer(
                1.0 / self.publish_rate, 
                self.control_loop, 
                callback_group=ReentrantCallbackGroup())
        
        rate = self.create_rate(10)  # 10 Hz polling 

        # Wait for completion (non-blocking with async)
        while not goal_state['formation_complete']:
            # ADDED: Check for cancellation during execution
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info(f'Goal {task_id} canceled')
                del self.active_goals[task_id]  # Remove from active goals
                result = Formation.Result()
                result.formation_complete = False
                return result
                
            # await asyncio.sleep(0.1)
            rate.sleep()  # This is non-blocking w.r.t. ROS time & allows executor to breathe

        # ADDED: Clean up goal state after completion
        del self.active_goals[task_id]
        
        # ADDED: Stop timer if no more active goals to save resources
        if not self.active_goals and self.controller_timer:
            self.controller_timer.cancel()
            self.controller_timer = None

        # Return result
        result = Formation.Result()
        result.formation_complete = True
        goal_handle.succeed()
        return result

    # Main control loop
    def control_loop(self):
        # Safety check: Wait until position and velocity updates have been received from all robots
        # before starting the control algorithm. This ensures all states are initialized.
        if not (all(self.position_received) and all(self.velocity_received)):
            self.get_logger().info("Not all robot positions and velocities have been received. Awaiting before start...")
            return
        
        # CHANGED: Removed single-goal cancellation check
        # OLD: if self._goal_handle.is_cancel_requested
        # NEW: Cancellation is handled in execute_callback for each goal individually
        
        # CHANGED: Process all active goals instead of just one
        # This loop iterates through all concurrent formation requests
        for task_id, goal_state in list(self.active_goals.items()):
            # Skip if this goal is already completed
            if goal_state['formation_complete']:
                continue

            # Extract goal-specific parameters from the goal_state dictionary
            goal_handle = goal_state['goal_handle']
            robot_ids = goal_state['robot_ids']
            formation_offsets = goal_state['formation_offsets']
            A = goal_state['A']
            desired_shape = goal_state['desired_shape']

            # FIXED: Create fresh feedback message for each control loop iteration
            # OLD: self.feedback_msg was reused and appended to, causing data accumulation
            # NEW: Create a new feedback message each time to avoid duplicates
            feedback_msg = Formation.Feedback()
            feedback_msg.robot_curr_positions = []
            feedback_msg.robot_ids = []

            # Process each robot independently
            for index, element in enumerate(robot_ids):    # enumerate is used here because of varying sizes of arrays that have robot info
                a_x, a_y = 0.0, 0.0     # Initialising acceleration value

                # Compute acceleration (for the case where the robot is simply moving to a designated position ('A' - alone))
                if desired_shape == "A":
                    a_x += formation_offsets[index][0] - self.positions[element][0] 
                    a_y += formation_offsets[index][1] - self.positions[element][1] 

                else:
                    # Compute acceleration based on formation control consensus formula
                    for jindex, jelement in enumerate(robot_ids):
                        # Only consider robots in communication range (defined by adjacency matrix A)
                        # A[i,j] == 1 means robot i should track information from robot j
                        if A[index][jindex] == 1:
                            a_x += (self.positions[jelement][0] - self.positions[element][0]) - (formation_offsets[jindex][0] - formation_offsets[index][0])
                            a_y += (self.positions[jelement][1] - self.positions[element][1]) - (formation_offsets[jindex][1] - formation_offsets[index][1])

                # Proportional gain
                a_x = a_x * self.k_p
                a_y = a_y * self.k_p 

                # ADDING centroid tracker (for MR tasks)
                if desired_shape != 'A':
                    k_centroid = 0.7
                    centroid_error_x = goal_state['formation_center'][0] - np.mean(self.positions[robot_ids, 0])
                    centroid_error_y = goal_state['formation_center'][1] - np.mean(self.positions[robot_ids, 1])
                    
                    a_x += k_centroid * centroid_error_x   # add to every robot, k_centroid = 0.5–1.0
                    a_y += k_centroid * centroid_error_y 

                # Add collision avoidance component
                col_acc_x, col_acc_y  = 0.0, 0.0
                robot_list = []
                for x in range(self.num_robots):
                    robot_list.append(x)
                for jindex, jelement in enumerate(robot_list):
                    if element == jelement:
                        continue    # Skip self-repulsion (a robot doesn't repel from itself)
                    
                    # Calculate distance towards neighboring robot
                    dx = self.positions[element][0] - self.positions[jelement][0]
                    dy = self.positions[element][1] - self.positions[jelement][1]
                    self.get_logger().error(f"T{task_id}, R{element} distance: {np.linalg.norm([dx, dy])}", throttle_duration_sec=3.0)

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
                # CHANGED: Now building feedback for this specific goal's robots only
                p = Point()
                p.x = self.positions[element][0]
                p.y = self.positions[element][1]
                feedback_msg.robot_curr_positions.append(p)
                feedback_msg.robot_ids.append(int(element))
                
            # Publish feedback for this goal
            goal_handle.publish_feedback(feedback_msg)

            # Check if formation has been achieved
            indexed_poses = self.positions[robot_ids]
            
            # FIXED: Corrected formation completion check
            # OLD: np.linalg.norm(np.linalg.norm(indexed_poses - formation_offsets))
            #      This applied norm twice, which is incorrect
            # NEW: Single norm calculation of the position error matrix
            position_error = np.linalg.norm(indexed_poses - formation_offsets)
            # position_error2 = np.mean(indexed_poses, axis=1) - np.array([goal_state['formation_center'][0], goal_state['formation_center'][1]])
            # position_error2 = np.linalg.norm(position_error2)
            # position_error = min(position_error, position_error2)
            # self.get_logger().info(f"{position_error:.2f}, {position_error2:.2f}")

            self.get_logger().warn(f"position error for the task {position_error}, for the task id of {task_id}", throttle_duration_sec=2.0)
            
            if position_error <= goal_state['MIN_DIFF']:
                # publish zero velocities when position is reached (simple 'keep_position')
                for rid in robot_ids:
                    self.publish_vel(0.0, 0.0, rid)
                goal_state['formation_complete'] = True
                self.get_logger().warn(f"formation compelte for the {task_id}")
                self.get_logger().info(f"Formation T{task_id} completed!")
            

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

    node = None
    executor = None

    try:
        node = FormationAction()
        executor = MultiThreadedExecutor()

        rclpy.spin(node, executor=executor)

    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if executor is not None:
            executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()