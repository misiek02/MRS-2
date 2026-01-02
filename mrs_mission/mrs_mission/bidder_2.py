import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from mrs_interfaces.msg import Neighbours       # published only by auctioneer
from mrs_interfaces.msg import TaskAnnouncement # published only by auctioneer
from mrs_interfaces.msg import TaskRemaining    # published only by auctioneer
from mrs_interfaces.msg import TaskProgress     # published by all bidders
from mrs_interfaces.srv import TaskBid
from mrs_interfaces.msg import TaskBidMsg       # published by all bidders
from mrs_interfaces.action import Formation
import numpy as np
from typing import Dict, List, Tuple


class Bidder2(Node):        # Bidder node for robot 2
    def __init__(self):
        super.__init__('bidder2_node')

        # Declaring paramers to ROS
        self.declare_parameter('num_robots', 4)
        self.declare_parameter('publish_rate', 10.0) # in Hz
        self.declare_parameter('const_speed', 1.0)

        # Load parameters
        self.num_robots = self.get_parameter('num_robots').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.const_speed = self.get_parameter('const_speed').value

        # ATTRIBUTES
        self.tasks_progress = Dict[int, Tuple[int, bool]] = {}   # robot_id, task_id, task_completed
        self.position = np.zeros((1, 2))
        self.position_received = False
        self.robot_id = 2   # hardcoded for all bidders
        self.neighbours: Dict[int, List[int]] = {}  # neighbours (for each task) received from dedicated topic 
        self.task_remaining = False     # flag to check if all tasks have been assigned
        self.task_schedule = None

        # PUBLISHERS
        self.tp_publisher = self.create_publisher(TaskProgress, f"/cf_{self.robot_id}/task_progress", 10)
        self.bid_publisher = self.create_publisher(TaskBidMsg, f'/cf_{self.robot_id}/bid', 10)

        # SUBSCRIBERS
        self.tp_subs = []
        self.pos_sub = self.create_subscription(PoseStamped, f'/cf_{self.robot_id}/pose', self.pose_callback, 10)
        self.ta_sub = self.create_subscription(TaskAnnouncement, "task_announcement", self.get_new_task_callback, 10)
        self.tr_sub = self.create_subscription(TaskRemaining, "task_remaining", self.tr_callback, 10)
        self.neighbours_sub = self.create_subscription(Neighbours, "neighbours", self.get_neighbours_callback, 10)
        for i in range(self.num_robots):
            robot_id = i+1  # Convert 0-indexed loop variable to 1-indexed robot ID (for topic naming)
            self.tp_subs.append(self.create_subscription(TaskProgress, f"/cf_{self.robot_id}/task_progress", lambda msg, rid=i: self.tp_callback(msg, rid), 10))

        # ACTION CLIENT
        self._action_client = ActionClient(self, Formation, "formation_action_node") 

        # TIMERS
        self.connection_timer = self.create_timer(5, self.connect_wait) # wait to allow publisher/subscriber connection


    # Startup timer callback function
    def connect_wait(self):
        self.destroy_timer(self.connection_timer) # stopping the 5-second wait
        # self.timer = self.create_timer(1.0 / self.publish_rate, self.control_loop)
        self.get_logger().info(f"Bidder_{self.robot_id} node started...")

    def pose_callback(self, msg:PoseStamped):
        self.position[0][0] = msg.pose.position.x
        self.position[0][1] = msg.pose.position.y

        self.position_received = True

    def tp_callback(self, msg:TaskProgress, rid):
        # Function to get and store all the task progress messages from other robots
        self.tasks_progress[rid+1] = (msg.task_id, msg.task_completed)

    def get_neighbours_callback(self, msg:Neighbours):
        # Function to update the list of neighbours for each task, to use for task coordination 
        self.neighbours[msg.task_id] = [msg.neighbours]
    
    def tr_callback(self, msg:TaskRemaining):
        self.task_remaining = msg.all_tasks_allocated

    def get_new_task_callback(self, msg:TaskAnnouncement):
        # Function to get new task announcement and publish it
        