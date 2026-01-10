import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
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


class Bidder3(Node):        # Bidder node for robot 3
    def __init__(self):
        super().__init__('bidder3_node')

        # Declaring paramers to ROS
        self.declare_parameter('num_robots', 4)
        self.declare_parameter('publish_rate', 10.0) # in Hz
        self.declare_parameter('const_speed', 1.0)

        # Load parameters
        self.num_robots = self.get_parameter('num_robots').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.const_speed = self.get_parameter('const_speed').value  

        # ATTRIBUTES
        self.tasks_progress: Dict[int, Tuple[int, bool]] = {}   # robot_id, task_id, task_completed
        self.position = np.zeros((1, 2))
        self.position_received = False
        self.robot_id = 3   # hardcoded for all bidders
        self.neighbours: Dict[int, List[int]] = {}  # neighbours (for each task) received from dedicated topic 
        self.task_remaining = False     # flag to check if all tasks have been assigned
        self.task_schedule:List[Tuple[int, float, List[float], int, str]] = []       # schedule for task allocated to robot (task, makespan, task_location, num_required, task_type)
        self.precedence_tasks: Dict[int, List[int]] = {}
        self.last_bid:TaskBid.Request = None    # to keep the last bid messge which will be resent if not all bida have been received

        # PUBLISHERS
        self.tp_publisher = self.create_publisher(TaskProgress, f"/cf_{self.robot_id}/task_progress", 10)
        self.bid_publisher = self.create_publisher(TaskBidMsg, f'/cf_{self.robot_id}/bid', 10)

        # SUBSCRIBERS
        sub_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.tp_subs = []
        self.pos_sub = self.create_subscription(PoseStamped, f'/cf_{self.robot_id}/pose', self.pose_callback, 10)
        self.ta_sub = self.create_subscription(TaskAnnouncement, "task_announcement", self.get_new_task_callback, sub_qos)
        self.tr_sub = self.create_subscription(TaskRemaining, "task_remaining", self.tr_callback, sub_qos)
        self.neighbours_sub = self.create_subscription(Neighbours, "neighbours", self.get_neighbours_callback, sub_qos)
        for i in range(self.num_robots):
            robot_id = i+1  # Convert 0-indexed loop variable to 1-indexed robot ID (for topic naming)
            self.tp_subs.append(self.create_subscription(TaskProgress, f"/cf_{robot_id}/task_progress", lambda msg, rid=i: self.tp_callback(msg, rid), 10))

        # SERVICE CLIENT
        self.bid_client = self.create_client(TaskBid, 'task_bid')
        while not self.bid_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Bid server not available. Waiting...')

        # ACTION CLIENT
        # self._action_client = ActionClient(self, Formation, "formation_action_node") 

        # TIMERS
        # self.connection_timer = self.create_timer(5, self.connect_wait) # wait to allow publisher/subscriber connection
        self.get_logger().info(f"Bidder_{self.robot_id} node started...")


    # Startup timer callback function
    # def connect_wait(self):
    #     self.destroy_timer(self.connection_timer) # stopping the 5-second wait
    #     # self.timer = self.create_timer(1.0 / self.publish_rate, self.control_loop)
        

    def pose_callback(self, msg:PoseStamped):
        self.position[0][0] = msg.pose.position.x
        self.position[0][1] = msg.pose.position.y

        self.position_received = True

    def tp_callback(self, msg:TaskProgress, rid):
        # Function to get and store all the task progress messages from other robots
        self.tasks_progress[rid+1] = (msg.task_id, msg.task_completed)

    def get_neighbours_callback(self, msg:Neighbours):
        # Function to update the list of neighbours for each task, to use for task coordination 
        self.neighbours[msg.task_id] = msg.neighbours
    
    def tr_callback(self, msg:TaskRemaining):
        self.task_remaining = msg.all_tasks_allocated
        if self.task_remaining == True:
            self.get_logger().info("All tasks have been assigned.")
            self.get_logger().info(f"{self.get_name()}: Task Schedule: {self.task_schedule}")
            # Destroy the sbscription
            self.destroy_subscription(self.ta_sub)
            self.ta_sub = None

    def get_new_task_callback(self, msg:TaskAnnouncement):
        # Function to get new task announcement and publish bid for it
        dist_to_task = np.linalg.norm(self.position - np.array([msg.task_location.x, msg.task_location.y]).reshape(1,2))
        duration = msg.task_time[0]         # [DU, ES, LF]
        if len(self.task_schedule) > 0:
            dist_to_task = np.linalg.norm(np.array(self.task_schedule[-1][2]).reshape(1,2) - np.array([msg.task_location.x, msg.task_location.y]).reshape(1,2)) 
            task_start_time = max((self.task_schedule[-1][1] + dist_to_task)/self.const_speed, msg.task_time[1])
        else:
            task_start_time = max(dist_to_task/self.const_speed, msg.task_time[1])
        # if (msg.task_time[-1] >= (task_start_time + duration)) and msg.winner_selected is False:
        if msg.winner_selected is False:            
            # Publish bid as message (this is for the auctioneer node to handle things)
            self.pub_bid(msg.task_id, task_start_time)
            
            # make a bid service request
            bid_request = TaskBid.Request()
            bid_request.bidder_id = self.robot_id
            bid_request.task_id = msg.task_id
            bid_request.es = task_start_time  
            self.last_bid = bid_request     # saving the last bid incase it has to be resent
            self.send_bidrequest(bid_request)

            # Add task to schedule (structure: task_id, makespan, task_location, num_required, task_type)
            self.task_schedule.append((msg.task_id, task_start_time+duration, [msg.task_location.x, msg.task_location.y], msg.num_required, msg.task_type))

    def bid_response_callback(self, future):
        try:
            response:TaskBid.Response = future.result()
            self.get_logger().info(f"{self.get_name()}: Bid response gotten!")
            if response.all_received == False:      
                self.send_bidrequest(self.last_bid)     # resend last bid
            else:
                if response.assigned == False:
                    self.task_schedule.remove(next(s for s in self.task_schedule if s[0] == response.task_id))  # remove the task from the schedule
                else:
                    self.precedence_tasks[response.task_id] = response.precedence_tasks

        except Exception as e:
            self.get_logger().error(f"{self.get_name()}: Service call failed: {e}")


    def send_bidrequest(self, req:TaskBid.Request):
        # Function to send a bid service request, with a populated req object
        future = self.bid_client.call_async(req)
        future.add_done_callback(self.bid_response_callback)

    def pub_bid(self, task_id, es):
        # Function to publish new bid to message callback
        msg = TaskBidMsg()
        msg.bidder_id = self.robot_id
        msg.task_id = task_id
        msg.es = es
        self.bid_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    bidder3_node = Bidder3()
    try:
        rclpy.spin(bidder3_node)
    except KeyboardInterrupt:
        pass
    finally:
        bidder3_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
