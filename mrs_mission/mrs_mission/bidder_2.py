import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.duration import Duration
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
import time
from collections import defaultdict

class Bidder2(Node):        # Bidder node for robot 2
    def __init__(self):
        super().__init__('bidder2_node')

        # Declaring paramers to ROS
        self.declare_parameter('num_robots', 4)
        self.declare_parameter('publish_rate', 10.0) # in Hz
        self.declare_parameter('const_speed', 1.0)
        self.declare_parameter('spacing', 0.5)

        # Load parameters
        self.num_robots = self.get_parameter('num_robots').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.const_speed = self.get_parameter('const_speed').value  
        self.formation_spacing = self.get_parameter('spacing').value

        # ATTRIBUTES
        # self.rtasks_progress: Dict[int, Tuple[int, bool]] = {}   # robot_id, task_id, task_completed
        self.rtasks_progress: Dict[int, Dict[int, int]] = defaultdict(dict)     # robot_id, task_id, tasklevel 
        self.tasks_progress: Dict[int, bool] = {}   # task_id, task_completed
        self.position = np.zeros((1, 2))
        self.position_received = False
        self.robot_id = 2   # hardcoded for all bidders
        self.neighbours: Dict[int, List[int]] = {}  # neighbours (for each task) received from dedicated topic. structure: task_id, [neighbours] 
        self.task_remaining = False     # flag to check if all tasks have been assigned
        self.task_schedule:List[Tuple[int, float, float, List[float], int, str, str]] = []       # schedule for task allocated to robot (task_id, makespan, task_duration, task_location, num_required, task_formation, task_type)
        self.precedence_tasks: Dict[int, List[int]] = {}
        self.last_bid:TaskBid.Request = None    # to keep the last bid messge which will be resent if not all bida have been received
        self.curr_task = None   # Current task in task schedule being handled
        self.task_requested = False
        self.prev_task_complete = False
        self.goal_sent = False      # to check if one formation request has been sent. if yes, then don't send again. This is to avoid multiple formation requests
        self.prev_taskid = None
        self.task_level = defaultdict(int)

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
            self.tp_subs.append(self.create_subscription(TaskProgress, f"/cf_{robot_id}/task_progress", lambda msg, rid=i: self.tp_callback(msg, rid), 5))

        # SERVICE CLIENT
        self.bid_client = self.create_client(TaskBid, 'task_bid')
        while not self.bid_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Bid server not available. Waiting...')

        # ACTION CLIENT
        self._action_client = ActionClient(self, Formation, "formation_action_node") 

        # TIMERS
        self.control_loop_timer = self.create_timer(1/self.publish_rate, self.do_task)  # timer to carry out assigned tasks
        self.get_logger().info(f"Bidder_{self.robot_id} node started...")
        

    def pose_callback(self, msg:PoseStamped):
        self.position[0][0] = msg.pose.position.x
        self.position[0][1] = msg.pose.position.y

        self.position_received = True

    def tp_callback(self, msg: TaskProgress, rid):
        # store per-robot tasklevel
        self.rtasks_progress[rid+1][msg.task_id] = msg.tasklevel

        # MONOTONIC global completion: keep the max level ever seen
        self.task_level[msg.task_id] = max(self.task_level[msg.task_id], msg.tasklevel)

        # (optional) keep your old bool too, but make it monotonic
        self.tasks_progress[msg.task_id] = self.tasks_progress.get(msg.task_id, False) or msg.task_completed


    def get_neighbours_callback(self, msg:Neighbours):
        # Function to update the list of neighbours for each task, to use for task coordination 
        # Note that the list of neighbours includes the bidder id as well
        self.neighbours[msg.task_id] = list(msg.neighbours)
        self.get_logger().info(f"{self.neighbours}")
    
    def tr_callback(self, msg:TaskRemaining):
        self.task_remaining = msg.all_tasks_allocated
        if self.task_remaining == True:
            self.get_logger().info(f"All tasks have been assigned. Total No. of tasks: {msg.n_tasks}")
            for i in range(msg.n_tasks):
                self.tasks_progress[i+1] = False    # initialising all tasks as not completed (NOTE: task ids start at 1)
            self.get_logger().info(f"{self.get_name()}: Task Schedule: {self.task_schedule}")
            # Destroy the sbscription
            self.destroy_subscription(self.ta_sub)
            self.ta_sub = None

    def get_new_task_callback(self, msg:TaskAnnouncement):
        # Function to get new task announcement and publish bid for it
        dist_to_task = np.linalg.norm(self.position - np.array([msg.task_location.x, msg.task_location.y]).reshape(1,2))
        duration = msg.task_time[0]         # [DU, ES, LF]
        if len(self.task_schedule) > 0:
            dist_to_task = np.linalg.norm(np.array(self.task_schedule[-1][3]).reshape(1,2) - np.array([msg.task_location.x, msg.task_location.y]).reshape(1,2)) 
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

            # Add task to schedule (structure: task_id, makespan, task_location, num_required, task_formation, task_type)
            self.task_schedule.append((msg.task_id, task_start_time+duration, duration, [msg.task_location.x, msg.task_location.y], msg.num_required, msg.task_formation, msg.task_type))

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
                    self.precedence_tasks[response.task_id] = list(response.precedence_tasks)

        except Exception as e:
            self.get_logger().error(f"{self.get_name()}: Service call failed: {e}")

    
    def do_task(self):
        # Don't start until auctioneer says all tasks allocated
        if not self.task_remaining:
            return

        # If we just finished a task, pop it and reset flags
        if self.prev_task_complete:
            if self.curr_task is not None:
                self.prev_taskid = self.curr_task[0]
            self.goal_sent = False
            if self.task_schedule:
                self.task_schedule.pop(0)
            self.prev_task_complete = False

        # Nothing left
        if not self.task_schedule:
            return

        # IMPORTANT: handle ONLY the first task in the schedule
        self.curr_task = self.task_schedule[0]
        task_id = self.curr_task[0]
        task_type = self.curr_task[-1]   # 'SR' or 'MR'

        # ---------------- Precedence constraints (shared SR/MR) ----------------
        if task_id in self.precedence_tasks:
            # Copy list so we don't mutate the dict value
            prec_tasks = list(self.precedence_tasks[task_id])

            # Remove "self" if it exists (fixes deadlock like [7,1,2])
            prec_tasks = [t for t in prec_tasks if t != task_id]

            # Optional: don't wait on the task we *just* finished (also don't mutate)
            if self.prev_taskid is not None:
                prec_tasks = [t for t in prec_tasks if t != self.prev_taskid]

            # Use .get() to avoid KeyError if some task id not initialized yet
            if not all(self.tasks_progress.get(t, False) for t in prec_tasks):
                return

        # ---------------- Execute SR task ----------------
        if task_type == 'SR':
            if not self.goal_sent:
                self.send_goal(
                    task_id,
                    self.curr_task[5],                 # shape
                    self.formation_spacing,
                    self.curr_task[3][0],              # x
                    self.curr_task[3][1],              # y
                    [self.robot_id]
                )
            return

        # ---------------- Execute MR task ----------------
        # Wait until neighbours list is received for this task
        if task_id not in self.neighbours or not self.neighbours[task_id]:
            return

        neighbours = list(self.neighbours[task_id])  # usually includes self

        # (Optional) simple leader election: lowest robot_id requests the formation
        leader = min(neighbours)

        # If any neighbour is currently doing some other task (tasklevel==1), wait
        for n in neighbours:
            if n == self.robot_id:
                continue
            # if neighbour has ANY ongoing task that is not this one, don't start MR yet
            if any(level == 1 for tid, level in self.rtasks_progress[n].items() if tid != task_id):
                self.get_logger().info(
                    f"Waiting: neighbour {n} still busy with another task {self.rtasks_progress[n]}",
                    throttle_duration_sec=2.0
                )
                return

        # If someone (leader or others) already started this MR task, just track it
        someone_started = any(
            self.rtasks_progress[n].get(task_id, 0) == 1
            for n in neighbours
            if n != self.robot_id
        )

        someone_finished = any(
            self.rtasks_progress[n].get(task_id, 0) == 2
            for n in neighbours
            if n != self.robot_id
        )

        # If someone finished, mark complete locally
        if someone_finished:
            self.get_logger().info(f"T{task_id} MR task complete (tracked)!")
            self.prev_task_complete = True
            self.goal_sent = False

            tp = TaskProgress()
            tp.robot_id = self.robot_id
            tp.task_completed = True
            tp.task_id = task_id
            tp.tasklevel = 2
            self.tp_publisher.publish(tp)
            return

        # If someone started, publish "ongoing" and wait
        if someone_started:
            self.goal_sent = True
            tp = TaskProgress()
            tp.robot_id = self.robot_id
            tp.task_completed = False
            tp.task_id = task_id
            tp.tasklevel = 1
            self.tp_publisher.publish(tp)
            return

        # Otherwise nobody started yet -> leader sends the goal
        if not self.goal_sent and self.robot_id == leader:
            self.send_goal(
                task_id,
                self.curr_task[5],                 # shape
                self.formation_spacing,
                self.curr_task[3][0],              # center x
                self.curr_task[3][1],              # center y
                neighbours                          # all robots in formation
            )
            return

        # Non-leader waits for leader to start
        return


    def send_goal(self, task_id, shape, spacing, center_x, center_y, robot_ids):
        """
            Send a formation goal to the formation action server

            Paramaters:
                - robot_ids: (list) list of robots for formation request. it is a list with a single value if the formation is to move to a position
        
        """
        
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        
        # Create goal message
        goal_msg = Formation.Goal()
        goal_msg.task_id = task_id
        goal_msg.spacing = spacing
        goal_msg.formation_center_x = center_x
        goal_msg.formation_center_y = center_y
        goal_msg.desired_shape = shape
        goal_msg.robot_ids = robot_ids
        
        self.get_logger().info(f'Sending goal: Task {task_id}, Shape={shape}, Robots={robot_ids}')

        # Set goal_sent flag
        self.goal_sent = True
        
        # Publish task progress message
        tp = TaskProgress()
        tp.robot_id = self.robot_id
        tp.task_completed = False
        tp.task_id = self.curr_task[0]  # remember task scheule structure: (task_id, makespan, duration, task_location, num_required, task_formation, task_type)
        tp.tasklevel = 1    # 0-notcomplete; 1-ongoing; 2-complete;
        self.tp_publisher.publish(tp)
        
        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def feedback_callback(self, feedback_msg):
        """Handle feedback during goal execution"""
        feedback = feedback_msg.feedback
        
        # Log current robot positions
        self.get_logger().info(
            f'(Feedback) Robots in formation: {len(feedback.robot_ids)}, T{self.task_schedule[0][0]}',
            throttle_duration_sec=2.0)  # Log every 2 seconds
        
        # Publish task progress message
        tp = TaskProgress()
        tp.robot_id = self.robot_id
        tp.task_completed = False
        tp.task_id = self.curr_task[0]  # remember task scheule structure: (task_id, makespan, duration, task_location, num_required, task_formation, task_type)
        tp.tasklevel = 1    # 0-notcomplete; 1-ongoing; 2-complete;
        self.tp_publisher.publish(tp)
        
    def goal_response_callback(self, future):
        """Handle the goal acceptance/rejection"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
            
        self.get_logger().info('Goal accepted')
        
        # Wait for result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        """Handle the final result"""
        result = future.result().result
        
        if result.formation_complete:
            # Publish task progress message
            tp = TaskProgress()
            tp.robot_id = self.robot_id
            tp.task_completed = True
            tp.task_id = self.curr_task[0]  # remember task schedule structure: (task_id, makespan, duration, task_location, num_required, task_formation, task_type)
            tp.tasklevel = 2    # 0-notcomplete; 1-ongoing; 2-complete;
            self.tp_publisher.publish(tp)

            self.prev_task_complete = True
            self.get_logger().info('Formation completed request successfully!')

            # wait at position for task_duration time, before moving to next task
            self.get_clock().sleep_for(Duration(seconds=self.task_schedule[0][2]))
        else:                
            self.task_requested = True
            self.get_logger().info('Formation request did not complete')

            # Publish task progress message
            tp = TaskProgress()
            tp.robot_id = self.robot_id
            tp.task_completed = False
            tp.task_id = self.curr_task[0]  # remember task scheule structure: (task_id, makespan, duration, task_location, num_required, task_formation, task_type)
            tp.tasklevel = 0    # 0-notcomplete; 1-ongoing; 2-complete;
            self.tp_publisher.publish(tp)
            


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
    bidder2_node = Bidder2()
    try:
        rclpy.spin(bidder2_node)
    except KeyboardInterrupt:
        pass
    finally:
        bidder2_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
