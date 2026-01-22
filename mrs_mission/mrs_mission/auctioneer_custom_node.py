import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from mrs_interfaces.msg import Neighbours       # published only by auctioneer
from mrs_interfaces.msg import TaskAnnouncement # published only by auctioneer
from mrs_interfaces.msg import TaskRemaining    # published only by auctioneer
from mrs_interfaces.srv import TaskBid
from mrs_interfaces.msg import TaskBidMsg       # published by all bidders
import numpy as np
from typing import Dict, List
from threading import Event, Thread, Lock
import heapq
import time



class AuctioneerCustomNode(Node):     # Auctioneer node for task, also doubles as Bidder node for robot 1
    def __init__(self):
        super().__init__('auctioneer_custom_node')

        # Declaring parameters to ROS
        self.declare_parameter('num_robots', 4)
        self.declare_parameter('publish_rate', 10.0) # in Hz
        self.declare_parameter('const_speed', 1.0)
        self.declare_parameter('n_tasks', 10)
        self.declare_parameter('T1')
        self.declare_parameter('T2')
        self.declare_parameter('T3')
        self.declare_parameter('T4')
        self.declare_parameter('T5')
        self.declare_parameter('T6')
        self.declare_parameter('T7')
        self.declare_parameter('T8')
        self.declare_parameter('T9')
        self.declare_parameter('T10')
        self.declare_parameter('TF')
        self.declare_parameter('TL')
        self.declare_parameter('TL2')
        self.declare_parameter('TL3')
        self.declare_parameter('TH')
        self.declare_parameter('p_1')
        self.declare_parameter('p_2')
        self.declare_parameter('p_3')
        self.declare_parameter('p_4')
        self.declare_parameter('T7_shape')
        self.declare_parameter('T8_shape')
        self.declare_parameter('T9_shape')
        self.declare_parameter('T10_shape')
        self.declare_parameter('bid_timeout', 5.0)  # timeout for bid collection

        # Load parameters
        self.num_robots = self.get_parameter('num_robots').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.const_speed = self.get_parameter('const_speed').value
        self.n_tasks = self.get_parameter('n_tasks').value
        self.bid_timeout = self.get_parameter('bid_timeout').value
        self.task_info: Dict[int, List[float]] = {}     # STRUCTURE: [pos_x, pos_y, num_robots_required, duration, assigned?]
        for i in range(self.n_tasks):
            self.task_info[i+1] = self.get_parameter(f"T{i+1}").value
            self.task_info[i+1].append(False)
        self.tf_list = self.get_parameter('TF').value
        self.tl_list = self.get_parameter('TL').value
        self.tl2_list = self.get_parameter('TL2').value
        self.tl3_list = self.get_parameter('TL3').value
        self.th_list = self.get_parameter('TH').value
        self.p_1 = self.get_parameter('p_1').value
        self.p_2 = self.get_parameter('p_2').value
        self.p_3 = self.get_parameter('p_3').value
        self.p_4 = self.get_parameter('p_4').value
        self.T7_shape = self.get_parameter('T7_shape').value
        self.T8_shape = self.get_parameter('T8_shape').value
        self.T9_shape = self.get_parameter('T9_shape').value
        self.T10_shape = self.get_parameter('T10_shape').value

        # ATTRIBUTES
        self.position = np.zeros((1, 2))
        self.position_received = False
        # self.robot_id = 1   # hardcoded for all bidders
        self.neighbours = []
        self.task_assigned_event = Event()  # using 'event' to avoid blocking entire node
        self.all_bids_received_event = Event()  # separate event for bid collection
        self.bids: Dict[int, List[float]] = {}  # Dict of all received bids
        self.bids_lock = Lock()  # lock for thread-safe access to bids
        self.bids_received = [False] * self.num_robots
        self.current_task_id = None  # track which task we're currently processing
        self.bid_collection_start_time = None  # for timeout tracking
        self.service_responses_sent = 0  # track how many service responses have been sent
        self.p_constraints = np.array([self.p_1, self.p_2, self.p_3, self.p_4])

        # PUBLISHERS
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.neighbours_publisher = self.create_publisher(Neighbours, "neighbours", qos)
        self.ta_publisher = self.create_publisher(TaskAnnouncement, "task_announcement", qos)
        self.tr_publisher = self.create_publisher(TaskRemaining, "task_remaining", qos)
        # self.tp_publisher = self.create_publisher(TaskProgress, f"/cf_{self.robot_id}/task_progress", 10, qos_profile=qos)

        # SUBSCRIBERS
        self.bids_subs = []
        for i in range(self.num_robots):
            robot_id = i+1  # Convert 0-indexed loop variable to 1-indexed robot ID (for topic naming)
            self.bids_subs.append(self.create_subscription(TaskBidMsg, f'/cf_{robot_id}/bid', lambda msg, rid=i: self.bid_callback(msg, rid), 10))

        # ACTION CLIENT
        # self._action_client = ActionClient(self, Formation, "formation_action_node") 

        # SERVICES
        self.bid_srv = self.create_service(TaskBid, 'task_bid', self.choose_best_bid_callback)

        # TIMERS
        # self.connection_timer = self.create_timer(10, self.connect_wait) # wait to allow publisher/subscriber connection
        self.timeout_timer = self.create_timer(0.1, self.check_bid_timeout)  # timer to check for bid timeout
        self.get_logger().info(f"Auctioneer node started...")
    
    # Startup timer callback function
    # def connect_wait(self):
    #     self.destroy_timer(self.connection_timer) # stopping the 5-second wait
    #     # self.timer = self.create_timer(1.0 / self.publish_rate, self.control_loop)
    #     self.get_logger().info(f"Auctioneer/Bidder_{self.robot_id} node started...")

    def bid_callback(self, msg:TaskBidMsg, rid:int):
        """Handle incoming bids with thread safety"""
        with self.bids_lock:
            # Only process bids for the current task
            if self.current_task_id is not None and msg.task_id == self.current_task_id:
                self.bids[rid+1] = [msg.bidder_id, msg.task_id, msg.es]
                self.bids_received[rid] = True
                
                self.get_logger().info(
                    f"Received bid from robot {msg.bidder_id} for task {msg.task_id}: "
                    f"es={msg.es:.2f} ({sum(self.bids_received)}/{self.num_robots} bids)"
                )
                
                # Check if all bids received
                if all(self.bids_received):
                    self.get_logger().info(f"All bids received for task {self.current_task_id}")
                    self.all_bids_received_event.set()

    def check_bid_timeout(self):
        """Check if bid collection has timed out"""
        if self.bid_collection_start_time is not None:
            elapsed = time.time() - self.bid_collection_start_time
            if elapsed > self.bid_timeout and not self.all_bids_received_event.is_set():
                with self.bids_lock:
                    missing = [i+1 for i, received in enumerate(self.bids_received) if not received]
                    self.get_logger().warn(
                        f"Bid timeout for task {self.current_task_id}! "
                        f"Missing bids from robots: {missing}"
                    )
                    # Trigger event anyway to prevent hanging
                    self.all_bids_received_event.set()

    def wait_for_all_bids(self, task_id: int, timeout: float = None) -> bool:
        """Wait for all bids to be received with optional timeout"""
        if timeout is None:
            timeout = self.bid_timeout
        
        self.bid_collection_start_time = time.time()
        success = self.all_bids_received_event.wait(timeout=timeout + 0.5)
        self.bid_collection_start_time = None
        
        if not success:
            self.get_logger().error(f"Failed to collect all bids for task {task_id}")
        
        return success

    def reset_bid_collection(self, task_id: int):
        """Reset bid collection state for a new task"""
        with self.bids_lock:
            self.bids.clear()
            self.bids_received = [False] * self.num_robots
            self.current_task_id = task_id
            self.all_bids_received_event.clear()
            self.task_assigned_event.clear()
            self.bid_collection_start_time = None
            self.service_responses_sent = 0  # reset service response counter

    def publish_task(self, tid: int, es: float, lf: float) -> float:
        """Publish a single task and wait for assignment"""
        # Reset state for new task
        self.reset_bid_collection(tid)
        
        # Prepare task announcement
        ta_msg = TaskAnnouncement()
        ta_msg.task_id = tid
        ta_msg.task_location.x = self.task_info[tid][0]
        ta_msg.task_location.y = self.task_info[tid][1]
        ta_msg.num_required = int(self.task_info[tid][2])
        du = self.task_info[tid][3]
        new_lf = es + du + 20
        ta_msg.task_time = [float(du), float(es), float(new_lf)]
        ta_msg.winner_selected = self.task_info[tid][4]
        if self.task_info[tid][2] > 1:
            ta_msg.task_type = "MR"
        else:
            ta_msg.task_type = "SR"

        # Add task formation shape
        if tid == 7:
            ta_msg.task_formation = self.T7_shape
        elif tid == 8:
            ta_msg.task_formation = self.T8_shape
        elif tid == 9:
            ta_msg.task_formation = self.T9_shape
        elif tid == 10:
            ta_msg.task_formation = self.T10_shape
        else:
            ta_msg.task_formation = 'A'

        # publish current task
        self.ta_publisher.publish(ta_msg)
        self.get_logger().info(f"NEW TASK PUBLISHED WITH ID: {tid} !!!!")

        # Wait for all bids to arrive
        self.get_logger().info(f"Waiting for bids for task {tid}...")
        self.wait_for_all_bids(tid)

        # wait till task is assigned
        self.get_logger().info(f"Waiting for task {tid} assignment...")
        self.task_assigned_event.wait()
        
        self.get_logger().info(f"Task {tid} assigned successfully")
        return new_lf

    def assign_tasks(self):
        # Publish new task, layer by layer
        lf = 0.0  # Initialize lf
        
        for tid in self.tf_list:    # tid --> task_id
            es = 10
            lf = self.publish_task(tid, es, lf)

        for tid in self.tl_list:    # tid --> task_id
            es = 10 + lf
            lf = self.publish_task(tid, es, lf)

        for tid in self.tl2_list:    # tid --> task_id
            es = 10 + lf
            lf = self.publish_task(tid, es, lf)

        for tid in self.tl3_list:    # tid --> task_id
            es = 10 + lf
            lf = self.publish_task(tid, es, lf)

        for tid in self.th_list:    # tid --> task_id
            es = 10 + lf
            lf = self.publish_task(tid, es, lf)

        # Check that all tasks have been assigned
        if not all([info[-1] for _,info in self.task_info.items()]):
            self.get_logger().warn("Not all tasks have been assigned!!!")
            self.get_logger().info(f"Current task info: {self.task_info}")
            self.get_logger().info("RESENDING TASKS!!!")
            # self.assign_tasks()
        else:
            tr_msg = TaskRemaining()
            tr_msg.all_tasks_allocated = True
            tr_msg.n_tasks = self.n_tasks
            self.tr_publisher.publish(tr_msg)   # publish that all tasks have been allocated

        

    def choose_best_bid_callback(self, request:TaskBid.Request, response:TaskBid.Response):
        self.get_logger().info(f"Received bid for T{request.task_id} from robot {request.bidder_id}")
        
        with self.bids_lock:
            # CRITICAL: Ignore bids for tasks that aren't the current task
            if request.task_id != self.current_task_id:
                self.get_logger().warn(
                    f"Ignoring late bid from robot {request.bidder_id} for task {request.task_id}. "
                    f"Current task is {self.current_task_id}"
                )
                response.all_received = False
                response.task_id = request.task_id
                return response
        
        # Wait for all bids if not yet received
        if not self.all_bids_received_event.is_set():
            self.get_logger().info("Waiting for remaining bids...")
            self.all_bids_received_event.wait(timeout=self.bid_timeout)
        
        with self.bids_lock:
            # Check if we have enough bids
            if not all(self.bids_received):
                missing = [i+1 for i, received in enumerate(self.bids_received) if not received]
                self.get_logger().warn(f"Not all bids received for task {request.task_id}. Missing: {missing}")
                response.all_received = False
                return response
            
            response.all_received = True

            # select bid with lowest es (earliest start) time
            if self.task_info[request.task_id][2] > 1:  # MR task
                winners = heapq.nsmallest(int(self.task_info[request.task_id][2]), self.bids.items(), key=lambda item: item[1][2])
                self.neighbours = [n for n,_ in winners]    # including the bidder id
                self.neighbours = list(map(int, self.neighbours))   # converting list of neighbours to integer
                self.publish_neighbours(request.task_id)   # publish neighbours to topic as well
                if request.bidder_id in self.neighbours:
                    response.neighbours = self.neighbours
                    response.neighbours.remove(request.bidder_id)
                    response.assigned = True
                    if request.task_id in self.p_constraints[:,0]:
                        ind = np.where(self.p_constraints[:,0] == request.task_id)[0][0]
                        # self.get_logger().info(f"{ind}")
                        # self.get_logger().info(f"{list(self.p_constraints[ind, :].astype(int))}")
                        # self.get_logger().info(f"{type(list(self.p_constraints[ind, :].astype(int)))}")
                        response.precedence_tasks = self.p_constraints[ind, 1:].astype(int).tolist()
                    # update task info Dict
                    self.task_info[request.task_id][-1] = True
                else:
                    response.assigned = False
            else: # SR task
                winner = min(self.bids, key=lambda k: self.bids[k][2])
                if request.bidder_id == winner:
                    response.assigned = True
                    # update task info Dict
                    self.task_info[request.task_id][-1] = True
                else:
                    response.assigned = False
                
            response.task_id = request.task_id  # append task_id
            
            # Count how many service responses we've sent
            self.service_responses_sent += 1
            
            # Only set task_assigned_event after ALL robots have received their response
            if self.service_responses_sent >= self.num_robots:
                self.get_logger().info(f"All service responses sent for task {request.task_id}")
                self.task_assigned_event.set()
                self.service_responses_sent = 0  # reset for next task
                self.bids_received = [False] * self.num_robots  # reset flag for next task
            
            return response
        
        
    def publish_neighbours(self, task_id):
        msg = Neighbours()
        msg.task_id = task_id
        msg.neighbours = self.neighbours
        self.neighbours_publisher.publish(msg)



def main(args=None):
    rclpy.init(args=args)

    node = AuctioneerCustomNode()

    # Spin the node in a background thread
    spin_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Small delay to ensure all connections established
    time.sleep(1.0)

    # Now it is SAFE to block
    node.get_logger().info("Starting AUCTIONS!!!")
    node.assign_tasks()

    # Continue after condition is met
    node.get_logger().info("Completed Auctions!!!")

    rclpy.shutdown()

if __name__ == '__main__':
    main()