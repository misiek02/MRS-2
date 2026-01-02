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
import numpy as np
from typing import Dict, List
from threading import Event, Thread
import heapq
from mrs_interfaces.action import Formation



class AuctioneerNode(Node):     # Auctioneer node for task, also doubles as Bidder node for robot 1
    def __init__(self):
        super().__init__('auctioneer_node')

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
        self.declare_parameter('TH')
        self.declare_parameter('p_1')
        self.declare_parameter('p_2')
        self.declare_parameter('p_3')
        self.declare_parameter('p_4')

        # Load parameters
        self.num_robots = self.get_parameter('num_robots').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.const_speed = self.get_parameter('const_speed').value
        self.n_tasks = self.get_parameter('n_tasks').value
        self.task_info: Dict[int, List[float]] = {}     # STRUCTURE: [pos_x, pos_y, num_robots_required, duration, assigned?]
        for i in range(self.n_tasks):
            self.task_info[i+1] = self.get_parameter(f"T{i+1}").value
            self.task_info[i+1].append(False)
        self.tf_list = self.get_parameter('TF').value
        self.tl_list = self.get_parameter('TL').value
        self.th_list = self.get_parameter('TH').value
        self.p_1 = self.get_parameter('p_1').value
        self.p_2 = self.get_parameter('p_2').value
        self.p_3 = self.get_parameter('p_3').value
        self.p_4 = self.get_parameter('p_4').value

        # ATTRIBUTES
        self.position = np.zeros((1, 2))
        self.position_received = False
        self.robot_id = 1   # hardcoded for all bidders
        self.neighbours = []
        self.task_assigned_event = Event()  # using 'event' to avoid blocking entire node
        self.bids: Dict[int, List[float]] = {}  # Dict of all received bids
        self.bids_received = [False] * self.num_robots
        self.p_constraints = np.array([self.p_1, self.p_2, self.p_3, self.p_4])

        # PUBLISHERS
        self.neighbours_publisher = self.create_publisher(Neighbours, "neighbours", 10)
        self.ta_publisher = self.create_publisher(TaskAnnouncement, "task_announcement", 10)
        self.tr_publisher = self.create_publisher(TaskRemaining, "task_remaining", 10)
        self.tp_publisher = self.create_publisher(TaskProgress, f"/cf_{self.robot_id}/task_progress", 10)

        # SUBSCRIBERS
        self.bids_subs = []
        self.pos_sub = self.create_subscription(PoseStamped, f'/cf_{self.robot_id}/pose', self.pose_callback, 10)
        for i in range(self.num_robots):
            robot_id = i+1  # Convert 0-indexed loop variable to 1-indexed robot ID (for topic naming)
            self.bids_subs.append(self.create_subscription(TaskBidMsg, f'/cf_{robot_id}/bid', lambda msg, rid=i: self.bid_callback(msg, rid), 10))

        # ACTION CLIENT
        self._action_client = ActionClient(self, Formation, "formation_action_node") 

        # SERVICES
        self.bid_srv = self.create_service(TaskBid, 'task_bid', self.choose_best_bid_callback)

        # TIMERS
        self.connection_timer = self.create_timer(5, self.connect_wait) # wait to allow publisher/subscriber connection

    
    # Startup timer callback function
    def connect_wait(self):
        self.destroy_timer(self.connection_timer) # stopping the 5-second wait
        # self.timer = self.create_timer(1.0 / self.publish_rate, self.control_loop)
        self.get_logger().info(f"Auctioneer/Bidder_{self.robot_id} node started...")

    def pose_callback(self, msg:PoseStamped):
        self.position[0][0] = msg.pose.position.x
        self.position[0][1] = msg.pose.position.y

        self.position_received = True

    def bid_callback(self, msg:TaskBidMsg, rid:int):
        self.bids[rid+1] = [msg.bidder_id, msg.task_id, msg.es]
        self.bids_received[rid] = True

    def assign_tasks(self):
        # Publish new task, layer by layer
        for tid in self.tf_list:    # tid --> task_id
            self.task_assigned_event.clear()
            ta_msg = TaskAnnouncement()
            ta_msg.task_id = tid
            ta_msg.task_location.x = self.task_info[tid][0]
            ta_msg.task_location.y = self.task_info[tid][1]
            ta_msg.num_required = self.task_info[tid][2]
            es = 10
            du = self.task_info[tid][3]
            lf = es + du + 20
            ta_msg.task_time = [du, es, lf]
            ta_msg.winner_selected = self.task_info[tid][4]
            if self.task_info[tid][2] > 1:
                ta_msg.task_type = "MR"
            else:
                ta_msg.task_type = "SR"

            # publish current task
            self.ta_publisher.publish(ta_msg)
            self.get_logger().info(f"NEW TASK PUBLISHED WITH ID: {tid} !!!!")

            # wait till task is assigned
            self.task_assigned_event.wait()


        for tid in self.tl_list:    # tid --> task_id
            self.task_assigned_event.clear()
            ta_msg = TaskAnnouncement()
            ta_msg.task_id = tid
            ta_msg.task_location.x = self.task_info[tid][0]
            ta_msg.task_location.y = self.task_info[tid][1]
            ta_msg.num_required = self.task_info[tid][2]
            es = 10 + lf
            du = self.task_info[tid][3]
            lf = es + du + 20
            ta_msg.task_time = [du, es, lf]
            ta_msg.winner_selected = self.task_info[tid][4]
            if self.task_info[tid][2] > 1:
                ta_msg.task_type = "MR"
            else:
                ta_msg.task_type = "SR"

            # publish current task
            self.ta_publisher.publish(ta_msg)
            self.get_logger().info(f"NEW TASK PUBLISHED WITH ID: {tid} !!!!")

            # wait till task is assigned
            self.task_assigned_event.wait()


        for tid in self.th_list:    # tid --> task_id
            self.task_assigned_event.clear()
            ta_msg = TaskAnnouncement()
            ta_msg.task_id = tid
            ta_msg.task_location.x = self.task_info[tid][0]
            ta_msg.task_location.y = self.task_info[tid][1]
            ta_msg.num_required = self.task_info[tid][2]
            es = 10 + lf
            du = self.task_info[tid][3]
            lf = es + du + 20
            ta_msg.task_time = [du, es, lf]
            ta_msg.winner_selected = self.task_info[tid][4]
            if self.task_info[tid][2] > 1:
                ta_msg.task_type = "MR"
            else:
                ta_msg.task_type = "SR"

            # publish current task
            self.ta_publisher.publish(ta_msg)
            self.get_logger().info(f"NEW TASK PUBLISHED WITH ID: {tid} !!!!")

            # wait till task is assigned
            self.task_assigned_event.wait()

        # Check that all tasks have been assigned
        if not all([info[-1] for _,info in self.task_info.items()]):
            self.get_logger().warn("Not all tasks have been assigned!!!")
        else:
            tr_msg = TaskRemaining()
            tr_msg.all_tasks_allocated = True
            self.tr_publisher.publish(tr_msg)   # publish that all tasks have been allocated

        

    def choose_best_bid_callback(self, request:TaskBid.Request, response:TaskBid.Response):
        self.get_logger().info(f"Received bid for T{request.task_id} from robot {request.bidder_id}")
        
        if all(self.bids_received):
            response.all_received = True
            self.bids_received = [False] * self.num_robots  # reset flag before next task is published

            # select bid with lowest es (earliest start) time
            if self.task_info[request.task_id][2] > 1:  # MR task
                winners = heapq.nsmallest(int(self.task_info[request.task_id][2]), self.bids.items(), key=lambda item: item[1][2])
                self.neighbours = [n for n,_ in winners]    # including the bidder id
                self.publish_neighbours(request.task_id)   # publish neighbours to topic as well
                if request.bidder_id in self.neighbours:
                    response.neighbours = self.neighbours.remove(request.bidder_id)
                    response.assigned = True
                    if request.task_id in self.p_constraints[:,0]:
                        ind = np.where(self.p_constraints[:,0])[0]
                        response.precedence_tasks = list(self.p_constraints[ind,:])
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
                
            self.task_assigned_event.set()
            return response

        else:
            response.all_received = False
            self.get_logger().warn("Not all bids have been received")
            self.task_assigned_event.set()
            return response
        
        
    def publish_neighbours(self, task_id):
        msg = Neighbours()
        msg.task_id = task_id
        msg.neighbours = self.neighbours
        self.neighbours_publisher.publish(msg)



def main(args=None):
    rclpy.init(args=args)

    node = AuctioneerNode()

    # Spin the node in a background thread
    spin_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Now it is SAFE to block
    node.get_logger().info("Starting AUCTIONS!!!")
    node.assign_tasks()

    # Continue after condition is met
    node.get_logger().info("Completed Auctions!!!")

    rclpy.shutdown()

if __name__ == '__main__':
    main()

