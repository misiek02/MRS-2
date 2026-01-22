import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from mrs_interfaces.msg import TaskProgress     # published by all bidders
from mrs_interfaces.srv import TaskBid
from mrs_interfaces.msg import TaskBidMsg       # published by all bidders
from mrs_interfaces.action import Formation
import numpy as np
from typing import Dict, List, Tuple
import time


class TestNode(Node):        
    def __init__(self):
        super().__init__('testnode')

        # ACTION CLIENT
        self._action_client = ActionClient(self, Formation, "formation_action_node") 
        
        # TIMERS
        self.connection_timer = self.create_timer(5, self.connect_wait)

    
    # Startup timer callback function
    def connect_wait(self):
        self.destroy_timer(self.connection_timer) # stopping the 3-second wait
        self.get_logger().info(f"Tester node started...")
        self.get_logger().info(f"Sending formation request...")
        
        self.send_goal(1, "T", 0.8, 0.0, 0.0, [1,2,3, 4])

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
            f'(Feedback) Robots in formation: {len(feedback.robot_ids)}',
            throttle_duration_sec=2.0)  # Log every 2 seconds

        
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
            self.prev_task_complete = True
            self.get_logger().info('Formation completed successfully!')
            

        else:                
            self.task_requested = True
            self.get_logger().info('Formation did not complete')


def main(args=None):
    rclpy.init(args=args)
    testnode = TestNode()
    try:
        rclpy.spin(testnode)
    except KeyboardInterrupt:
        pass
    finally:
        testnode.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
