# ============================================================================
# ROS2 Consensus-based Rendezvous Controller for Multi-Robot Systems
# ============================================================================
# This script implements a distributed control algorithm where multiple robots
# converge to a common meeting point (rendezvous) using consensus protocol.
# The robots communicate in a ring topology and avoid collisions while converging.
#
# Key Features:
# - Distributed consensus algorithm (no central controller required)
# - Collision avoidance through repulsive forces
# - Ring topology communication pattern (scalable, robust)
# - Velocity saturation for safety
# - 20 Hz control loop for smooth performance
# ============================================================================

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path  # ADDED: RViz trajectory visualization message
import numpy as np
import math

class ConsensusRendezvous(Node):
    def __init__(self):
        """Initialize the Consensus Rendezvous node with control parameters and ROS subscriptions."""
        super().__init__('consensus_rendezvous')
        
        # Total number of robots in the swarm
        self.num_robots = 4
        
        # Control parameters for the consensus algorithm
        self.k_p = 1.0           # Consensus gain (attraction): controls convergence speed
        self.k_sep = 1.5         # Separation gain (repulsion): prevents collisions
        self.safe_dist = 0.5     # Minimum safe distance between robots (meters)
        
        # Position storage array: stores (x, y) coordinates for each robot
        self.positions = np.zeros((self.num_robots, 2))
        # Track which robots have sent at least one position update
        self.position_received = [False] * self.num_robots

        # ============================================================================
        # Communication Topology Matrix (Adjacency Matrix A)
        # ============================================================================
        # Defines which robots communicate with which robots
        # Entry A[i,j] = 1 means robot i receives information from robot j
        # 
        # Current Topology: Ring (Cyclic) - ensures circular connectivity
        # Communication pattern:
        #   Robot 0 receives from Robot 1  (R0 <- R1)
        #   Robot 1 receives from Robot 2  (R1 <- R2)
        #   Robot 2 receives from Robot 3  (R2 <- R3)
        #   Robot 3 receives from Robot 0  (R3 <- R0)
        # 
        # Advantages of ring topology:
        # - Minimal communication overhead (each robot communicates with 1 neighbor)
        # - Scalable to larger swarms
        # - Robust to node failures (graceful degradation)
        # ============================================================================
        self.A = np.array([
            [0, 1, 1, 1], 
            [1, 0, 1, 1], 
            [1, 1, 0, 1], 
            [1, 1, 1, 0]  
        ])

        # Initialize lists to store ROS publishers and subscribers for all robots
        self.pubs = []
        self.subs = []

        # ============================================================================
        # ADDED: RViz trajectory visualization using nav_msgs/Path
        # ============================================================================
        # We create one Path publisher per robot:
        #   /cf_<robot_id>/path
        #
        # Each time we receive a /cf_<robot_id>/pose message, we append that pose to
        # the Path and publish it. RViz2 can draw the Path as a line showing the
        # robot trajectory over time.
        #
        # RViz setup:
        #   - Set "Fixed Frame" to "world"
        #   - Add -> Path
        #   - Choose topic /cf_1/path, /cf_2/path, ...
        # ============================================================================
        self.path_pubs = []
        self.paths = [Path() for _ in range(self.num_robots)]
        self.fixed_frame = "world"         # Must match the pose frame used by the simulator
        self.max_path_points = 2000        # Limit to avoid RViz slowdown/memory growth

        # Create ROS2 communication channels for each robot in the swarm
        for i in range(self.num_robots):
            # Convert 0-indexed loop variable to 1-indexed robot ID (for topic naming)
            robot_id = i + 1
            
            # ================================================================
            # SUBSCRIPTION: Receive robot pose (position and orientation)
            # ================================================================
            # Subscribe to robot pose topic from simulator or Loco Positioning System
            # Topic name: /cf_<robot_id>/pose
            # Message type: PoseStamped (contains position and orientation timestamps)
            # Callback: pose_callback - updates internal position array when data arrives
            # Queue size: 10 - buffer up to 10 messages before dropping oldest ones
            self.subs.append(self.create_subscription(
                PoseStamped,  # Message type for pose data
                f'/cf_{robot_id}/pose',  # Topic to subscribe to
                lambda msg, rid=i: self.pose_callback(msg, rid),  # Callback handler
                10  # Queue size for buffering
            ))
            
            # ================================================================
            # PUBLISHER: Send velocity commands to control robot motion
            # ================================================================
            # Publish velocity commands to control horizontal motion
            # Topic name: /cf_<robot_id>/cmd_vel
            # Message type: Twist (contains linear and angular velocity vectors)
            # Queue size: 10 - buffer messages to handle temporary delays
            self.pubs.append(self.create_publisher(
                Twist,  # Message type for velocity commands
                f'/cf_{robot_id}/cmd_vel',  # Topic to publish to
                10  # Queue size for buffering
            ))

            # ================================================================
            # ADDED: PUBLISHER: Publish robot trajectory for RViz
            # ================================================================
            self.path_pubs.append(self.create_publisher(
                Path,
                f'/cf_{robot_id}/path',
                10
            ))
            self.paths[i].header.frame_id = self.fixed_frame
        
        # ====================================================================
        # CONTROL LOOP TIMER
        # ====================================================================
        # Create a timer that triggers the control loop at regular intervals
        # Frequency: 20 Hz (0.05 second period)
        # Higher frequency = smoother control and better consensus convergence
        # Lower frequency = reduced computational load (trade-off)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info("Consensus Rendezvous Node Started (Ring Topology, 4 Robots)")

    def pose_callback(self, msg, robot_idx):
        """Callback function triggered when a robot publishes its pose.
        
        This function is called asynchronously whenever a PoseStamped message
        arrives on the subscribed /cf_<id>/pose topic. It updates the internal
        position record for the corresponding robot.
        
        Args:
            msg (PoseStamped): Pose message containing position (x,y,z) and 
                              orientation (quaternion) with timestamp
            robot_idx (int): Index of the robot (0 to num_robots-1)
        """
        # Extract and store the x and y coordinates from the pose message
        # Note: We ignore z-coordinate (altitude) as it's controlled separately
        self.positions[robot_idx] = [msg.pose.position.x, msg.pose.position.y]
        
        # Mark this robot as having sent at least one position update
        # This flag prevents control loop from starting until all robots report
        self.position_received[robot_idx] = True

        # ============================================================================
        # ADDED: Update and publish the Path for RViz (trajectory visualization)
        # ============================================================================
        # We append the current pose to the Path. RViz will draw the trajectory
        # when you display the /cf_<id>/path topic as a Path display.
        p = PoseStamped()
        p.header.stamp = msg.header.stamp
        p.header.frame_id = self.fixed_frame
        p.pose = msg.pose

        self.paths[robot_idx].header.stamp = msg.header.stamp
        self.paths[robot_idx].poses.append(p)

        # Keep only the last max_path_points poses to avoid unlimited growth
        if len(self.paths[robot_idx].poses) > self.max_path_points:
            self.paths[robot_idx].poses.pop(0)

        self.path_pubs[robot_idx].publish(self.paths[robot_idx])

    def control_loop(self):
        """Main control loop: computes and sends velocity commands to all robots.
        
        This function implements the consensus algorithm combined with collision avoidance.
        It runs at 20 Hz and executes the following steps for each robot:
        
        1. CONSENSUS: Attract each robot toward neighbors (based on ring topology)
        2. SEPARATION: Repel robots that get too close (collision avoidance)
        3. VELOCITY LIMITING: Cap maximum velocity for safety and stability
        4. COMMAND PUBLISHING: Send computed velocities to robots
        
        The result is a distributed algorithm where robots coordinate without
        a central controller, converging to a common meeting point while maintaining
        safe distances from each other.
        """
        # Safety check: Wait until position updates have been received from all robots
        # before starting the control algorithm. This ensures all state is initialized.
        if not all(self.position_received):
            return

        # Process each robot independently in the swarm
        for i in range(self.num_robots):
            # Initialize velocity components (x and y directions) for this robot
            u_x, u_y = 0.0, 0.0
            
            # ================================================================
            # PHASE 1: CONSENSUS ALGORITHM (CONVERGENCE TO RENDEZVOUS POINT)
            # ================================================================
            # Compute attraction forces based on distributed consensus protocol
            # 
            # Consensus Formula:
            #   u_i = sum over j in neighbors of A[i,j] * (x_j - x_i)
            # 
            # Where:
            #   A[i,j] = 1 if robot i receives information from robot j (ring topology)
            #   x_j = position of robot j
            #   x_i = position of robot i
            #   (x_j - x_i) = vector pointing from robot i toward robot j
            # 
            # Behavior:
            #   - Each robot is attracted to the positions of its neighbors
            #   - All robots converge to the centroid (average position)
            #   - For rendezvous task, this creates a common meeting point
            # ================================================================
            
            for j in range(self.num_robots):
                # Only consider robots in communication range (defined by adjacency matrix A)
                # A[i,j] == 1 means robot i should track information from robot j
                if self.A[i][j] == 1:
                    # Calculate position difference vector (neighbor - self)
                    # This vector points from robot i toward robot j
                    u_x += (self.positions[j][0] - self.positions[i][0])
                    u_y += (self.positions[j][1] - self.positions[i][1])
            
            # Scale the consensus forces by the control gain (k_p)
            # Higher k_p → faster convergence but may cause oscillations
            # Lower k_p → slower convergence but more stable motion
            u_x *= self.k_p
            u_y *= self.k_p

            # ================================================================
            # PHASE 2: COLLISION AVOIDANCE (SEPARATION FORCES)
            # ================================================================
            # Compute repulsive forces from robots that are too close
            # 
            # This layer prevents collisions while maintaining consensus convergence
            # by adding repulsive forces when robots enter danger zone
            # 
            # Separation Method:
            #   - Monitor distance to all other robots
            #   - Apply repulsive force if distance < safe_dist
            #   - Force increases as robots get closer (inverse relationship)
            #   - Direction is always away from neighboring robot
            # ================================================================
            col_x, col_y = 0.0, 0.0
            
            for j in range(self.num_robots):
                if i == j:
                    # Skip self-repulsion (a robot doesn't repel from itself)
                    continue
                
                # Calculate distance and direction to neighboring robot
                dx = self.positions[i][0] - self.positions[j][0]
                dy = self.positions[i][1] - self.positions[j][1]
                dist = math.sqrt(dx**2 + dy**2)
                
                # Check if neighboring robot is within collision danger zone
                # dist > 0.01 prevents numerical issues from division by very small numbers
                if dist < self.safe_dist and dist > 0.01:
                    # Compute repulsive force magnitude using smooth function
                    # Formula: F = k_sep * (1/d - 1/safe_dist)
                    # 
                    # Characteristics:
                    #   - F = 0 when d >= safe_dist (no repulsion outside danger zone)
                    #   - F increases as d decreases (stronger repulsion when closer)
                    #   - Function is smooth and differentiable (no hard boundaries)
                    # ====================================================
                    force = self.k_sep * (1.0/dist - 1.0/self.safe_dist)
                    
                    # Apply force in direction away from the neighboring robot
                    # (dx/dist) is unit vector pointing from j to i
                    col_x += (dx/dist) * force
                    col_y += (dy/dist) * force
            
            # ================================================================
            # PHASE 3: COMBINE CONTROL SIGNALS
            # ================================================================
            # Merge consensus attraction and collision avoidance repulsion forces
            # The combined signal drives robots toward meeting point while avoiding collisions
            final_vx = u_x + col_x
            final_vy = u_y + col_y

            # ================================================================
            # PHASE 4: VELOCITY SATURATION (SAFETY LIMITER)
            # ================================================================
            # Limit maximum velocity to prevent excessive acceleration and ensure stability
            # 
            # Importance:
            #   - Prevents jerky movements and mechanical stress
            #   - Ensures real robots can follow the commands physically
            #   - Improves numerical stability of control algorithm
            #   - Provides safety margin for emergency maneuvers
            # ================================================================
            max_vel = 0.5  # Maximum velocity in meters per second
            
            # Calculate magnitude of velocity vector (Euclidean norm)
            v_norm = math.sqrt(final_vx**2 + final_vy**2)
            
            # If velocity exceeds limit, scale it down while preserving direction
            # This is more stable than clamping individual components
            if v_norm > max_vel:
                # Calculate scale factor to limit velocity
                scale = max_vel / v_norm
                # Apply scale to both components (preserves direction, limits magnitude)
                final_vx *= scale
                final_vy *= scale

            # ================================================================
            # PHASE 5: PUBLISH VELOCITY COMMAND TO ROBOT
            # ================================================================
            # Create ROS2 Twist message with computed velocity commands
            msg = Twist()
            msg.linear.x = float(final_vx)   # Velocity in x direction (forward/backward)
            msg.linear.y = float(final_vy)   # Velocity in y direction (left/right)
            
            # Note on z-axis (vertical motion):
            # The z-axis (altitude) velocity is intentionally omitted here because:
            #   1. In simulation (CrazySim) and real Crazyflie, altitude is controlled separately
            #   2. A velocity multiplexer (vel_mux) maintains altitude automatically after takeoff
            #   3. We focus on horizontal (xy) rendezvous motion only
            #   4. Vertical control is handled by a dedicated height controller
            
            # Publish velocity command to the robot's cmd_vel topic
            # The flight controller will receive this and adjust motor speeds accordingly
            self.pubs[i].publish(msg)

def main(args=None):
    """Main entry point for the ROS2 consensus rendezvous node.
    
    This function:
    1. Initializes the ROS2 client library
    2. Creates an instance of the ConsensusRendezvous controller node
    3. Starts the main event loop (spins the node)
    4. Handles graceful shutdown when interrupted (Ctrl+C)
    5. Cleans up resources before exiting
    
    Args:
        args: Optional command-line arguments passed to rclpy.init()
    """
    # Initialize ROS2 client library
    # This sets up the underlying middleware and communication layer
    rclpy.init(args=args)
    
    # Create an instance of the consensus rendezvous controller node
    # This initializes all subscribers, publishers, and the control timer
    node = ConsensusRendezvous()
    
    try:
        # Spin the node: enters the main event loop
        # This keeps the node running and processing callbacks (subscriptions, timers)
        # The control_loop() function will be called every 0.05 seconds
        # The pose_callback() will be called whenever pose messages arrive
        # Execution continues here until the node is shut down
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Gracefully handle Ctrl+C interruption
        # This exception is raised when user presses Ctrl+C in the terminal
        pass
    finally:
        # Clean up: destroy the node and shutdown ROS2
        # This releases all ROS2 resources and cleanly disconnects from middleware
        node.destroy_node()
        rclpy.shutdown()

# ============================================================================
# Script Entry Point
# ============================================================================
# This ensures main() is only called when the script is run directly,
# not when it's imported as a module in another script
if __name__ == '__main__':
    main()

