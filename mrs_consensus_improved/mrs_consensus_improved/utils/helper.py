# ===================================
# HELPER FUNCTIONS PACKAGE
# ===================================
import numpy as np

def get_adjency_matrix_from_topology(topology:str) -> np.ndarray:
    """
        Function to get the adjancy matrix, based on the selected topology (check the params file for details)

        Parameters:
            topology: (str) selected rendezvous configuration. This is fitted for 4 robots (A, B, C, D, E, F)

        Returns:
            A: (np.ndarray) adjancy matrix representing the topology
    """
    match topology:
        case 'A':
            A = np.array([
                [0, 1, 0, 1],
                [1, 0, 1, 0],
                [0, 1, 0, 1],
                [1, 0, 1, 0]
            ])
            return A
        case 'B':
            A = np.array([
                [0, 0, 0, 1],
                [0, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0]
            ])
            return A
        case 'C':
            A = np.array([
                [0, 1, 0, 0],
                [1, 0, 0, 0],
                [0, 0, 0, 1],
                [0, 0, 1, 0]
            ])
            return A
        case 'D':
            A = np.array([
                [0, 1, 0, 1],
                [0, 0, 0, 0],
                [0, 1, 0, 1],
                [0, 0, 0, 0]
            ])
            return A
        case 'E':
            A = np.array([
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
                [1, 0, 0, 0]
            ])
            return A
        case 'F':
            A = np.array([
                [0, 0, 1, 0],
                [1, 0, 0, 0],
                [0, 1, 0, 1],
                [0, 0, 0, 0]
            ])
            return A
        case _: 
            print("Topology must be A, B, C, D, E, or F. Check params file for details!")
            print("Returning a fully connected configuration")
            return np.zeros((4,4))


def compute_separation_acc(dist_x, dist_y, safe_dist, k_sep):
    """
        Function to compute the separation acceleration between robot i and neighbour j

        Parameters:
            dist_x: (float) distance in x between robot i and its neighbour j
            dist_y: (float) distance in y between robot i and its neighbour j

        Returns:
            sep_x, sep_y: (tuple) value of separation acceleration repelling robot i from its neighbour j
    """

    sep_x, sep_y = 0.0, 0.0     # initialising separation acc. value

    dist_norm = np.linalg.norm([dist_x, dist_y])

    # Check if neighboring robot is within collision danger zone
    # dist > 0.01 prevents numerical issues from division by very small numbers
    if dist_norm < safe_dist and dist_norm > 0.01:
        # Compute repulsive force magnitude using smooth function
        # Formula: F = k_sep * (1/d - 1/safe_dist)
        force = k_sep * (1.0/dist_norm - 1.0/safe_dist)

        # Apply force in direction away from the neighboring robot
        # (dx/dist) is unit vector pointing from j to i
        sep_x += (dist_x/dist_norm) * force
        sep_y += (dist_y/dist_norm) * force

    return sep_x, sep_y

def cap_acceleration(ax, ay, max_acc=2.0):
    acc_norm = np.linalg.norm([ax, ay])

    if acc_norm > max_acc:
        scale = max_acc / acc_norm
        capped_x = ax * scale
        capped_y = ay * scale
        return capped_x, capped_y
    else:
        return ax, ay
    
def get_formation_offset_matrix(desired_shape:str, num_robots:int, vleader_pos:list = [0.0, 0.0], spacing=0.7):
    """
        Function to create the offset matrix for formation control.
        Note that this function assumes: the number of robots in the formation is EVEN.

        Parameters:
            desired_shape: (str) string indicating the desired formation shape. Check params file for details
            num_robots: (int) number of robots partaking in formation
            vleader_pos: (list) 2D list of virtual leader initial position. Default is [0.0, 0.0]
            spacing: (float) minimum distance between robots in formation 

        Returns:
            offsets: (np.ndarray) Nx2 array of robot offsets representing their desired formation (wrt to the virtual leader position)
    """
    assert num_robots % 2 == 0  # check that the number of robots is even
    offsets = [np.zeros((2,))] * num_robots

    match desired_shape:
        case 'Lh':
            # NOTE: Horizontal formation is arranged on y axis
            half_num_robots = num_robots // 2
            
            # Assign positions for first half of the line
            for i in range(int(num_robots/2)):
                offsets[i] = np.array([vleader_pos[0], vleader_pos[1]+((half_num_robots-1)*spacing)])
                half_num_robots -= 1
            # Assign positions for second half of the line
            half_num_robots = num_robots // 2
            for i in range(int(num_robots/2), num_robots):
                offsets[i] = np.array([vleader_pos[0], vleader_pos[1]-(half_num_robots*spacing)])
                half_num_robots -= 1

            return np.array(offsets)
        
        case 'Lv':
            # NOTE: Vertical formation is arranged on x axis
            half_num_robots = num_robots // 2
            
            # Assign positions for first half of the line
            for i in range(int(num_robots/2)):
                offsets[i] = np.array([vleader_pos[0]+((half_num_robots-1)*spacing), vleader_pos[1]])
                half_num_robots -= 1
            # Assign positions for second half of the line
            half_num_robots = num_robots // 2
            for i in range(int(num_robots/2), num_robots):
                offsets[i] = np.array([vleader_pos[0]-(half_num_robots*spacing), vleader_pos[1]])
                half_num_robots -= 1

            return np.array(offsets)
        
        case 'S':   # robots are arranged from top left corner of square, in clockwise order
            edge_length = int((num_robots / 4) * spacing)    # square is four-sided
            half_edge_length = int(edge_length / 2)    
            side_num_robots = int(num_robots / 4)  # number of robots per side
            
            # --------------------------
            # Top-side
            ts_vleader_pos = [vleader_pos[0]+half_edge_length, vleader_pos[1]]
            half_side_num_robots = side_num_robots // 2
            
            # Assign positions for first half of the line
            for i in range(int(side_num_robots/2)):
                offsets[i] = np.array([ts_vleader_pos[0], ts_vleader_pos[1]+(half_side_num_robots*spacing)])
                half_side_num_robots -= 1
            # Assign positions for second half of the line
            half_side_num_robots = side_num_robots // 2
            for i in range(int(side_num_robots/2), side_num_robots):
                offsets[i] = np.array([ts_vleader_pos[0], ts_vleader_pos[1]-(half_side_num_robots*spacing)])
                half_side_num_robots -= 1

            # --------------------------
            # Right-side
            rs_vleader_pos = [vleader_pos[0], vleader_pos[1] - half_edge_length]
            half_side_num_robots = side_num_robots // 2

            # Assign positions for first half of the line
            for i in range(side_num_robots, (side_num_robots + int(side_num_robots/2))):
                offsets[i] = np.array([rs_vleader_pos[0]+(half_side_num_robots*spacing), rs_vleader_pos[1]])
                half_side_num_robots -= 1
            # Assign positions for second half of the line
            half_side_num_robots = side_num_robots // 2
            for i in range((side_num_robots + int(side_num_robots/2)), side_num_robots*2):
                offsets[i] = np.array([rs_vleader_pos[0]-(half_side_num_robots*spacing), rs_vleader_pos[1]])
                half_side_num_robots -= 1

            # --------------------------
            # Bottom-side
            bs_vleader_pos = [vleader_pos[0]-half_edge_length, vleader_pos[1]]
            half_side_num_robots = side_num_robots // 2

            # Assign positions for second half of the line
            for i in range(side_num_robots*2, side_num_robots*2 + int(side_num_robots/2)):
                offsets[i] = np.array([bs_vleader_pos[0], bs_vleader_pos[1]-(half_side_num_robots*spacing)])
                half_side_num_robots -= 1
            # Assign positions for first half of the line
            half_side_num_robots = side_num_robots // 2
            for i in range(side_num_robots*2 + int(side_num_robots/2), side_num_robots*3):
                offsets[i] = np.array([bs_vleader_pos[0], bs_vleader_pos[1]+(half_side_num_robots*spacing)])
                half_side_num_robots -= 1
            

            # --------------------------
            # Left-side
            ls_vleader_pos = [vleader_pos[0], vleader_pos[1] + half_edge_length]
            half_side_num_robots = side_num_robots // 2

            # Assign positions for second half of the line
            for i in range(side_num_robots*3, side_num_robots*3 + int(side_num_robots/2)):
                offsets[i] = np.array([ls_vleader_pos[0]-(half_side_num_robots*spacing), ls_vleader_pos[1]])
                half_side_num_robots -= 1
            # Assign positions for first half of the line
            half_side_num_robots = side_num_robots // 2
            for i in range(side_num_robots*3 + int(side_num_robots/2), num_robots):
                offsets[i] = np.array([ls_vleader_pos[0]+(half_side_num_robots*spacing), ls_vleader_pos[1]])
                half_side_num_robots -= 1

            # Return the offset positions
            return np.array(offsets)
        
        case 'T':   # valid only for 3, 4 and 6 number of robots (clockwise arrangement starting from top triangle vertex)
            if num_robots == 3:
                offsets[0] = np.array([vleader_pos[0]+spacing, vleader_pos[1]])
                offsets[1] = np.array([vleader_pos[0]-spacing, vleader_pos[1]-spacing])
                offsets[2] = np.array([vleader_pos[0]-spacing, vleader_pos[1]+spacing])
            elif num_robots == 4:
                offsets[0] = np.array([vleader_pos[0]+spacing, vleader_pos[1]])
                offsets[1] = np.array([vleader_pos[0]-spacing, vleader_pos[1]-spacing])
                offsets[2] = np.array([vleader_pos[0]-spacing, vleader_pos[1]])
                offsets[3] = np.array([vleader_pos[0]-spacing, vleader_pos[1]+spacing])
            elif num_robots == 6: # equilateral triangle
                L = spacing * 2     # length of one side
                h = (np.sqrt(3)*L)/2    # triangle height
                offsets[0] = np.array([vleader_pos[0]+(h/2), vleader_pos[1]])
                x_spacing = spacing * np.cos(np.pi/6)
                y_spacing = spacing * np.sin(np.pi/6)
                offsets[1] = np.array([vleader_pos[0]+(h/2)-x_spacing, vleader_pos[1]-y_spacing])
                offsets[2] = np.array([vleader_pos[0]-(h/2), vleader_pos[1]-(L/2)])
                offsets[3] = np.array([vleader_pos[0]-(h/2), vleader_pos[1]])
                offsets[4] = np.array([vleader_pos[0]-(h/2), vleader_pos[1]+(L/2)])
                offsets[5] = np.array([vleader_pos[0]+(h/2)-x_spacing, vleader_pos[1]+y_spacing])
            else:
                return 
            
            # Return the offset positions
            return np.array(offsets)

        case _:
            return None # invalid string provided for desired_shape
