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