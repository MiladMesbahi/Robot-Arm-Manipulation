import numpy as np

def dh_transform(a, d, alpha, theta):
    """
    Returns the transformation matrix using Denavit-Hartenberg parameters.
    """
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha),  np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta),  np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0,             np.sin(alpha),                 np.cos(alpha),                d],
        [0,             0,                             0,                            1]
    ])

def calcJacobian(q_in):
    """
    Calculate the full Jacobian of the end effector in a given configuration
    :param q_in: 1 x 7 configuration vector (of joint angles) [q1,q2,q3,q4,q5,q6,q7]
    :return: J - 6 x 7 matrix representing the Jacobian, where the first three
    rows correspond to the linear velocity and the last three rows correspond to
    the angular velocity, expressed in world frame coordinates
    """
    x_disp = [0, 0, 0.0825, -0.0825, 0, 0.088, 0]
    z_disp = [0.333, 0, 0.316, 0, 0.384, 0, 0.21]
    alpha = [-np.pi/2, np.pi/2, np.pi/2, -np.pi/2, np.pi/2, np.pi/2, 0]
    
    # Compute the forward kinematics and save each transformation matrix
    T = np.eye(4)
    T_matrices = []  
    
    for i in range(7):
        T_i = dh_transform(x_disp[i], z_disp[i], alpha[i], q_in[i])
        T = np.dot(T, T_i)  # Accumulate the transformation matrices
        T_matrices.append(T.copy())  # Append copy of the current transformation

    # position and orientation of the end-effector
    o_n = T[:3, 3]  

    J_v = np.zeros((3, 7))  # Linear part
    J_w = np.zeros((3, 7))  # Angular part

    z_prev = np.array([0, 0, 1])  # z-axis of the base frame
    o_prev = np.array([0, 0, 0])  # origin of the base frame

# compute the Jacobian components
    for i in range(7):
        T_i = T_matrices[i]  # Transformation matrix link i
        R_i = T_i[:3, :3]  # Rotation matrix link i
        o_i = T_i[:3, 3]  # Position link i

        z_i = R_i @ np.array([0, 0, 1])  

        # Linear velocity Jacobian cross product
        J_v[:, i] = np.cross(z_prev, (o_n - o_prev))

        # Angular velocity Jacobian 
        J_w[:, i] = z_prev

        z_prev = z_i
        o_prev = o_i

    # linear and angular Jacobians
    J = np.vstack((J_v, J_w))

    return J

if __name__ == '__main__':
    q= np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
    print(np.round(calcJacobian(q),3))
