import numpy as np 
from lib.calcJacobian import calcJacobian



def IK_velocity(q_in, v_in, omega_in):
    """
    :param q_in: 1 x 7 vector corresponding to the robot's current configuration.
    :param v_in: The desired linear velocity in the world frame. If any element is
    Nan, then that velocity can be anything
    :param omega_in: The desired angular velocity in the world frame. If any
    element is Nan, then that velocity is unconstrained i.e. it can be anything
    :return:
    dq - 1 x 7 vector corresponding to the joint velocities. If v_in and omega_in
         are infeasible, then dq should minimize the least squares error. If v_in
         and omega_in have multiple solutions, then you should select the solution
         that minimizes the l2 norm of dq
    """

    ## STUDENT CODE GOES HERE
    J = calcJacobian(q_in)

    velocity_target =  np.vstack((v_in.reshape((3, 1)), omega_in.reshape((3, 1))))
    for i in range(6):
        if np.isnan(velocity_target[i]):
          velocity_target[i] = 0  # NaN values to 0 
          J[i, :] = 0  # Unconstrained rows ignored

    # Least squares solution for joint velocities dq
    dq, residuals, rank, s = np.linalg.lstsq(J, velocity_target, rcond=None)

    return dq.flatten()