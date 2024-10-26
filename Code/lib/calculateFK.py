import numpy as np
from math import pi


class FK():
    def __init__(self):
        # DH parameters
        self.d1 = 0.333
        self.d3 = 0.316
        self.d5 = 0.384
        self.d7 = 0.21
        self.a3 = 0.0825
        self.a4 = -0.0825
        self.a6 = 0.088
        self.alpha = [-np.pi/2, np.pi/2, np.pi/2, -
                      np.pi/2, np.pi/2, np.pi/2, 0]
        self.a = [0, 0, self.a3, self.a4, 0, self.a6, 0]
        self.d = [self.d1, 0, self.d3, 0, self.d5, 0, self.d7]

        # joint offsets
        self.joint_offsets = [
            np.array([0, 0, 0.141, 1]),    # 0 -
            np.array([0, 0, 0, 1]),        # 1
            np.array([0, 0, 0.195, 1]),    # 2
            np.array([0, 0, 0, 1]),        # 3
            np.array([0, 0, 0.125, 1]),    # 4
            np.array([0, 0, -0.015, 1]),   # 5
            np.array([0, 0, 0.051, 1]),    # 6
            np.array([0, 0, 0, 1])         # 7
        ]

    def dh_transform(self, a, alpha, d, theta):
        """
        DH table
        """
        return np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha),
             np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(alpha), -
             np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])

    def forward(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        jointPositions - 8x3 matrix, where each row corresponds to a rotational joint of the robot or end effector.
                         Each row contains the [x,y,z] coordinates in the world frame of the respective joint's center.
        T0e - a 4x4 homogeneous transformation matrix representing the end effector frame in the world frame.
        """
        jointPositions = np.zeros((8, 3))
        jointPositions[0, :] = [0, 0, 0.141]

        T_prev = np.identity(4)
        for i in range(7):
            theta = q[i]
            if i == 6:
                theta -= pi / 4

            Ti = self.dh_transform(self.a[i], self.alpha[i], self.d[i], theta)
            T_prev = T_prev @ Ti

            joint_pos_homogeneous = T_prev @ self.joint_offsets[i + 1]
            jointPositions[i + 1, :] = joint_pos_homogeneous[:3]

        T0e = T_prev

        jointPositions = np.round(jointPositions, decimals=6)
        return jointPositions, T0e

    def get_axis_of_rotation(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        axis_of_rotation_list: - 3x7 np array of unit vectors describing the axis of rotation for each joint in the
                                 world frame

        """
      
    
    def compute_Ai(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        Ai: - 4x4 list of np array of homogenous transformations describing the FK of the robot. Transformations are not
              necessarily located at the joint locations
        """
        # STUDENT CODE HERE: This is a function needed by lab 2
        Ai = []
        T_prev = np.identity(4)
        for i in range(7):
            theta = q[i]
            # transformation matrix for joint i
            Ti = self.dh_transform(self.a[i], self.alpha[i], self.d[i], theta)
            T_prev = T_prev @ Ti
        
            # cumulative transformation matrix
            Ai.append(T_prev)

        return ()


"""""
if __name__ == "__main__":

    fk = FK()

    # matches figure in the handout
    q = np.array([0, 0, 0, -pi/2, 0, pi, pi/4])

    joint_positions, T0e = fk.forward(q)

    print("Joint Positions:\n", joint_positions)
    print("End Effector Pose:\n", T0e)
"""
