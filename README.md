# Robot-Arm-Manipulation
 Robotics Kinematics and Control Labs for Franka Emika Arm

**Lab 1: Forward Kinematics for Robot Arm**
 
_Overview:_
This lab focuses on implementing forward kinematics for the Panda robot arm to compute the position and orientation of each joint relative to the world frame. Through coding, simulation, and hardware testing, we aim to deepen our spatial understanding of robotic kinematics and validate theoretical results.

_Key Components:_
Forward Kinematics Calculation: Implemented a forward kinematics function to determine the position of the end-effector and all intermediate joint positions based on given joint angles.
Coordinate Frame Assignment: Defined coordinate frames for each link and computed the transformation between consecutive frames.
Simulation Testing: Visualized results using Gazebo and RViz for initial verification, with additional custom configurations to test edge cases and expected results.
Hardware Validation: Compared calculated joint and end-effector positions to real-world positions on the physical Panda arm, analyzing any deviations between simulation and hardware performance.

**Lab 2: Jacobians and Velocity Kinematics**

_Overview:_
In this lab, we extend our kinematic analysis to include Jacobians and velocity kinematics for the Panda robot arm, allowing us to compute both forward and inverse velocity kinematics. The implementation is tested in both simulated and hardware environments.

_Key Components:_
Jacobian Calculation: Implemented the Jacobian matrix in calcJacobian.py to relate joint velocities to end-effector velocities.
Forward and Inverse Velocity Kinematics: Calculated end-effector linear and angular velocities from joint velocities and implemented least-squares solutions for cases with no or infinite inverse velocity solutions.
Angular Velocity Tracking: Developed a function to track angular velocity based on orientation differences using rotation matrices and extracting skew-symmetric parts.
Trajectory Tracking: Implemented periodic trajectory tracking (ellipse, line, and figure-eight) for the end-effector in follow.py to test velocity control accuracy.
