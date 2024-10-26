import sympy as sp

# Define symbolic variables
q = sp.symbols('q1:8')  # Joint variables q1, q2, ..., q7
q_dot = sp.symbols('q_dot1:8')  # Joint velocities

# Define DH parameters
x_disp = [0, 0, 0.0825, -0.0825, 0, 0.088, 0]
z_disp = [0.333, 0, 0.316, 0, 0.384, 0, 0.21]
alpha = [-sp.pi/2, sp.pi/2, sp.pi/2, -sp.pi/2, sp.pi/2, sp.pi/2, 0]

# Define transformation matrix function
def dh_transform(a, d, alpha, theta):
    return sp.Matrix([
        [sp.cos(theta), -sp.sin(theta)*sp.cos(alpha), sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
        [sp.sin(theta), sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
        [0, sp.sin(alpha), sp.cos(alpha), d],
        [0, 0, 0, 1]
    ])

# Compute the forward kinematics
T = sp.eye(4)  # Identity matrix for base
T_matrices = []  # Store transformation matrices for each link

for i in range(7):
    T_i = dh_transform(x_disp[i], z_disp[i], alpha[i], q[i])
    T = T * T_i  # Multiply the transformation matrices
    T_matrices.append(T)

# Extract end-effector position and orientation
o_n = T[:3, 3]  # End-effector position
R_n = T[:3, :3]  # End-effector orientation

# Initialize Jacobian matrices
J_v = sp.zeros(3, 7)
J_w = sp.zeros(3, 7)

# Compute Jacobian columns
z_prev = sp.Matrix([0, 0, 1])  # Initial z-axis (base frame)
o_prev = sp.Matrix([0, 0, 0])  # Initial origin (base frame)

for i in range(7):
    T_i = T_matrices[i]  # Transformation matrix up to link i
    R_i = T_i[:3, :3]  # Rotation matrix of link i
    o_i = T_i[:3, 3]  # Position of link i

    z_i = R_i * sp.Matrix([0, 0, 1])  # Z-axis of the current joint

    # Linear velocity Jacobian component
    J_v[:, i] = z_prev.cross(o_n - o_prev)

    # Angular velocity Jacobian component (only for revolute joints)
    J_w[:, i] = z_prev

    # Update for next iteration
    z_prev = z_i
    o_prev = o_i

# Combine linear and angular Jacobians
J = sp.Matrix.vstack(J_v, J_w)

# Print the Jacobian in zero configuration
zero_config = {q[i]: 0 for i in range(7)}  # Set all joint variables to zero
J_zero_config = J.subs(zero_config)

# Print results
print("Jacobian at zero configuration:")
sp.pprint(J_zero_config)