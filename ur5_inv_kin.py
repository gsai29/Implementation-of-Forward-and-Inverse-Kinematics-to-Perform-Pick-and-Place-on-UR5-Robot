import numpy as np
import math as m
import time

def ur5_params():
    """
    Returns
    -------
    home_position : Home position of the UR5 with DH parameters
    screw : Screw matrix for UR5 with DH parameters
    """
    # UR5 link parameters (in meters)
    l1 = 0.425
    l2 = 0.392
    h1 = 0.160
    h2 = 0.09475
    w1 = 0.134
    w2 = 0.0815
    # home_orientation=RxyztoRM(3.176,-0.339,0.037)

    # home_vec=np.array([-0.4175, -0.498, 0.52])

    # home_position = np.array([
    #     [-1, 0, 0, l1 + l2],
    #     [0, 0, 1, w1 + w2],
    #     [0, 1, 0, h1 - h2],
    #     [0, 0, 0, 1]
    # ])
    # home_position = np.transpose(np.reshape(np.append(np.transpose(home_orientation), home_vec), (-1, 3)))
    # home_position=np.vstack((home_position, [0, 0, 0, 1]))
    home_position = np.array([
    [-0.0309, - 0.9993 ,   0.0208, - 0.4175],
    [- 0.9987,    0.0317,    0.0409, - 0.4980],
    [- 0.0415, - 0.0195, - 0.9989,    0.5208],
    [0   ,      0    ,     0  ,  1.0000]
    ])
    # screw = np.array([
    #     [0, 0, 1, 0, 0, 0],
    #     [0, 1, 0, -h1, 0, 0],
    #     [0, 1, 0, -h1, 0, l1],
    #     [0, 1, 0, -h1, 0, l1+l2],
    #     [0, 0, -1, -w1, l1+l2, 0],
    #     [0, 1, 0, h2-h1, 0, l1+l2]
    # ])
    screw = np.array([
        [0, 0, 1, 0, 0, 0],
        [0.621, -0.7838, 0, 0.1274, 0.1009, 0],
        [0.621, -0.7838, 0, 0.4377, 0.3468, 0.1544],
        [0.621, -0.7838, 0, 0.4895, 0.3878, 0.541],
        [-0.7831, -0.6204, -0.0417, 0.4058, -0.5033, -0.1329],
        [0.0208, 0.0409, -0.9989, 0.4762, -0.4062, -0.0067]
    ])
    return home_position, screw


def RxyztoRM(rx, ry, rz):
    R1 = np.around(np.array([
        [1, 0, 0],
        [0, np.cos(rx), -np.sin(rx)],
        [0, np.sin(rx), np.cos(rx)]
    ]), 6)
    R2 = np.around(np.array([
        [np.cos(ry), 0, np.sin(ry)],
        [0, 1, 0],
        [-np.sin(ry), 0, np.cos(ry)]
    ]), 6)
    R3 = np.around(np.array([
        [np.cos(rz), -np.sin(rz), 0],
        [np.sin(rz), np.cos(rz), 0],
        [0, 0, 1]
    ]), 6)
    return R1 @ R2 @ R3

def RzyxtoRM(rz,ry,rx):
    R1 = np.around(np.array([
        [1, 0, 0],
        [0, np.cos(rx), -np.sin(rx)],
        [0, np.sin(rx), np.cos(rx)]
    ]), 6)
    R2 = np.around(np.array([
        [np.cos(ry), 0, np.sin(ry)],
        [0, 1, 0],
        [-np.sin(ry), 0, np.cos(ry)]
    ]), 6)
    R3 = np.around(np.array([
        [np.cos(rz), -np.sin(rz), 0],
        [np.sin(rz), np.cos(rz), 0],
        [0, 0, 1]
    ]), 6)
    return R3 @ R2 @ R1

def crossProductOperator(vector):
    w1, w2, w3 = vector
    W = [
        [0, -w3,  w2],
        [w3,   0, -w1],
        [-w2,  w1,   0]
    ]
    Ax = np.asarray(W)
    return Ax


def exponential_map(action_axis, theta):
    action_axis = np.asarray(action_axis)
    linear = action_axis[3:]
    angular = action_axis[:3]

    exp_rot = exponential_form_rotation(angular, theta)
    exp_tras = exponential_form_traslation(linear, angular, theta)

    expMap = np.block([[exp_rot, exp_tras], [np.zeros((1, 3)), 1]])
    return (expMap)


def exponential_form_rotation(angular, theta):
    c = crossProductOperator(angular) @ crossProductOperator(angular)
    expRot = np.eye(3) + np.sin(theta) * \
        crossProductOperator(angular) + (1-np.cos(theta)) * c
    return expRot


def exponential_form_traslation(linear, angular, theta):
    l1, l2, l3 = linear
    lin = np.array([[l1, l2, l3]])
    angular_mat = crossProductOperator(angular)
    c = angular_mat  @ angular_mat
    expTras = (theta * np.eye(3) + (1 - np.cos(theta)) *
               angular_mat + (theta - np.sin(theta)) * c) @ (lin.transpose())
    return expTras


def Ad(mat4):
    mat4 = np.asarray(mat4)
    rot = mat4[:3, :3]
    tras = mat4[0:3, 3]
    Ad = np.block([[rot, crossProductOperator(tras) @ rot],
                  [np.zeros((3, 3)), rot]])
    return (Ad)


def denavit_transformation(theta, index):
    """
    Computes the homogeneous transformation according to the Denavit-Hatenberg convention
    Parameters
    ----------
    theta : Rotation in z-axis [radians].
    Internal variables
    ------------------
    d : Distance between x-axes in meters.
    alpha : Angle between z_1 and and z_0 axes.
    r : Distance between z-axes.
    Returns
    -------
    G : Homogeneous transformation.
    """

    d = [0.089159, 0, 0, 0.10915, 0.09465, 0.0523]
    alpha = [m.pi/2, 0, 0, m.pi/2, -m.pi/2, 0]
    r = [0, -0.425, -0.3922, 0, 0, 0]

    c_theta = m.cos(theta)
    s_theta = m.sin(theta)
    c_alpha = m.cos(alpha[index])
    s_alpha = m.sin(alpha[index])

    # print('DH Values: ', c_theta, s_theta, c_alpha, s_alpha)

    R_z = np.array([
        [c_theta, -s_theta, 0, 0],
        [s_theta, c_theta, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])  # DH rotation z-axis
    T_z = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, d[index]],
        [0, 0, 0, 1]
    ])  # DH translation z-axis
    T_x = np.array([
        [1, 0, 0, r[index]],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])  # DH translation x-axis
    R_x = np.array([
        [1, 0, 0, 0],
        [0, c_alpha, -s_alpha, 0],
        [0, s_alpha, c_alpha, 0],
        [0, 0, 0, 1]
    ])  # DH rotation x-axis

    # print(R_z, T_z, T_x, R_x)

    G = R_z @ T_z @ T_x @ R_x

    return G


def compute_jacobian(theta, screw, dof=6):
    # Compute DH transformations
    # Compute exponential maps too

    G_local = []
    expMap_local = []
    for i in range(dof):
        G_local.append(denavit_transformation(theta[i], i))
        expMap_local.append(G_local[i] @ exponential_map(screw[i], theta[i]))

    # Get G for the adjoint operator
    G = []
    for i in range(dof):
        if i == 0:
            g = np.eye(4)
        else:
            g = g @ G[i-1]
        G.append(g @ expMap_local[i])

    # Get space Jacobian
    J_s = []
    for i in range(6):
        J_s.append(Ad(G[i]) @ screw[i])

    # print('Space : \n', J_s)

    # Get location of end effector tip (p_tip)
    p_k = np.zeros((3, 1))
    # p_k = np.array([[0],[0],[0.5]])

    p_0_extended = G[5] @ np.block([[p_k], [1]])
    p_0 = p_0_extended[:3]

    p_0_cpo = np.array(
        [[0, -p_0[2], p_0[1]], [p_0[2], 0, -p_0[0]], [-p_0[1], p_0[0], 0]])

    # Geometric Jacobian
    """
    The geometric Jacobian is obtained from the spatial Jacobian and the vector p_tip
    p_tip : tip point of the end effector
    p_0^tip : p measured from the inertial reference frame
    p_k^tip : p measured from the frame k, if k is the end effector and the tip is at its origin then p_k = 0
    [p_0^tip; 1] = G_0^k [p_k^tip; 1]
    """
    J_g = np.block(
        [[np.eye(3), -p_0_cpo], [np.zeros((3, 3)), np.eye(3)]]) @ J_s

    # print('Geometric : \n', J_g)

    # Get Roll, Pitch, Yaw coordinates
    R = G[5][0:3][0:3]

    r_roll = m.atan2(R[2][1], R[2][2])
    r_pitch = m.atan2(-R[2][0], m.sqrt(R[2][2]*R[2][2] + R[2][1]*R[2][1]))
    r_yaw = m.atan2(R[1][0], R[0][0])

    # Build kinematic operator for Roll, Pitch, Yaw configuration
    # Taken from Olguin's formulaire book

    B = np.array(
        [
            [m.cos(r_pitch) * m.cos(r_yaw), -m.sin(r_yaw), 0],
            [m.cos(r_pitch) * m.cos(r_yaw), m.cos(r_yaw), 0],
            [- m.sin(r_pitch), 0, 1]
        ]
    )

    # print('Kinematic : \n', B)

    # Get Analytic Jacobian
    """
    Obtained from function
    J_a(q) = [[I 0],[0, B(alpha)^-1]] J_g(q)
    B(alpha) =    for roll, pitch, yaw
    """
    J_a = np.block(
        [
            [np.eye(3), np.zeros((3, 3))],
            [np.zeros((3, 3)), np.linalg.inv(B)]
        ]
    )

    return J_a


def compute_e(T_d,T):
    e = T_d - T

    # e = theta_d - theta_0
    return e


def root_finding(theta_0, T_d, tryMax, dof, home_position, screw):

    n_try = 1  # Count number of iterations
    tol = 0.0001  # error tolerance
    theta = theta_0
    T = compute_T(screw, theta_0, dof, home_position)
    # Gets error from the transformation matrix
    e = compute_e(T_d, T)

    while n_try < tryMax and np.linalg.norm(e) > tol:
        ja = compute_jacobian(theta, screw)
        j_temp = np.zeros((6, 6))
        for i in range(6):
            for j in range(6):
                j_temp[i][j] = ja[i][j]

        inverse_jacobian = np.linalg.inv(j_temp)
        theta = theta + inverse_jacobian @ (T_d - T)
        e = compute_e(theta_d, theta, dof, home_position, screw)
        n_try += 1
    return theta, e, n_try


def compute_T(screw, theta, dof, M):
    """
    Parameters
    ----------
    screw : screw matrix.
    theta : coordinates.
    dof : robot degrees of freedom.
    M : 4 x 4 matrix describing the position of the end effector at theta_i = 0.
    Returns
    -------
    T : New end effector position.
    """

    expMap_local = []
    T = np.eye(4)
    for i in range(dof):
        expMap_local.append(exponential_map(screw[i], theta[i]))
        T = T @ expMap_local[i]
    T = T @ M
    return T

def main():
    np.set_printoptions(precision=3, suppress=True) # Restrict decimals in console output

    dof = 6
    home_position, screw = ur5_params() # Get UR5 parameters

    theta_0 = np.array([0, 0, 0, 0, 0, 0]) # Initial position

    T_0 = compute_T(screw, theta_0, dof, home_position)
    T_d = np.array([[-0.031, - 0.999,  0.021, 0],
    [-0.999,  0.032,  0.041, - 0.6],
    [-0.042, - 0.019, - 0.999,  0.521],
    [0,     0,     0,     1]])


    print("Home position: \n", T_0, "\n")
    print("Desired position: \n", T_d, "\n")

    # Find solution to the system
    theta, delta, n = root_finding(theta_0, theta_d, 20, dof, home_position, screw)
    T_theta = compute_T(screw, theta, dof, home_position)

    print('Solution : \n', theta, '\n', 'Transformation : \n', T_theta, '\n')

    R = T_theta[0:3][0:3] # Get RPY for the solution

    # r_roll = m.atan2(R[2][1],R[2][2])
    # r_pitch = m.atan2(-R[2][0],m.sqrt(R[2][2]*R[2][2] + R[2][1]*R[2][1]))
    # r_yaw = m.atan2(R[1][0],R[0][0])

    # root_finding()
    # while norm(targetPose - currentPose) > 10e-4:
    #     J_a = compute_jacobian(S_space, M, currentQ);
    #     deltaQ = np.linalg.pinv(J_a)*(targetPose-currentPose)
    #     currentQ = currentQ+deltaQ'
    #     T = fkine(S_body, M, currentQ, 'body')
    #     currentPose = T(1: 3, 4);
    #     if plotOn
    #        try
    #             stanf.teach(currentQ)
    #             plot_ellipse(J_a*J_a', currentPose,'alter', h);
    #         catch e
    #            continue
    #         end
    #     end
if __name__=="__main__":
    main()
