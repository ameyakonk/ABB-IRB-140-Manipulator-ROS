# Ninad Harishchandrakar HW4
# Problem 1

from sympy import *
import numpy as np
import matplotlib.pyplot as plt

d1, d4, a1, a2, theta1, theta2, theta3, theta4, theta5, theta6 = symbols('d1 d4 a1 a2 theta1 theta2 theta3 theta4 theta5 theta6')


def Transform(theta, d, a, alpha):
    T = Matrix([[cos(theta), (-sin(theta))*cos(alpha), (sin(theta))*sin(alpha), a*cos(theta)],
            [sin(theta), (cos(theta))*cos(alpha), (-cos(theta))*sin(alpha), a*sin(theta)],
            [0, sin(alpha), cos(alpha), d],
            [0, 0, 0, 1]])
    
    return T


def main():

    dh_frame_1 = [theta1, 352, 70, -pi/2]
    dh_frame_2 = [theta2-pi/2, 0, 360, 0]
    dh_frame_3 = [theta3, 0, 0, -pi/2]
    dh_frame_4 = [theta4, 380, 0, pi/2]
    dh_frame_5 = [theta5, 0, 0, -pi/2]
    dh_frame_6 = [theta1+pi/2, 0, 0, 0]

    # Forward Kinematics
    T01 = Transform(dh_frame_1[0], dh_frame_1[1], dh_frame_1[2], dh_frame_1[3])
    T12 = Transform(dh_frame_2[0], dh_frame_2[1], dh_frame_2[2], dh_frame_2[3])
    T23 = Transform(dh_frame_3[0], dh_frame_3[1], dh_frame_3[2], dh_frame_3[3])
    T34 = Transform(dh_frame_4[0], dh_frame_4[1], dh_frame_4[2], dh_frame_4[3])
    T45 = Transform(dh_frame_5[0], dh_frame_5[1], dh_frame_5[2], dh_frame_5[3])
    T56 = Transform(dh_frame_6[0], dh_frame_6[1], dh_frame_6[2], dh_frame_6[3])

    T02 = T01 * T12
    T03 = T02 * T23
    T04 = T03 * T34
    T05 = T04 * T45
    T06 = T05 * T56

    ## Find O matrices
    O0 = Matrix([0, 0, 0])
    O1 = T01[0:3, 3]
    O2 = T02[0:3, 3]
    O3 = T03[0:3, 3]
    O4 = T04[0:3, 3]
    O5 = T05[0:3, 3]
    O6 = T06[0:3, 3]

    ## Find Z matrices
    Z0 = Matrix([0, 0, 1])
    Z1 = T01[0:3, 2]
    Z2 = T02[0:3, 2]
    Z3 = T03[0:3, 2]
    Z4 = T04[0:3, 2]
    Z5 = T05[0:3, 2]
    Z6 = T06[0:3, 2]

    ## Find J column matrices
    J1 = (Z0.cross(O6-O0)).row_insert(3, Z0)
    J2 = (Z1.cross(O6-O1)).row_insert(3, Z1)
    J3 = (Z2.cross(O6-O2)).row_insert(3, Z2)
    J4 = (Z3.cross(O6-O3)).row_insert(3, Z3)
    J5 = (Z4.cross(O6-O4)).row_insert(3, Z4)
    J6 = (Z5.cross(O6-O5)).row_insert(3, Z5)

    ## Jacobian matrix
    J = (((((J1.row_join(J2)).row_join(J3).row_join(J4)).row_join(J5)).row_join(J6)))
    print('Jacobian J = ')
    print()
    pprint(J)
    print()


    # dT = 1  #Time
    # iter = 200  #No. of Iterations
    # theta_cyl = 0 #Cylindrical coordinate theta
    # theta_cyl_dot = ((6.28)/iter)/dT
    # Q = Matrix([0.1, 0, 0, 0, 0, 0])
    # r = 100 #Cylindrical coordinates radius
   
    # i = 0

    # while i < iter:
        
    #     J_temp = J.subs([(theta1, Q[0]), (theta2, Q[1]), (theta3, Q[2]), (theta4, Q[3]), (theta5, Q[4]), (theta6, Q[5])])

    #     J_temp = np.array(J_temp).astype(np.float64)
    #     J_inv = np.linalg.inv(J_temp)
    #     J_inv = Matrix(J_inv)

    #     # X_dot of cylindrical matrix
    #     X_dot = Matrix([(-r*(cos(theta_cyl))*theta_cyl_dot), 0, (r*(sin(theta_cyl))*theta_cyl_dot), 0, 0, 0])

    #     # Inverse Velocity Kinematics
    #     Q_dot = J_inv * X_dot

    #     Q = Q + ((dT)*(Q_dot))

    #     # Calculate coordinates of end effector through T07
    #     x_pen = (T06.subs([(theta1, Q[0]), (theta2, Q[1]), (theta3, Q[2]), (theta4, Q[3]), (theta5, Q[4]), (theta6, Q[5])]))[0, 3]
    #     z_pen = (T06.subs([(theta1, Q[0]), (theta2, Q[1]), (theta3, Q[2]), (theta4, Q[3]), (theta5, Q[4]), (theta6, Q[5])]))[2, 3]

    #     print(x_pen, z_pen)

    #     plt.scatter(x_pen, z_pen)

    #     theta_cyl = theta_cyl + (6.28/iter)
    #     i = i + 1


    # plt.show()


main()
