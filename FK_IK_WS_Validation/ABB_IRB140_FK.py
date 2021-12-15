# Problem 1

from sympy import *

d1, d4, a1, a2, theta1, theta2, theta3, theta4, theta5, theta6 = symbols('d1 d4 a1 a2 theta1 theta2 theta3 theta4 theta5 theta6')


def Transform(theta, d, a, alpha):
    T = Matrix([[cos(theta), (-sin(theta))*cos(alpha), (sin(theta))*sin(alpha), a*cos(theta)],
            [sin(theta), (cos(theta))*cos(alpha), (-cos(theta))*sin(alpha), a*sin(theta)],
            [0, sin(alpha), cos(alpha), d],
            [0, 0, 0, 1]])
    
    return T

def main():

    #DH Table
    dh_frame_1 = [theta1, d1, a1, -pi/2]
    dh_frame_2 = [theta2-pi/2, 0, a2, 0]
    dh_frame_3 = [theta3, 0, 0, -pi/2]
    dh_frame_4 = [theta4, d4, 0, pi/2]
    dh_frame_5 = [theta5, 0, 0, -pi/2]
    dh_frame_6 = [theta6+pi/2, 0, 0, 0]

    # #FK Validation
    # dh_frame_1 = [0, d1, a1, -pi/2]
    # dh_frame_2 = [0-pi/2, 0, a2, 0]
    # dh_frame_3 = [0, 0, 0, -pi/2]
    # dh_frame_4 = [0, d4, 0, pi/2]
    # dh_frame_5 = [0, 0, 0, -pi/2]
    # dh_frame_6 = [0+pi/2, 0, 0, 0]

    # #IK Validation
    # dh_frame_1 = [-2.03, 0.352, 0.07, -pi/2]
    # dh_frame_2 = [-0.06-pi/2, 0, 0.360, 0]
    # dh_frame_3 = [0.004, 0, 0, -pi/2]
    # dh_frame_4 = [0, 0.38, 0, pi/2]
    # dh_frame_5 = [1.633, 0, 0, -pi/2]
    # dh_frame_6 = [-2.034+pi/2, 0.3, 0, 0]



    T01 = Transform(dh_frame_1[0], dh_frame_1[1], dh_frame_1[2], dh_frame_1[3])
    # print('T01 = ')
    # print()
    # pprint(T01)
    # print()

    T12 = Transform(dh_frame_2[0], dh_frame_2[1], dh_frame_2[2], dh_frame_2[3])
    # print('T12 = ')
    # print()
    # pprint(T12)
    # print()

    T23 = Transform(dh_frame_3[0], dh_frame_3[1], dh_frame_3[2], dh_frame_3[3])
    # print('T23 = ')
    # print()
    # pprint(T23)
    # print()

    T34 = Transform(dh_frame_4[0], dh_frame_4[1], dh_frame_4[2], dh_frame_4[3])
    # print('T34 = ')
    # print()
    # pprint(T34)
    # print()

    T45 = Transform(dh_frame_5[0], dh_frame_5[1], dh_frame_5[2], dh_frame_5[3])
    # print('T45 = ')
    # print()
    # pprint(T45)
    # print()

    T56 = Transform(dh_frame_6[0], dh_frame_6[1], dh_frame_6[2], dh_frame_6[3])
    # print('T56 = ')
    # print()
    # pprint(T56)
    # print() 


    print('dh_frame_1 =')
    print()
    pprint(dh_frame_1)
    print()

    print('dh_frame_2 =')
    print()
    pprint(dh_frame_2)
    print()

    print('dh_frame_3 =')
    print()
    pprint(dh_frame_3)
    print()

    print('dh_frame_4 =')
    print()
    pprint(dh_frame_4)
    print()

    print('dh_frame_5 =')
    print()
    pprint(dh_frame_5)
    print()

    print('dh_frame_6 =')
    print()
    pprint(dh_frame_6)
    print()


    T06 = T01 * T12 * T23 * T34 * T45 * T56
    print('T06 = ')
    print()
    pprint(T06)
    print()




main()
