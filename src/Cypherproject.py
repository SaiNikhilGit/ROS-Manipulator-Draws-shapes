#!/usr/bin/env python3

import math
import numpy as np
import matplotlib.pyplot as plt
import rospy
import tinyik as ti
from std_msgs.msg import Float64

#ini_pos = [1.0, 0.0, 1]
ini_pos=[1.0 ,0.0,2]


def Draw_circle(r):
    x = []
    z = []
    circle_pointlis = []
    theta = np.linspace(0, np.pi * 2, 150)
    for i in theta:
        x.append(r * np.cos(i))
        z.append(r * np.sin(i))
    for i in range(len(x)):
        circle_pointlis.append([ini_pos[0], ini_pos[1] + x[i], ini_pos[2] + z[i]])
    return circle_pointlis


def Make_Triangle(a):
    r = a / math.sqrt(3)
    point_1 = [ini_pos[0], ini_pos[1], ini_pos[2] + r]
    point_2 = [ini_pos[0], ini_pos[1] + 0.5 * a, ini_pos[2] - 0.5 * r]
    point_3 = [ini_pos[0], ini_pos[1] - 0.5 * a ,ini_pos[2] - 0.5 * r]
    triangle_pointlis = [point_1, point_2, point_3, point_1]
    return triangle_pointlis




def Make_square(d):
    point_1 = [ini_pos[0], ini_pos[1] + d / 2, ini_pos[2] ]
    point_2 = [ini_pos[0], ini_pos[1] - d / 2, ini_pos[2] ]
    point_3 = [ini_pos[0], ini_pos[1] - d / 2, ini_pos[2] - d ]
    point_4 = [ini_pos[0], ini_pos[1] + d / 2, ini_pos[2] - d ]
    sqr_pointlis = [point_1, point_4, point_3, point_2]
    print(sqr_pointlis)
    return sqr_pointlis


def Make_rectan(l, b):
    point_1 = [ini_pos[0], ini_pos[1] + l / 2, ini_pos[2] ]
    point_2 = [ini_pos[0], ini_pos[1] - l / 2, ini_pos[2] ]
    point_3 = [ini_pos[0], ini_pos[1] - l / 2, ini_pos[2] - b ]
    point_4 = [ini_pos[0], ini_pos[1] + l / 2, ini_pos[2] - b ]
    rectangle_pointlis = [point_1, point_4, point_3, point_2]
    return rectangle_pointlis


def Make_pentagon(s):
    pentagon = []
    R = s
    for n in range(0, 5):
        x = R * math.cos(math.radians(90 + n * 72))
        y = R * math.sin(math.radians(90 + n * 72))
        pentagon.append([x, y])

    point_1 = [ini_pos[0], ini_pos[1]+pentagon[0][0] , ini_pos[2]+pentagon[0][1] ]
    point_2 = [ini_pos[0], ini_pos[1]+pentagon[1][0] , ini_pos[2]+pentagon[1][1] ]
    point_3 = [ini_pos[0], ini_pos[1]+pentagon[2][0] , ini_pos[2]+pentagon[2][1] ]
    point_4 = [ini_pos[0], ini_pos[1]+pentagon[3][0] , ini_pos[2]+pentagon[3][1] ]
    point_5 = [ini_pos[0], ini_pos[1]+pentagon[4][0],  ini_pos[2]+pentagon[4][1] ]
    pentagon_pointlis=[point_1,point_2,point_3,point_4,point_5,point_1]
    return pentagon_pointlis

    print(point_1)
    print(point_2)

        


manip = ti.Actuator(
    ['z', [0., 0., 0.4], 'y', [0., 0., 0.8], 'y', [0., 0., 0.8], 'y', [0., 0., 0.8], 'y', [0., 0., 0.25]])

error4x = 0.085736
error4y = 0.085736
error4z = -0.714264

arm_pub_1 = rospy.Publisher('/firstjoint_position_controller/command', Float64, queue_size=10)
arm_pub_2 = rospy.Publisher('/secondjoint_position_controller/command', Float64, queue_size=10)
arm_pub_3 = rospy.Publisher('/thirdjoint_position_controller/command', Float64, queue_size=10)
arm_pub_4 = rospy.Publisher('/fourthjoint_position_controller/command', Float64, queue_size=10)
arm_pub_5 = rospy.Publisher('/fifthjoint_position_controller/command', Float64, queue_size=10)

rospy.init_node('manip')
rate = rospy.Rate(2)
rate2 = rospy.Rate(300)

def mover_func():
    choice = 1
    circleop = 0

    while choice != 0:

        move_arm(ini_pos, circleop)
        rate.sleep()
        print("drawing shape options :")
        print("1. square ")
        print("2. rectangle ")
        print("3. circle")
        #print("4. kite")
        print("4. triangle")
        print("5. pentagon")
        print("6. EXIT")
        choice = int(input("Give a particular option to Draw "))

        if choice == 3:
            r = float(input("Enter the radius of the circle  "))
            vertices = Draw_circle(r)
            c = int(input("Rotations: "))
            circleop = 1

        elif choice == 4:
            a = float(input("Enter the length of the TRAINGLE   "))
            vertices = Make_Triangle(a)
            c = int(input("Rotations "))
            circleop = 0

        elif choice == 1:
            d = float(input("Enter the length "))
            vertices = Make_square(d)
            c = int(input("Rotations :"))
            circleop = 0

        elif choice == 2:
            l = float(input("Enter the length "))
            b = float(input("Enter the breadth "))
            vertices = Make_rectan(l, b)
            c = int(input("Rotations : "))
            circleop = 0


        elif choice == 5:
            s = float(input("Enter the side length of pentagon "))
            vertices = Make_pentagon(s)
            c = int(input("Rotations : "))
            circleop = 0


        else:
            vertices = ini_pos

        i = 0
        count = 0

        while (i < len(vertices) and count < c):
            if i == len(vertices) - 1:
                if circleop == 1:
                    rate2.sleep()
                else:
                    rate.sleep()
                move_arm(vertices[i], circleop)
                i = 0
                count += 1

            else:
                if circleop == 1:
                    rate2.sleep()
                else:
                    rate.sleep()
                move_arm(vertices[i], circleop)
                i += 1

        count = 0
    circleop = 0
    move_arm(ini_pos, circleop)

def move_arm(point, circleop):
    manip.ee = [point[0] + error4x, point[1] + error4y, point[2] + error4z]

    angles = manip.angles
    joint_angle_1 = angles[0]
    joint_angle_2 = angles[1]
    joint_angle_3 = angles[2]
    joint_angle_4 = angles[3]
    joint_angle_5 = angles[4]

    move_joint_1 = Float64()
    move_joint_2 = Float64()
    move_joint_3 = Float64()
    move_joint_4 = Float64()
    move_joint_5 = Float64()

    move_joint_1 = joint_angle_1
    rospy.loginfo(move_joint_1)
    arm_pub_1.publish(move_joint_1)
    if circleop == 1:
        rate2.sleep()
    else:
        rate.sleep()

    move_joint_2 = joint_angle_2
    rospy.loginfo(move_joint_2)
    arm_pub_2.publish(move_joint_2)
    if circleop == 1:
        rate2.sleep()
    else:
        rate.sleep()

    move_joint_3 = joint_angle_3
    rospy.loginfo(move_joint_3)
    arm_pub_3.publish(move_joint_3)
    if circleop == 1:
        rate2.sleep()
    else:
        rate.sleep()

    move_joint_4 = joint_angle_4
    rospy.loginfo(move_joint_4)
    arm_pub_4.publish(move_joint_4)
    if circleop == 1:
        rate2.sleep()
    else:
        rate.sleep()

    move_joint_5 = joint_angle_5
    rospy.loginfo(move_joint_5)
    arm_pub_5.publish(move_joint_5)
    if circleop == 1:
        rate2.sleep()
    else:
        rate.sleep()

mover_func()