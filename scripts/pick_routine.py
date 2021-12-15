#!/usr/bin/env python3
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
import std_msgs.msg
from irb140_commander.msg import PoseRPY,PoseRPYarray

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import rospy
import copy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np

path_pub = rospy.Publisher('robot_commander/cmd_path', PoseRPYarray, queue_size=10)
gripper_pub = rospy.Publisher('irb_140/gripper_controller/command', JointTrajectory, queue_size=10)
#/gripper/camera1/image_raw
rospy.init_node('pathSender', anonymous=True)
rate = rospy.Rate(2) # 10hz
rate.sleep()

lists=PoseRPYarray()
lists.eef_step=0.01
points=PoseRPY()
grip_ctrl=JointTrajectory()
grip_points= JointTrajectoryPoint()

grip_ctrl.joint_names.append("gripper_body__left_ext")
grip_points.time_from_start = rospy.Duration(1)
h = std_msgs.msg.Header()

def gripper_action(apertura):
    h.stamp = rospy.Time.now()
    grip_ctrl.header=h
    grip_points.positions=[apertura]
    grip_ctrl.points=[grip_points]
    gripper_pub.publish(grip_ctrl)
    print("gripper command")

def position1():
    gripper_action(0.23)

    #points.position.x=0.5;points.position.y=0.0;points.position.z=0.5
    points.position.x=0.15;points.position.y=-0.43;points.position.z=0.6
    #points.rpy.roll=0;points.rpy.pitch=0;points.rpy.yaw=-1.57
    points.rpy.roll=0;points.rpy.pitch=1.57;points.rpy.yaw=0
    lists.poses.append(copy.deepcopy(points))
    path_pub.publish(lists)
    print("start_position")
    msg=rospy.wait_for_message('/robot_commander/path_done',std_msgs.msg.String)
    print("path_traversal")
    lists.poses.clear()

    points.position.z=0.4
    lists.poses.append(copy.deepcopy(points))
    path_pub.publish(lists)
    msg=rospy.wait_for_message('/robot_commander/path_done',std_msgs.msg.String)
    print("pick_object")
    lists.poses.clear()

    gripper_action(0.53)
    gripper_action(-0.2)
    #msg=rospy.wait_for_message('/gazebo/gripper/gripped',std_msgs.msg.String)
    print("object_picked")

    points.position.z=0.5
    lists.poses.append(copy.deepcopy(points))
    points.position.x=-0.25;points.position.y=-0.4;points.position.z=0.43
    #points.rpy.roll=0;points.rpy.pitch=0;points.rpy.yaw=-1.57
    points.rpy.roll=0;points.rpy.pitch=1.57;points.rpy.yaw=0
    lists.poses.append(copy.deepcopy(points))
    points.position.y=-0.38;points.position.z=0.42
    lists.poses.append(copy.deepcopy(points))
    path_pub.publish(lists)
    print("task completed")
    msg=rospy.wait_for_message('/robot_commander/path_done',std_msgs.msg.String)
    print("trajectory completed")
    lists.poses.clear()

    #suelta objeto
    gripper_action(0.52)
    print("routine done")

def position2():
    gripper_action(0.23)
    #points.position.x=0.5;points.position.y=0.0;points.position.z=0.5
    points.position.x=0.3;points.position.y=-0.4;points.position.z=0.6
    points.rpy.roll=0;points.rpy.pitch=1.57;points.rpy.yaw=0
    lists.poses.append(copy.deepcopy(points))
    path_pub.publish(lists)
    print("start_position")
    msg=rospy.wait_for_message('/robot_commander/path_done',std_msgs.msg.String)
    print("path_traversal")
    lists.poses.clear()

    points.position.z=0.38
    lists.poses.append(copy.deepcopy(points))
    path_pub.publish(lists)
    msg=rospy.wait_for_message('/robot_commander/path_done',std_msgs.msg.String)
    print("pick_object")
    lists.poses.clear()

    gripper_action(0.53)
    gripper_action(-0.22)
    #msg=rospy.wait_for_message('/gazebo/gripper/gripped',std_msgs.msg.String)
    print("object_picked")

    points.position.z=0.5
    lists.poses.append(copy.deepcopy(points))
    points.position.x=-0.19;points.position.y=-0.4;points.position.z=0.43
    #points.rpy.roll=0;points.rpy.pitch=0;points.rpy.yaw=-1.57
    points.rpy.roll=0;points.rpy.pitch=1.57;points.rpy.yaw=0
    lists.poses.append(copy.deepcopy(points))
    points.position.x=-0.19;points.position.y=-0.38;points.position.z=0.43
    lists.poses.append(copy.deepcopy(points))
    path_pub.publish(lists)
    print("task completed")
    msg=rospy.wait_for_message('/robot_commander/path_done',std_msgs.msg.String)
    print("trajectory completed")
    lists.poses.clear()
    gripper_action(0.52)
    print("routine done")

position1()
position2()

br = CvBridge()

def callback(msg):
    print("Image received...")
    image = br.imgmsg_to_cv2(msg)
    print(image)
    cv2.imshow("frame", image)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.Subscriber('/gripper/camera1/image_raw', Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    cv2.namedWindow("frame", 1)

if __name__ == '__main__':
    listener()