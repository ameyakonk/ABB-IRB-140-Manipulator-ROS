#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np

import rospy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def callback(data):

    plt.close('all')
    print(len(data.points))
    points=np.arange(0,len(data.points),1)
    joint1=[];joint2=[];joint3=[];joint4=[];joint5=[];joint6=[];
    jointv1=[];jointv2=[];jointv3=[];jointv4=[];jointv5=[];jointv6=[];
    time=[];
    for i in range((len(data.points))):
        joint1.append(data.points[i].positions[0]);joint2.append(data.points[i].positions[1]);joint3.append(data.points[i].positions[2])
        joint4.append(data.points[i].positions[3]);joint5.append(data.points[i].positions[4]);joint6.append(data.points[i].positions[5])
        jointv1.append(data.points[i].velocities[0]);jointv2.append(data.points[i].velocities[1]);jointv3.append(data.points[i].velocities[2])
        jointv4.append(data.points[i].velocities[3]);jointv5.append(data.points[i].velocities[4]);jointv6.append(data.points[i].velocities[5])
        time.append(round(data.points[i].time_from_start.to_sec(), 2))

    fig=plt.figure(figsize=(15,8))
    ax = fig.add_subplot(111)    # The big subplot
    fig.text(0.5, 0.04, 'x=Time[s]', ha='center', va='center',bbox={'facecolor': 'red', 'alpha': 0.5, 'pad': 10},fontsize=15)
    fig.text(0.06, 0.5, 'y=Angles [rad]', ha='center', va='center', rotation='vertical',bbox={'facecolor': 'red', 'alpha': 0.5, 'pad': 10},fontsize=15)

    print(time)

    def plot_joint(datosx,datosy,stilo,tag,subplot):
        plt.subplot(subplot)
        plt.plot(datosx, datosy, stilo, label=tag)  # Plot some data on the axes.
        plt.grid()
        # plt.xticks(np.arange(0,len(data.points)+1,1))
        plt.xticks(list(range(len(time))),time)
        plt.legend()

    plot_joint(points,joint1,'--ro','joint1',321)
    plot_joint(points,joint2,'--bo','joint2',322)
    plot_joint(points,joint3,'--go','joint3',323)
    plot_joint(points,joint4,'--ko','joint4',324)
    plot_joint(points,joint5,'--yo','joint5',325)
    plot_joint(points,joint6,'--mo','joint6',326)
    fig.suptitle('Angles, total points='+str(len(data.points)), fontsize=20)

    fig=plt.figure(figsize=(15,8))
    plot_joint(points,jointv1,'--ro','joint1',321)
    plot_joint(points,jointv2,'--bo','joint2',322)
    plot_joint(points,jointv3,'--go','joint3',323)
    plot_joint(points,jointv4,'--ko','joint4',324)
    plot_joint(points,jointv5,'--yo','joint5',325)
    plot_joint(points,jointv6,'--mo','joint6',326)
    fig.suptitle('Velocity, total points='+str(len(data.points)), fontsize=20)
    fig.text(0.5, 0.04, 'x=time[s]', ha='center', va='center',bbox={'facecolor': 'green', 'alpha': 0.5, 'pad': 10},fontsize=15)
    fig.text(0.06, 0.5, 'y=Vel.Angular [rad/s]', ha='center', va='center', rotation='vertical',bbox={'facecolor': 'green', 'alpha': 0.5, 'pad': 10},fontsize=15)

    plt.show()

if __name__ == '__main__':

    x=np.arange(0,6,1)
    print(x)
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/robot_commander/robot_traj', JointTrajectory, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
