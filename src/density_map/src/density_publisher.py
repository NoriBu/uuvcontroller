#!/usr/bin/env python

import rospy
from uuv_gazebo_ros_plugins_msgs.srv import SetFloat
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import numpy as np

pub = rospy.Publisher('density', Float32, queue_size=10)

def change_density(x):
    rospy.wait_for_service('/uuvfloat/set_fluid_density')

    try:
        set_density = rospy.ServiceProxy('/uuvfloat/set_fluid_density', SetFloat)
        resp1 = set_density(x)
        pub.publish(x)
        return resp1

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def calculate_density(depth):

    ppar = [0.0006215, 0.02046, 1004]
    p = np.poly1d(ppar)
    density = p(abs(depth))

    change_density(density)


def callback(data):
    calculate_density(data.pose.pose.position.z)

def listener():
    rospy.init_node('density_publisher', anonymous=True)
    rospy.Subscriber("/uuvfloat/pose_gt", Odometry, callback)
    
    rospy.spin()

if __name__ == '__main__':
    listener()

