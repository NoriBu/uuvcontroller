#!/usr/bin/env python

import numpy as np
import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Float32
from uuv_gazebo_ros_plugins_msgs.srv import SetFloat
from uuv_gazebo_ros_plugins_msgs.srv import *
from uuv_gazebo_ros_plugins_msgs.msg import UnderwaterObjectModel

altidata  = np.Inf
stop_service = False
logging = 0
init_loop = True
density = np.Inf
density_map_data = []
altimeter_map_data = []

def change_volume(offset):

    rospy.wait_for_service('/uuvfloat/set_volume_offset')

    try:
        set_volume = rospy.ServiceProxy('/uuvfloat/set_volume_offset', SetFloat)
        resp1 = set_volume(offset)
        print("Volume set")
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def read_altimeter(data):

    global altidata
    altidata = data.range

def read_density(data):

    global density
    density = data.data

def listener():

    global altidata, stop_service, logging, init_loop, density

    rospy.init_node('depthlogic', anonymous=True)

    rospy.Subscriber("/uuvfloat/altimeter", Range, read_altimeter)
    rate = rospy.Rate(5)

    if init_loop:
        if stop_service == False:
            surface_density = rospy.wait_for_message("/density", Float32)
            print(surface_density.data)
            change_volume(-0.5)
            stop_service = True

        while (not rospy.is_shutdown()) and (logging == 0):
            if (altidata < 8):
                change_volume(0)
                logging = 1
            rate.sleep()

        rospy.Subscriber("/density", Float32, read_density)
        rate = rospy.Rate(5)

        while (not rospy.is_shutdown()) and logging == 1:

            if density != np.Inf:
                density_map_data.append(density)
                altimeter_map_data.append(altidata)

            if (density <= surface_density.data):
                logging = 0
                init_loop = False
                
            rate.sleep()

    if not init_loop:
        altim = [altimeter_map_data[-1] - altimeter_map_data[i] for i in range(len(altimeter_map_data))]
        pfit = np.polyfit(altim, density_map_data, 2)
        pol_our = np.poly1d(pfit)
        print(pol_our)

if __name__ == '__main__':
    listener()
