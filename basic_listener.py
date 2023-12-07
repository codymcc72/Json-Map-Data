#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix

def gps_callback(gps_data):
    x_gps = gps_data.longitude
    y_gps = gps_data.latitude
    timestamp = gps_data.header.stamp

    print("GPS Data")
    print("X: {:.6f}".format(x_gps))
    print("Y: {:.6f}".format(x_gps))
    print("Timestamp: {}".format(timestamp))
    print("\n")

def gps_listener():
    rospy.init_node('gps_listener', anonymous=True)
    rospy.Subscriber('tric_navigation/gps/head_data', NavSatFix, gps_callback)
    rospy.spin()

if __name__ == '__main__':
    gps_listener()
