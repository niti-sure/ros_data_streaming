#!/usr/bin/env python

import csv
import rospy
import message_filters
from std_msgs.msg import Int32, Float32, String, Header
from sensor_msgs.msg import Image, CameraInfo 
#import pandas as pd

def callback(image, data, info):
 # The callback processing the pairs of numbers that arrived at approximately the same time
    #print("Got image and IMU data!")
    X = (float(data.data[data.data.index("A") + 1:data.data.index("B")]))
    Y = (float(data.data[data.data.index("B") + 1:data.data.index("C")]))
    Z = (float(data.data[data.data.index("C") + 1:data.data.index("D")]))
    Ax = (float(data.data[data.data.index("D") + 1:data.data.index("E")]))
    Ay = (float(data.data[data.data.index("E") + 1:data.data.index("F")]))
    Az = (float(data.data[data.data.index("F") + 1:data.data.index("G")]))
    Gx = (float(data.data[data.data.index("G") + 1:data.data.index("H")]))
    Gy = (float(data.data[data.data.index("H") + 1:data.data.index("I")]))
    Gz = (float(data.data[data.data.index("I") + 1:data.data.index("J")]))
    
  #  teen = data.data
    
    tsec = int(info.header.stamp.secs)
   # print(nsec)
    #(int(info["header"]["stamp"]["nsecs"]))
    rrow = [X, Y, Z, Ax, Ay, Az, Gx, Gy, Gz, tsec]
    writer.writerow(rrow) 
    
    
#    print("X: ", X)
#    print("Y: ", Y)
#    print("Z: ", Z)
    
def time_sync():


    rospy.init_node('synchronizer', anonymous = True)
    image_sub = message_filters.Subscriber('/usb_cam/image_raw', Image)
    data_sub = message_filters.Subscriber('/imu', String) 
    info_sub = message_filters.Subscriber('/usb_cam/camera_info', CameraInfo)
    ts = message_filters.ApproximateTimeSynchronizer([image_sub, data_sub, info_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback)
    rospy.spin()

if __name__== '__main__':
    header = ['X', 'Y', 'Z', 'Ax', 'Ay', 'Az', 'Gx', 'Gy', 'Gz', 'tsecs']
    with open('/home/nitika/catkin_ws/saved_data_1.csv', 'w', encoding='UTF8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header)
        time_sync()
        writer.close()
