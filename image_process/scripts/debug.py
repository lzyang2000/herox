#!/usr/bin/env python
import cv2
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from rtabmap_ros.msg import MapData
from cv_bridge import CvBridge, CvBridgeError
from message_filters import TimeSynchronizer, Subscriber, ApproximateTimeSynchronizer
import time
import sys
import os
import numpy as np
import pickle
import tf
from geometry_msgs.msg import PoseStamped


if False:
    path = '../../../'
    with open(path + 'debug.pkl','rb') as f:
        coords = pickle.load(f)
        
    TwoD_index = []
    for i in coords:
        TwoD_index.append([i.pose.position.x,i.pose.position.y])
        
    TwoD_index = np.array(TwoD_index)
        
    fig, ax1 = plt.subplots(figsize=(7, 10))
    ax1.plot(TwoD_index[:,1],TwoD_index[:,0], '-o')
    ax1.scatter(TwoD_index[:,1],TwoD_index[:,0], picker=True)
    plt.show()
    
    
sum = 0
def callback_debug(data):
    global sum
    sum += 1
    n = data.nodes
    print(n[0].image)
rospy.init_node('debug', anonymous=True)
rospy.Subscriber("/rtabmap/mapData",MapData,callback=callback_debug)
rospy.spin()

