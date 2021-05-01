#!/usr/bin/env python
import cv2
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from cv_bridge import CvBridge, CvBridgeError
from message_filters import TimeSynchronizer, Subscriber, ApproximateTimeSynchronizer
import time
import sys
import os
import numpy as np
import pickle
import tf
from geometry_msgs.msg import PoseStamped
bridge = CvBridge()

class listen():
    def __init__(self):
        rospy.init_node('listener', anonymous=True)

        self.odom_sub = rospy.Subscriber("/odom",Odometry,callback=self.callback_path)
        self.odom_pub = rospy.Publisher("/correct_odom",Odometry,queue_size=1)
        self.listener = tf.TransformListener()
        
    
    def callback_path(self,data):
        # self.mapList.append(data.poses[-1])
        #self.lastMap = data.poses
        pose = PoseStamped()
        pose.header=data.header
        pose.pose=data.pose.pose
        try:
            data.pose.pose = (self.listener.transformPose('map',pose).pose)
            self.odom_pub.publish(data)
        except:
            print('error')


        
        
        
if __name__ == '__main__':
    listen()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        sys.exit(0)