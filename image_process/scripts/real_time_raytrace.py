#!/usr/bin/env python
import cv2
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from cv_bridge import CvBridge, CvBridgeError
from message_filters import TimeSynchronizer, Subscriber, ApproximateTimeSynchronizer
import time
import sys
import os
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped


class tracer():
    def __init__(self):
        rospy.init_node('tracer', anonymous=True)
        self.odom_sub = rospy.Subscriber("image_process/sync_odom",Odometry,callback=self.callback_odom)
        self.map_sub = rospy.Subscriber("map",OccupancyGrid,callback=self.callback_map)
        self.raytrace_pub = 
        self.tf_listener = tf.TransformListener()
        self.odomList = []
        self.latest_map = None
        while not rospy.is_shutdown() and len(self.odomList) and self.latest_map is not None:
            self.raytrace()
            
        print('done initialization')


    def callback_odom(self,data):
        pose = PoseStamped()
        pose.header=data.header
        pose.pose=data.pose.pose
        try:
            self.odomList.append(self.tf_listener.transformPose('map',pose))
        except:
            print('TF error in tracer')

    def callback_map(self, data):
        self.latest_map = np.array(data.data)
        print('map shape', self.latest_map.shape)

    def raytrace(self):
        print('raytrace')


if __name__ == '__main__':
    tracer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        sys.exit(0)