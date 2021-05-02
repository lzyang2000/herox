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
import skimage.io as skio
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped


class tracer():
    def __init__(self):
        rospy.init_node('tracer', anonymous=True)
        self.odom_sub = rospy.Subscriber("image_process/sync_odom",Odometry,callback=self.callback_odom)
        self.map_sub = rospy.Subscriber("map",OccupancyGrid,callback=self.callback_map)

        self.raytrace_pub = rospy.Publisher("raytrace",OccupancyGrid,queue_size=1,latch=True)

        self.tf_listener = tf.TransformListener()
        self.odomList = []
        self.odomThresh = 0.01

        self.last_odom = None
        self.latest_map = None


    def callback_odom(self,data):
        pose = PoseStamped()
        pose.header=data.header
        pose.pose=data.pose.pose
        try:
            odom = self.tf_listener.transformPose('map',pose)
        except:
            print('TF error in tracer')
            return

        if self.last_odom is None:
            self.last_odom = odom
            self.odomList.append(odom)
        else:
            if (np.sqrt((self.last_odom.pose.position.x - odom.pose.position.x) ** 2 +
                        (self.last_odom.pose.position.y - odom.pose.position.y) ** 2) > self.odomThresh):
                self.last_odom = odom
                self.odomList.append(odom)

    def callback_map(self, data):
        print('map value', np.unique(data.data))
        map = np.array(data.data)
        
        unknown = map == -1
        wall = map == 100
        free = map == 0
        
        canvas = np.zeros(map.shape)
        canvas[unknown] = 205
        canvas[wall] = 0
        canvas[free] = 254
        map = canvas.astype(np.uint8)
        
        size = int(np.sqrt(map.shape[0]))
        self.latest_map = map.reshape((size, size))[::-1]
        skio.imsave('test.png', self.latest_map)
        print('map shape', self.latest_map.shape)

    def raytrace(self):
        #print('raytrace')
        pass


if __name__ == '__main__':
    T = tracer()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if T.latest_map is not None and len(T.odomList):
            T.raytrace()
        else:
            print('empty info, sleep')
            rate.sleep()

