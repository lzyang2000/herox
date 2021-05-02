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
        self.image_sub_rgb = rospy.Subscriber("image_process/sync_img",Image,callback=self.callback_image)
        self.image_sub_ir = rospy.Subscriber("image_process/sync_ir",Image,callback=self.callback_IR)
        self.odom_sub = rospy.Subscriber("image_process/sync_odom",Odometry,callback=self.callback_path)
        self.imgList = []
        self.irList = []
        self.mapList = []
        self.listener = tf.TransformListener()

        
        self.lastMap = None
        rospy.on_shutdown(self.shutdown)
        
    
    def callback_path(self,data):
        # self.mapList.append(data.poses[-1])
        #self.lastMap = data.poses
        pose = PoseStamped()
        pose.header=data.header
        pose.pose=data.pose.pose
        try:
            self.mapList.append(self.listener.transformPose('map',pose))
        except:
            print('error')

    def callback_image(self,data):
        image_cv = bridge.imgmsg_to_cv2(data, "rgb8")
        self.imgList.append(image_cv)
        
    def callback_IR(self,data):
        image_cv = bridge.imgmsg_to_cv2(data, "mono8")
        self.irList.append(image_cv)
        
    def shutdown(self):
        c = 0
        if len(self.imgList) > len(self.mapList):
            diff = len(self.imgList) - len(self.mapList)
            self.imgList = self.imgList[diff:]
            self.irList = self.irList[diff:]
        print(len(self.imgList), len(self.irList), len(self.mapList))
        for i,j,k in zip(self.imgList,self.irList,self.mapList):
            cv2.imwrite(str(c)+'_RGB.jpg',i)
            cv2.imwrite(str(c)+'_IR.jpg',j)
        #for k in self.mapList:   
            with open(str(c)+".pkl",'wb') as f:
                pickle.dump(k,f)
                
            c+=1
        #with open("debug.pkl",'wb') as f:
            #pickle.dump(self.lastMap, f)
        
        
        
if __name__ == '__main__':
    listen()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        sys.exit(0)