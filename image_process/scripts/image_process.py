#!/usr/bin/env python

import cv2
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge, CvBridgeError
from message_filters import TimeSynchronizer, Subscriber, ApproximateTimeSynchronizer
import time
import sys
import os
import numpy as np
import pickle
class processor:
    def __init__(self,):
        self.bridge = CvBridge()
        self.image_sub_EO = Subscriber(rospy.get_param('rgb_topic'), Image)
        self.image_sub_IR = Subscriber(rospy.get_param('ir_topic'), Image)
        self.odom_sub = Subscriber(rospy.get_param('odom_topic'), Odometry)
        self.sync =  ApproximateTimeSynchronizer([self.image_sub_EO, self.image_sub_IR,self.odom_sub], 60,rospy.get_param('sync_delta',0.005), allow_headerless=True)
        self.image_pub_rgb = rospy.Publisher("image_process/sync_img",Image,queue_size=5)
        self.image_pub_ir = rospy.Publisher("image_process/sync_ir",Image,queue_size=5)
        self.odom_pub = rospy.Publisher("image_process/sync_odom",Odometry,queue_size=5)
        self.sync.registerCallback(self.callback)
        self.count = 0.0
        self.start = True
        self.start_time = 0.0
        self.image_rgb_list = []
        self.image_thm_list = []
        self.odom_list = []
        self.rate =  rospy.Rate(1)
        rospy.on_shutdown(self.shutdown)
        
    def callback(self,image_rgb,image_ir,odom):
        try:
            if self.start:
                self.start_time =  rospy.get_time()
                self.start = False
            # rgb_image_cv = self.bridge.imgmsg_to_cv2(image_rgb, "rgb8")
            # thm_image_cv = self.bridge.imgmsg_to_cv2(image_ir, "mono16")
            # image_rgb, image_ir = self.process(rgb_image_cv,thm_image_cv,odom) 
            # self.image_pub_rgb.publish(self.bridge.cv2_to_imgmsg(image_rgb,"rgb8"))
            self.image_pub_rgb.publish(image_rgb)
            self.image_pub_ir.publish(image_ir)
            self.odom_pub.publish(odom)
            # self.image_pub_ir.publish(self.bridge.cv2_to_imgmsg(image_ir,"mono8"))
            self.count+=1
            # print("Sync FPS:", self.count/(rospy.get_time()-self.start_time))
            self.rate.sleep()
        except CvBridgeError as e:
            print(e)
            
    def process(self, image_rgb,image_ir, odom):
        image_rgb_processed = image_rgb
        image_ir_processed = cv2.cvtColor(image_ir,cv2.COLOR_GRAY2RGB)
        image_ir_processed = cv2.resize(image_ir_processed,(image_rgb_processed.shape[1],image_rgb_processed.shape[0]))
        image_rgb_processed = cv2.hconcat([image_rgb_processed,image_ir_processed])
        
        # self.image_rgb_list.append(image_rgb_processed)
        # self.image_thm_list.append(image_ir_processed)
        # self.odom_list.append(odom)
        # print(odom)
        # with open(str(self.count)+".pkl") as f:
        #     pickle.dump(odom.pose,f)
        return image_rgb_processed,image_ir_processed
    def shutdown(self):
        c = 0
        print(len(self.image_rgb_list),len(self.image_thm_list),len(self.odom_list))
        for i,j,k in zip(self.image_rgb_list,self.image_thm_list,self.odom_list):
            i = cv2.resize(i, (j.shape[1],j.shape[0]))
            j = cv2.merge([j,j,j])
            cv2.imwrite(str(c)+'.jpg',cv2.hconcat([i,j]))
            with open(str(c)+".pkl",'wb') as f:
                pickle.dump(k.pose,f)
            c+=1
def main(args):
    rospy.init_node('image_process', anonymous=True)
    ic = processor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        sys.exit(0)
    # cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

















