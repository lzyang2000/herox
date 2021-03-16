#!/usr/bin/env python

import cv2
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from message_filters import TimeSynchronizer, Subscriber, ApproximateTimeSynchronizer
import time
import sys
import os
import numpy as np
class processor:
    def __init__(self,):
        self.bridge = CvBridge()
        self.image_sub_EO = Subscriber(rospy.get_param('rgb_topic'), Image)
        self.image_sub_IR = Subscriber(rospy.get_param('ir_topic'), Image)
        self.sync =  ApproximateTimeSynchronizer([self.image_sub_EO, self.image_sub_IR], 60,rospy.get_param('sync_delta',0.005), allow_headerless=True)
        self.image_pub_rgb = rospy.Publisher("image_process/sync_rgb",Image,queue_size=5)
        self.image_pub_ir = rospy.Publisher("image_process/sync_ir",Image,queue_size=5)
        self.sync.registerCallback(self.callback)
        self.count = 0.0
        self.start = True
        self.start_time = 0.0
        
    def callback(self,image_rgb,image_ir):
        try:
            if self.start:
                self.start_time =  rospy.get_time()
            rgb_image_cv = self.bridge.imgmsg_to_cv2(image_rgb, "rgb8")
            thm_image_cv = self.bridge.imgmsg_to_cv2(image_ir, "mono8")
            image_rgb, image_ir = self.process(rgb_image_cv,thm_image_cv) 
            self.image_pub_rgb.publish(self.bridge.cv2_to_imgmsg(image_rgb,"rgb8"))
            self.image_pub_ir.publish(self.bridge.cv2_to_imgmsg(image_ir,"mono8"))
            self.count+=1
            print("Sync FPS: ", self.count/(rospy.get_time()-self.start_time))
        except CvBridgeError as e:
            print(e)
            
    def process(self, image_rgb,image_ir):
        image_rgb_processed = image_rgb
        image_ir_processed = image_ir
        return image_rgb_processed,image_ir_processed
        
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

















