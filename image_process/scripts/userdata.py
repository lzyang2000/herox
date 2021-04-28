#!/usr/bin/env python

import cv2
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge, CvBridgeError
from message_filters import TimeSynchronizer, Subscriber, ApproximateTimeSynchronizer, Publisher
import time
import sys
import os
import numpy as np
import pickle
class processor:
    def __init__(self,):
        self.bridge = CvBridge()
        self.image_sub_IR = Subscriber(rospy.get_param('ir_topic'), Image, self.callback)
        self.userdata_pub = Publisher('/rtabmap/user_data', UserData, queue_size=10)
        rospy.on_shutdown(self.shutdown)
        
    def callback(self,image_ir):
        try:
            # rgb_image_cv = self.bridge.imgmsg_to_cv2(image_rgb, "rgb8")
            thm_image_cv = self.bridge.imgmsg_to_cv2(image_ir, "mono8")
            # image_rgb, image_ir = self.process(rgb_image_cv,thm_image_cv,odom) 
            # self.image_pub_rgb.publish(self.bridge.cv2_to_imgmsg(image_rgb,"rgb8"))
            msg = UserData()
            
            # To make it compatible with c++ sub example, use dBm
            
            # Create user data [level, stamp].
            # Any format is accepted.
            # However, if CV_8UC1 format is used, make sure rows > 1 as 
            # rtabmap will think it is already compressed.
            msg.rows = thm_image_cv.shape[0]
            msg.cols = thm_image_cv.shape[1]
            msg.type = 0 # Use OpenCV type (here 6=CV_64FC1): http://ninghang.blogspot.com/2012/11/list-of-mat-type-in-opencv.html
            
            # We should set stamp in data to be able to
            # retrieve it from the saved user data as we need 
            # to get precise position in the graph afterward.
            msg.data = np.array(thm_image_cv,dtype=np.uint8).flatten().tolist()

            self.userdata_pub.publish(msg)
            # self.image_pub_ir.publish(self.bridge.cv2_to_imgmsg(image_ir,"mono8"))
            self.count+=1
            # print("Sync FPS:", self.count/(rospy.get_time()-self.start_time))
        except CvBridgeError as e:
            print(e)
    def shutdown(self):
        print("Shutdown")
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
















