#!/usr/bin/env python

import cv2
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
import time
import sys
import os
import numpy as np
import pickle
import glob

class visualizer:
    def __init__(self,):
        self.count = 0.0
        self.start = True
        self.start_time = 0.0

    
    def process(path):
        TODO
        
        


def main(args):
    visualizer().process('/home/locobot/slam_ws/image_data')


if __name__ == '__main__':
    main(sys.argv)
