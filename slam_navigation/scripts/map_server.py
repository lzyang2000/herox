#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
class MapServer():
    def __init__(self):
        self.__map = OccupancyGrid()
        self.__sub_map = rospy.Subscriber('/rtabmap/proj_map', OccupancyGrid, self.__callback_map, queue_size=1)
        self.__pub_occ = rospy.Publisher('/map', OccupancyGrid, queue_size=1)
    
    def __callback_map(self,data):
        self.__map = data
        
    def publish_occupied_map(self):
        # register occupied map for visualization
        self.__map.header.stamp = rospy.Time.now()
        self.__pub_occ.publish(self.__map)

        # publish grid height map for reso convertor 
        


if __name__ == '__main__':
    rospy.init_node("map_server")
    ms = MapServer()
    rate = rospy.Rate(0.3) # 10hz
    while not rospy.is_shutdown():
        ms.publish_occupied_map()
        rate.sleep()
    rospy.spin()