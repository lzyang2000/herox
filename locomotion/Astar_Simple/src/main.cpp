//
// Created by anxing.
//
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <opencv2/opencv.hpp>
#include "Astar.h"
#include "OccMapTransform.h"
#include <gazebo_msgs/ModelStates.h>
#include <tf/tf.h>
#include "Point_3D.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float32MultiArray.h>
using namespace cv;
using namespace std;


//-------------------------------- Global variables ---------------------------------//
// Subscriber
ros::Subscriber map_sub;
ros::Subscriber startPoint_sub;

ros::Subscriber targetPoint_sub;
// Publisher
ros::Publisher mask_pub;
ros::Publisher path_pub;

// Object
nav_msgs::OccupancyGrid OccGridMask;
nav_msgs::Path path;
pathplanning::AstarConfig config;
pathplanning::Astar astar;
OccupancyGridParam OccGridParam;
Point_3D startPoint, targetPoint;


Mat global_Map;
Mat global_Mask;
// Parameter
double InflateRadius;
bool map_flag;
bool startpoint_flag;
bool targetpoint_flag;
bool start_flag;
int rate;
int height = 2021;
int width = 2021;


std::vector<float> height_maps;

float min(float a,float b ){
    if(a<=b)
        return a;
    else
    {
        return b ;
    }
}

float max(float a,float b ){
    if(a<=b)
        return b;
    else
    {
        return a ;
    }
}

void Smooth(nav_msgs::Path& path){
    for(int i=0;i<path.poses.size();i++){
        int num = 5;
        float low = max(0,i-num);
        float high = min(path.poses.size()-1,i+num);
        float x=0,y=0,theta=0,alpha=0;
        for(int j=low;j<=high;j++)
        {
            x += path.poses[j].pose.position.x;
            y += path.poses[j].pose.position.y;
        }
        path.poses[i].pose.position.x = x/(high-low+1);
        path.poses[i].pose.position.y = y/(high-low+1);


        // path[i].x = 0;
        // path[i].y = 0;
        // path[i].theta = theta/(high-low+1);
        // path[i].alpha = alpha/(high-low+1);
        // path[i].x = (path[i].x + path[low].x + path[high].x)/3;
        // path[i].y = (path[i].y + path[low].y + path[high].y)/3;
        // path[i].theta = (path[i].theta + path[low].theta + path[high].theta)/3;
        // path[i].alpha = (path[i].alpha + path[low].alpha + path[high].alpha)/3;
    }
}
//-------------------------------- Callback function ---------------------------------//
void MapCallback(const nav_msgs::OccupancyGrid& msg)
{

    // ROS_INFO("MapCallback");
    // Get parameter
    OccGridParam.GetOccupancyGridParam(msg);

    // Get map
    height = OccGridParam.height;
    width = OccGridParam.width;
    int OccProb;
    float Hight;

    Mat Map(height, width, CV_8UC1);

    for(int i=0;i<height;i++)
    {
        for(int j=0;j<width;j++)
        {

            OccProb = msg.data[i * width + j];
            OccProb = (OccProb < 0) ? 0 : OccProb; // set Unknown to 0
            // The origin of the OccGrid is on the bottom left corner of the map
            Map.at<uchar>(height-i-1, j) = 255 - round(OccProb * 255.0 / 100.0);

    
        }
    }
    // Initial Astar
    Mat Mask;
    config.InflateRadius = round(InflateRadius / OccGridParam.resolution);
    astar.InitAstar(Map, Mask, config);
    global_Map = Map;
    global_Mask = Mask;
    // Publish Mask
    // nav_msgs::OccupancyGrid a;
    // OccGridMask = a;
    OccGridMask.header.stamp = ros::Time::now();
    OccGridMask.header.frame_id = "/map";
    OccGridMask.info = msg.info;
    for(int i=0;i<height;i++)
    {
        for(int j=0;j<width;j++)
        {
            OccProb = Mask.at<uchar>(height-i-1, j) * 255;
            OccGridMask.data.push_back(OccProb);
        }
    }

    // Set flag
    map_flag = true;
    // startpoint_flag = false;
    // targetpoint_flag = false;
}

void curr_state_callback(const nav_msgs::Odometry& msg){
    // ROS_INFO("curr_state: %d",map_flag);
    if(map_flag){
         // ROS_INFO("curr_state!\n");
        
        Point_3D src_point(msg.pose.pose.position.x, msg.pose.pose.position.y);
        double roll, pitch, yaw;
        geometry_msgs::Quaternion q =msg.pose.pose.orientation;
        tf::Quaternion quat(q.x, q.y, q.z, q.w); // x, y, z, w
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        src_point.theta=yaw;
        OccGridParam.Map2ImageTransform(src_point, startPoint);
  

    startpoint_flag = true;
    if(map_flag && startpoint_flag && targetpoint_flag)
    {
        start_flag = true;
    }
    // ROS_INFO("startPoint: %f %f %f %f", msg.data[0], msg.data[1], startPoint.y, startPoint.theta );
      }
}

void TargetPointtCallback(const move_base_msgs::MoveBaseGoal& msg)
{
    // ROS_INFO("TargetPointtCallback: %d",map_flag);
    if (map_flag){
        
    Point_3D src_point(msg.target_pose.pose.position.x, msg.target_pose.pose.position.y);
    OccGridParam.Map2ImageTransform(src_point, targetPoint);

    double roll, pitch, yaw;
    geometry_msgs::Quaternion q =msg.target_pose.pose.orientation;
    tf::Quaternion quat(q.x, q.y, q.z, q.w); // x, y, z, w
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    targetPoint.theta=yaw;

    // Set flag
    targetpoint_flag = true;
    if(map_flag && startpoint_flag && targetpoint_flag)
    {
        start_flag = true;
    }

   ROS_INFO("targetPoint: %f %f %d %d", msg.target_pose.pose.position.x, msg.target_pose.pose.position.y,
            targetPoint.x, targetPoint.y);
}
}

//-------------------------------- Main function ---------------------------------//
int main(int argc, char * argv[])
{
    //  Initial node
    ros::init(argc, argv, "astar");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ROS_INFO("Start astar node!\n");

    // Initial variables
    map_flag = false;
    startpoint_flag = false;
    targetpoint_flag = false;
    start_flag = false;

    // Parameter
    nh_priv.param<bool>("Euclidean", config.Euclidean, true);
    nh_priv.param<int>("OccupyThresh", config.OccupyThresh, -1);
    nh_priv.param<double>("InflateRadius", InflateRadius, -1);
    nh_priv.param<int>("rate", rate, 5);

    // Subscribe topics
    map_sub = nh.subscribe("/map", 10, MapCallback);

    startPoint_sub = nh.subscribe("/correct_odom", 10, curr_state_callback);
    targetPoint_sub = nh.subscribe("/explore/frontier", 10, TargetPointtCallback);
    

    // Advertise topics
    mask_pub = nh.advertise<nav_msgs::OccupancyGrid>("mask", 1);
    path_pub = nh.advertise<nav_msgs::Path>("nav_path", 10);
    
    ros::Publisher global_pub = nh.advertise<std_msgs::Float32MultiArray>("global_path", 1);

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("config_maker_array", 1);
    ros::Publisher marker_people_pub = nh.advertise<visualization_msgs::MarkerArray>("config_maker_people_array", 1);
    ros::Publisher marker_line_pub = nh.advertise<visualization_msgs::MarkerArray>("config_maker_line_array", 1);
    
    std_msgs::Float32MultiArray global_path;
    visualization_msgs::MarkerArray markerarray;

    visualization_msgs::Marker marker;
  
    int marker_id=0;

    // Loop and wait for callback
    ros::Rate loop_rate(rate);
    while(ros::ok())
    {
        if(start_flag)
        {

            int sequ = 0;
            // Start planning path
            vector<Point_3D> PathList;
            
            astar.PathPlanning(startPoint, targetPoint, PathList);
            
            // astar.DrawPath(global_Map, PathList, global_Mask);
            global_path.data.resize(PathList.size()*4);

            if(!PathList.empty())
            {
                
                
                path.header.stamp = ros::Time::now();
                path.header.frame_id = "/map";
                path.poses.clear();

                
                markerarray.markers.clear();
                for(int i=0;i<PathList.size();i++)
                {
                    
                    Point_3D dst_point;
                    OccGridParam.Image2MapTransform(PathList[i], dst_point);
                    geometry_msgs::PoseStamped pose_stamped;
                    pose_stamped.header.seq = sequ;
                    sequ++;
                    pose_stamped.header.stamp = ros::Time::now();
                    pose_stamped.header.frame_id = "/map";
                    pose_stamped.pose.position.x = dst_point.x ;
                    pose_stamped.pose.position.y = dst_point.y ;
                    // pose_stamped.pose.position.x = dst_point.x ;
                    // pose_stamped.pose.position.y = dst_point.y ;
                    pose_stamped.pose.orientation.w = dst_point.theta ;

                    global_path.data[i*4+0]=dst_point.x;
                    global_path.data[i*4+1]=dst_point.y;
                    global_path.data[i*4+2]=dst_point.theta;
                    global_path.data[i*4+3]=dst_point.data_collect;
                    // pose_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, dst_point.theta );
//-------------------------------draw--------------------------------//
                    if (i%1 == 0){
               


                        marker.header.stamp = ros::Time::now();
                        marker.header.frame_id = "/map";
                        marker.id = marker_id;
                        marker.type = visualization_msgs::Marker::CUBE;
                        marker.pose.position.x=dst_point.x ;
                        marker.pose.position.y=dst_point.y ;
                        marker.pose.position.z=0.075;
                        marker.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0, 0, dst_point.theta );
                        // marker.pose.orientation.x = 0.0;
                        // marker.pose.orientation.y = 0.0;
                        // marker.pose.orientation.z = 0.0;
                        // marker.pose.orientation.w = 1.0;
                        marker.scale.x = 0.4;
                        marker.scale.y = 0.2;
                        marker.scale.z = 0.15;
                        marker.color.a= 1.0;
                        marker.color.r = 0;
                        marker.color.g=1;
                        marker.color.b=0;


                       
                        if (i == PathList.size()-1)
                        {
                            markerarray.markers.push_back(marker);
                               
                            // cout<<PathList[i].length<<endl;
                            marker_id++;
                        }
                        
                    }
                    
                    path.poses.push_back(pose_stamped);
                }
                ROS_INFO("%d",sequ);
                // path.header.seq = sequ;
                marker_id=0;
                Smooth(path);
                path_pub.publish(path);
            
                marker_pub.publish(markerarray);
                

                global_pub.publish(global_path);

                ROS_INFO("Find a valid path successfully");
            }
            else
            {
                ROS_ERROR("Can not find a valid path");
            }

            // Set flag
            // start_flag = false;
        }

        if(map_flag)
        {
             // ROS_INFO("431!\n");
            mask_pub.publish(OccGridMask);
        }

        loop_rate.sleep();
        ros::spinOnce();
    }


    return 0;
}
