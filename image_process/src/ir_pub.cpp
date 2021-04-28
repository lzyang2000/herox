#include <ros/ros.h>
#include <std_msgs/String.h>
#include <rtabmap_ros/MsgConversion.h>
#include <rtabmap_ros/UserData.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/publisher.h>
#include <sys/socket.h>
#include <linux/wireless.h>
#include <sys/ioctl.h>

// Demo:
// $ roslaunch freenect_launch freenect.launch depth_registration:=true
// $ roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start" user_data_async_topic:=/wifi_signal rtabmapviz:=false rviz:=true
// $ rosrun rtabmap_ros wifi_signal_pub interface:="wlan0"
// $ rosrun rtabmap_ros wifi_signal_sub
// In RVIZ add PointCloud2 "wifi_signals"

// A percentage value that represents the signal quality
// of the network. WLAN_SIGNAL_QUALITY is of type ULONG.
// This member contains a value between 0 and 100. A value
// of 0 implies an actual RSSI signal strength of -100 dbm.
// A value of 100 implies an actual RSSI signal strength of -50 dbm.
// You can calculate the RSSI signal strength value for wlanSignalQuality
// values between 1 and 99 using linear interpolation.
std::string frameId;
ros::Publisher irPub;
ros::Subscriber irSub;
void irCallback(const sensor_msgs::ImageConstPtr &msg)
{
   cv_bridge::CvImagePtr cv_ptr;
   try
   {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
   }
   catch (cv_bridge::Exception &e)
   {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
   }
   ros::Time stamp = ros::Time::now();

   rtabmap_ros::UserData dataMsg;
   dataMsg.header.frame_id = frameId;
   dataMsg.header.stamp = stamp;
   rtabmap_ros::userDataToROS(cv_ptr->image, dataMsg, false);
   irPub.publish<rtabmap_ros::UserData>(dataMsg);
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "ir_pub");

   ros::NodeHandle nh;
   ros::NodeHandle pnh("~");

   double rateHz = 60; // Hz
   frameId = "camera_link";
   pnh.param("frame_id", frameId, frameId);

   irPub = nh.advertise<rtabmap_ros::UserData>("ir_image_pack", 1);
   irSub = nh.subscribe("/flir_boson/image_rect", 10, irCallback);

   ros::spin();

   return 0;
}

// /**
//  * This tutorial demonstrates simple receipt of messages over the ROS system.
//  */
// void chatterCallback(const std_msgs::String::ConstPtr& msg)
// {
//   ROS_INFO("I heard: [%s]", msg->data.c_str());
// }

// int main(int argc, char **argv)
// {
//   /**
//    * The ros::init() function needs to see argc and argv so that it can perform
//    * any ROS arguments and name remapping that were provided at the command line.
//    * For programmatic remappings you can use a different version of init() which takes
//    * remappings directly, but for most command-line programs, passing argc and argv is
//    * the easiest way to do it.  The third argument to init() is the name of the node.
//    *
//    * You must call one of the versions of ros::init() before using any other
//    * part of the ROS system.
//    */
//   ros::init(argc, argv, "listener");

//   /**
//    * NodeHandle is the main access point to communications with the ROS system.
//    * The first NodeHandle constructed will fully initialize this node, and the last
//    * NodeHandle destructed will close down the node.
//    */
//   ros::NodeHandle n;

//   /**
//    * The subscribe() call is how you tell ROS that you want to receive messages
//    * on a given topic.  This invokes a call to the ROS
//    * master node, which keeps a registry of who is publishing and who
//    * is subscribing.  Messages are passed to a callback function, here
//    * called chatterCallback.  subscribe() returns a Subscriber object that you
//    * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
//    * object go out of scope, this callback will automatically be unsubscribed from
//    * this topic.
//    *
//    * The second parameter to the subscribe() function is the size of the message
//    * queue.  If messages are arriving faster than they are being processed, this
//    * is the number of messages that will be buffered up before beginning to throw
//    * away the oldest ones.
//    */
//   ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

//   /**
//    * ros::spin() will enter a loop, pumping callbacks.  With this version, all
//    * callbacks will be called from within this thread (the main one).  ros::spin()
//    * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
//    */
//   ros::spin();

//   return 0;
// }

// static const std::string OPENCV_WINDOW = "Image window";

// class ImageConverter
// {
//   ros::NodeHandle nh_;
//   image_transport::ImageTransport it_;
//   image_transport::Subscriber image_sub_;
//   image_transport::Publisher image_pub_;

// public:
//   ImageConverter()
//     : it_(nh_)
//   {
//     // Subscrive to input video feed and publish output video feed
//     image_sub_ = it_.subscribe("/camera/image_raw", 1,
//       &ImageConverter::imageCb, this);
//     image_pub_ = it_.advertise("/image_converter/output_video", 1);

//     cv::namedWindow(OPENCV_WINDOW);
//   }

//   ~ImageConverter()
//   {
//     cv::destroyWindow(OPENCV_WINDOW);
//   }

//   void imageCb(const sensor_msgs::ImageConstPtr& msg)
//   {
//     cv_bridge::CvImagePtr cv_ptr;
//     try
//     {
//       cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//     }
//     catch (cv_bridge::Exception& e)
//     {
//       ROS_ERROR("cv_bridge exception: %s", e.what());
//       return;
//     }

//     // Draw an example circle on the video stream
//     if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
//       cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

//     // Update GUI Window
//     cv::imshow(OPENCV_WINDOW, cv_ptr->image);
//     cv::waitKey(3);

//     // Output modified video stream
//     image_pub_.publish(cv_ptr->toImageMsg());
//   }
// };

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "image_converter");
//   ImageConverter ic;
//   ros::spin();
//   return 0;
// }