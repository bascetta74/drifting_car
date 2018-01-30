#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"


void chatterCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  ROS_INFO("I heard message %d at %d:%d", msg->header.seq, msg->header.stamp.sec, msg->header.stamp.nsec);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  ros::spin();

  return 0;
}

