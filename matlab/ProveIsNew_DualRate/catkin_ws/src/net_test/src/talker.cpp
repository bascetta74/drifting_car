#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"

#include <sstream>

#define LOOP_RATE 100 // 100 Hz
#define NUM_POINT 1


int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseArray>("chatter", 1000);

  ros::Rate loop_rate(LOOP_RATE);

  int count = 0;
  while (ros::ok())
  {
    geometry_msgs::PoseArray msg;

    msg.header.seq   = count;
    msg.header.stamp = ros::Time::now();

    for (int k=0; k<NUM_POINT; k++)
    {
      geometry_msgs::Pose pose;
      pose.position.x    = count;
      pose.position.y    = 0;
      pose.position.z    = 0;
      pose.orientation.x = 0;
      pose.orientation.y = 0;
      pose.orientation.z = 0;
      pose.orientation.w = 1;

      msg.poses.push_back(pose);
    }

    ROS_INFO("Message %d sent", msg.header.seq);

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
