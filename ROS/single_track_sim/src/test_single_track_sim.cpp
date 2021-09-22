#include "single_track_sim/test_single_track_sim.h"

#include "car_msgs/car_cmd.h"

void test_single_track_sim::Prepare(void)
{
 RunPeriod = RUN_PERIOD_DEFAULT;

 /* ROS topics */
 vehicleCommand_publisher = Handle.advertise<car_msgs::car_cmd>("/controller_cmd", 1);

 /* Initialize node state */


 ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void test_single_track_sim::RunPeriodically(float Period)
{
 ros::Rate LoopRate(1.0/Period);

 ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0/Period);

 while (ros::ok())
 {
  PeriodicTask();

  ros::spinOnce();

  LoopRate.sleep();
 }
}

void test_single_track_sim::Shutdown(void)
{

 ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void test_single_track_sim::PeriodicTask(void)
{
 /* Vehicle commands */
#ifdef SPEED
 const double car2motor_conversion = 0.195;
 double speed = 5.0*car2motor_conversion;
 double steer = 0.1;

 /* Publishing vehicle commands */
 car_msgs::car_cmd msg;
 msg.speed_ref = speed;
 msg.steer_ref = steer;
 vehicleCommand_publisher.publish(msg);
#endif

#ifdef FORCE
 double force = 5.0;
 double steer = 0.1;

 /* Publishing vehicle commands */
 car_msgs::car_cmd msg;
 msg.speed_ref = force;
 msg.steer_ref = steer;
 vehicleCommand_publisher.publish(msg);
#endif
}
