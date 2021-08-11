#include "single_track_sim/single_track_sim.h"

#include <sensor_msgs/Imu.h>
#include <rosgraph_msgs/Clock.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float64MultiArray.h>

#include <unistd.h>


void single_track_sim::Prepare(void)
{
 /* Retrieve parameters from ROS parameter server */
 std::string FullParamName;

 // Simulator parameters
 FullParamName = ros::this_node::getName()+"/actuator_model";
 if (false == Handle.getParam(FullParamName, actuator_model))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 FullParamName = ros::this_node::getName()+"/tyre_model";
 if (false == Handle.getParam(FullParamName, tyre_model))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 FullParamName = ros::this_node::getName()+"/dt";
 if (false == Handle.getParam(FullParamName, dt))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 // Vehicle parameters
 FullParamName = ros::this_node::getName()+"/cog_a";
 if (false == Handle.getParam(FullParamName, a))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 FullParamName = ros::this_node::getName()+"/cog_b";
 if (false == Handle.getParam(FullParamName, b))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 FullParamName = ros::this_node::getName()+"/m";
 if (false == Handle.getParam(FullParamName, m))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 FullParamName = ros::this_node::getName()+"/mu";
 if (false == Handle.getParam(FullParamName, mu))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 FullParamName = ros::this_node::getName()+"/Cf";
 if (false == Handle.getParam(FullParamName, Cf))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 FullParamName = ros::this_node::getName()+"/Cr";
 if (false == Handle.getParam(FullParamName, Cr))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 FullParamName = ros::this_node::getName()+"/Iz";
 if (false == Handle.getParam(FullParamName, Iz))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 // Actuator model parameters
 FullParamName = ros::this_node::getName()+"/mu_steer";
 if (false == Handle.getParam(FullParamName, mu_steer))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 FullParamName = ros::this_node::getName()+"/wn_steer";
 if (false == Handle.getParam(FullParamName, wn_steer))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 FullParamName = ros::this_node::getName()+"/csi_steer";
 if (false == Handle.getParam(FullParamName, csi_steer))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 FullParamName = ros::this_node::getName()+"/tau_steer";
 if (false == Handle.getParam(FullParamName, tau_steer))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 // Vehicle initial state
 FullParamName = ros::this_node::getName()+"/r0";
 if (false == Handle.getParam(FullParamName, r0))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 FullParamName = ros::this_node::getName()+"/Vy0";
 if (false == Handle.getParam(FullParamName, Vy0))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 FullParamName = ros::this_node::getName()+"/x0";
 if (false == Handle.getParam(FullParamName, x0))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 FullParamName = ros::this_node::getName()+"/y0";
 if (false == Handle.getParam(FullParamName, y0))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 FullParamName = ros::this_node::getName()+"/psi0";
 if (false == Handle.getParam(FullParamName, psi0))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 /* ROS topics */
 vehicleCommand_subscriber = Handle.subscribe("/controller_cmd", 1, &single_track_sim::vehicleCommand_MessageCallback, this);
 vehiclePose_publisher = Handle.advertise<geometry_msgs::Pose2D>("/car/ground_pose", 1);
 vehicleIMU_publisher = Handle.advertise<sensor_msgs::Imu>("/imu/data", 1);
 vehicleState_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/car/state", 1);
 clock_publisher = Handle.advertise<rosgraph_msgs::Clock>("/clock", 1);

 /* Create simulator class */
 if (actuator_model == 0) {         // ideal
  if (tyre_model == 0) {                // linear
   sim = new single_track_ode(dt, single_track_ode::LINEAR, single_track_ode::IDEAL);
  }
  else if (tyre_model == 1) {           // fiala with saturation
   sim = new single_track_ode(dt, single_track_ode::FIALA_WITH_SATURATION, single_track_ode::IDEAL);
  }
  else if (tyre_model == 2) {           // fiala witout saturation
   sim = new single_track_ode(dt, single_track_ode::FIALA_WITHOUT_SATURATION, single_track_ode::IDEAL);
  }
  else {                                // error
   ROS_ERROR("Node %s: unknown tyre model.", ros::this_node::getName().c_str());
  }
 }
 else if (actuator_model == 1) {    // real
  if (tyre_model == 0) {                // linear
   sim = new single_track_ode(dt, single_track_ode::LINEAR, single_track_ode::REAL);
  }
  else if (tyre_model == 1) {           // fiala with saturation
   sim = new single_track_ode(dt, single_track_ode::FIALA_WITH_SATURATION, single_track_ode::REAL);
  }
  else if (tyre_model == 2) {           // fiala witout saturation
   sim = new single_track_ode(dt, single_track_ode::FIALA_WITHOUT_SATURATION, single_track_ode::REAL);
  }
  else {                                // error
   ROS_ERROR("Node %s: unknown tyre model.", ros::this_node::getName().c_str());
  }
 }
 else {                             // error
  ROS_ERROR("Node %s: unknown actuator model.", ros::this_node::getName().c_str());
 }

 /* Initialize simulator class */
 sim->setInitialState(r0, Vy0, x0, y0, psi0);
 sim->setSteeringActuatorParams(mu_steer, wn_steer, csi_steer, tau_steer);
 sim->setVehicleParams(m, a, b, Cf, Cr, mu, Iz);

 ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void single_track_sim::RunPeriodically(void)
{
 ROS_INFO("Node %s running.", ros::this_node::getName().c_str());

 while (ros::ok())
 {
  PeriodicTask();

  ros::spinOnce();

  usleep(1000);
 }
}

void single_track_sim::Shutdown(void)
{
 // Delete ode object
 delete sim;

 ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void single_track_sim::vehicleCommand_MessageCallback(const car_msgs::car_cmd::ConstPtr& msg)
{
 /*  Set vehicle commands */
 sim->setReferenceCommands(msg->speed_ref, msg->steer_ref);
}

void single_track_sim::PeriodicTask(void)
{
 /*  Integrate the model */
 sim->integrate();

 /*  Extract measurement from simulator */
 double x, y, theta;
 sim->getPose(x, y, theta);
 double ay, yawrate, vy;
 sim->getLateralDynamics(ay, yawrate, vy);
 double sideslip;
 sim->getSideslip(sideslip);
 double slip_front, slip_rear;
 sim->getSlip(slip_front, slip_rear);
 double force_front, force_rear;
 sim->getLateralForce(force_front, force_rear);
 double velocity_act, steer_act;
 sim->getCommands(velocity_act, steer_act);
 double time;
 sim->getTime(time);

 /*  Publish vehicle pose */
 geometry_msgs::Pose2D vehiclePoseMsg;
 vehiclePoseMsg.x = x;
 vehiclePoseMsg.y = y;
 vehiclePoseMsg.theta = theta;
 vehiclePose_publisher.publish(vehiclePoseMsg);

 /*  Publish IMU data */
 sensor_msgs::Imu imuDataMsg;
 imuDataMsg.header.stamp = ros::Time(time);
 imuDataMsg.angular_velocity.x = 0.0;
 imuDataMsg.angular_velocity.y = 0.0;
 imuDataMsg.angular_velocity.z = yawrate;
 imuDataMsg.linear_acceleration.x = 0.0;  // This is not considered in the model (yet)
 imuDataMsg.linear_acceleration.y = ay;
 imuDataMsg.linear_acceleration.z = 0.0;
 vehicleIMU_publisher.publish(imuDataMsg);

 /*  Publish vehicle state */
 std_msgs::Float64MultiArray vehicleStateMsg;
 vehicleStateMsg.data.push_back(x);
 vehicleStateMsg.data.push_back(y);
 vehicleStateMsg.data.push_back(theta);
 vehicleStateMsg.data.push_back(yawrate);
 vehicleStateMsg.data.push_back(vy);
 vehicleStateMsg.data.push_back(ay);
 vehicleStateMsg.data.push_back(sideslip);
 vehicleStateMsg.data.push_back(slip_front);
 vehicleStateMsg.data.push_back(slip_rear);
 vehicleStateMsg.data.push_back(force_front);
 vehicleStateMsg.data.push_back(force_rear);
 vehicleStateMsg.data.push_back(velocity_act);
 vehicleStateMsg.data.push_back(steer_act);
 vehicleState_publisher.publish(vehicleStateMsg);

 /*  Publish clock */
 rosgraph_msgs::Clock clockMsg;
 clockMsg.clock = ros::Time(time);
 clock_publisher.publish(clockMsg);
}
