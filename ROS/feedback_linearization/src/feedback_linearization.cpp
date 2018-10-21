#include "feedback_linearization/feedback_linearization.h"

#include <tf/transform_datatypes.h>

#include <geometry_msgs/Point.h>
#include <std_msgs/Float64MultiArray.h>

#include "car_msgs/car_cmd.h"


void feedback_linearization::Prepare(void)
{
 RunPeriod = RUN_PERIOD_DEFAULT;

 /* Retrieve parameters from ROS parameter server */
 std::string FullParamName;

 // run_period
 FullParamName = ros::this_node::getName()+"/run_period";

 if (false == Handle.getParam(FullParamName, RunPeriod))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 // Car parameters
 FullParamName = ros::this_node::getName()+"/cog_dist_front";
 if (false == Handle.getParam(FullParamName, a))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 FullParamName = ros::this_node::getName()+"/cog_dist_rear";
 if (false == Handle.getParam(FullParamName, b))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 FullParamName = ros::this_node::getName()+"/cornering_front";
 if (false == Handle.getParam(FullParamName, Cf))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 FullParamName = ros::this_node::getName()+"/cornering_rear";
 if (false == Handle.getParam(FullParamName, Cr))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 FullParamName = ros::this_node::getName()+"/yaw_inertia";
 if (false == Handle.getParam(FullParamName, Jz))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 FullParamName = ros::this_node::getName()+"/mass";
 if (false == Handle.getParam(FullParamName, m))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 // Controller parameters
 FullParamName = ros::this_node::getName()+"/P_dist";
 if (false == Handle.getParam(FullParamName, P_dist))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str()); 

 FullParamName = ros::this_node::getName()+"/speed_thd";
 if (false == Handle.getParam(FullParamName, speed_thd))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str()); 

 // Other parameters
 FullParamName = ros::this_node::getName()+"/theta_offset";
 if (false == Handle.getParam(FullParamName, theta_offset))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str()); 

 FullParamName = ros::this_node::getName()+"/deltas_thd";
 if (false == Handle.getParam(FullParamName, deltas_thd))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str()); 

 /* ROS topics */
 vehiclePose_subscriber = Handle.subscribe("/vehiclePose", 1, &feedback_linearization::vehiclePose_MessageCallback, this);
 vehicleIMU_subscriber = Handle.subscribe("/imu/data", 1, &feedback_linearization::vehicleIMU_MessageCallback, this);
 controllerCommand_publisher = Handle.advertise<car_msgs::car_cmd>("/controller_cmd", 1);
 vehicleState_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/feedback_linearization/vehicleState", 1);
 pointP_publisher = Handle.advertise<geometry_msgs::Point>("/feedback_linearization/pointP", 1);
 pointPvelocity_publisher = Handle.advertise<geometry_msgs::Point>("/feedback_linearization/pointPvelocity", 1);

 /* Initialize node state */
 _time = 0.0;

 _vehicleSideslip = 0.0;

 _vehiclePose.push_back(0.0);
 _vehiclePose.push_back(0.0);
 _vehiclePose.push_back(0.0);

 _vehicleVelocity.push_back(0.0);
 _vehicleVelocity.push_back(0.0);

 _vehicleAngularVelocity.push_back(0.0);
 _vehicleAngularVelocity.push_back(0.0);
 _vehicleAngularVelocity.push_back(0.0);

 _vehicleAcceleration.push_back(0.0);
 _vehicleAcceleration.push_back(0.0);
 _vehicleAcceleration.push_back(0.0);
		
 ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void feedback_linearization::RunPeriodically(float Period)
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

void feedback_linearization::Shutdown(void)
{
 ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void feedback_linearization::vehiclePose_MessageCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
 if ((pow(msg->x-_vehiclePose.at(0),2)+pow(msg->y-_vehiclePose.at(1),2))>deltas_thd*deltas_thd)
 {
  /* Vehicle cog velocity */
  _vehicleVelocity.at(0) = (msg->x-_vehiclePose.at(0))/RunPeriod;
  _vehicleVelocity.at(1) = (msg->y-_vehiclePose.at(1))/RunPeriod;

  /* Vehicle 2D pose */
  _vehiclePose.at(0) = msg->x;
  _vehiclePose.at(1) = msg->y;
  _vehiclePose.at(2) = msg->theta-theta_offset;

  /* Vehicle sideslip */
  _vehicleSideslip = atan2(_vehicleVelocity.at(1),_vehicleVelocity.at(0))-_vehiclePose.at(2);
 }
}

void feedback_linearization::vehicleIMU_MessageCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
 /* Vehicle 2D pose */
 _vehicleAcceleration.at(0) = msg->linear_acceleration.x;
 _vehicleAcceleration.at(1) = msg->linear_acceleration.y;
 _vehicleAcceleration.at(2) = msg->linear_acceleration.z;
 _vehicleAngularVelocity.at(0) = msg->angular_velocity.x;
 _vehicleAngularVelocity.at(1) = msg->angular_velocity.y;
 _vehicleAngularVelocity.at(2) = msg->angular_velocity.z;
}

void feedback_linearization::PeriodicTask(void)
{
 /* Compute point P velocity */
 double vPx, vPy;
 circle_trajectory(vPx, vPy, _time, 2.0, 0.5, P_dist);

 /* Compute feedback linearization */
 double speed, steer;
 fblin_control_singletrack_pointP(vPx, vPy, speed, steer);

 double xP, yP;
 fblin_output_singletrack_pointP(xP, yP);

 /* Updating time */
 _time = _time+RunPeriod;

 /* Publishing car command values */
 car_msgs::car_cmd msg;
 msg.speed_ref = speed;
 msg.steer_ref = steer;
 controllerCommand_publisher.publish(msg);

 /* Publishing for data logging */
 std_msgs::Float64MultiArray vehicleStateMsg;
 vehicleStateMsg.data.clear();
 vehicleStateMsg.data.push_back(_vehicleVelocity.at(0));
 vehicleStateMsg.data.push_back(_vehicleVelocity.at(1));
 vehicleStateMsg.data.push_back(_vehicleSideslip);
 vehicleState_publisher.publish(vehicleStateMsg);

 geometry_msgs::Point pointP;
 pointP.x = xP;
 pointP.y = yP;
 pointP.z = 0;
 pointP_publisher.publish(pointP);

 geometry_msgs::Point pointPvelocity;
 pointPvelocity.x = vPx;
 pointPvelocity.y = vPy;
 pointPvelocity.z = 0;
 pointP_publisher.publish(pointPvelocity);
}


/* Private functions defining feedback linearization laws */
void feedback_linearization::fblin_control_singletrack_pointP(double vPx, double vPy, double& speed, double& steer)
{
 double omega = 1/P_dist*(vPy*cos(_vehicleSideslip+_vehiclePose.at(2))-vPx*sin(_vehicleSideslip+_vehiclePose.at(2)));
 double absoluteSpeed = vPx*cos(_vehicleSideslip+_vehiclePose.at(2))+vPy*sin(_vehicleSideslip+_vehiclePose.at(2));

 if (std::fabs(absoluteSpeed)<speed_thd)
    steer = 0;
 else
    steer = absoluteSpeed*m/Cf*omega+(Cr/Cf+1)*_vehicleSideslip-(Cr/Cf*b-a)*_vehicleAngularVelocity.at(2)/absoluteSpeed;

 speed = absoluteSpeed*cos(_vehicleSideslip);
}

void feedback_linearization::fblin_output_singletrack_pointP(double& xP, double& yP)
{
 xP = _vehiclePose.at(0)+P_dist*cos(_vehicleSideslip+_vehiclePose.at(2));
 yP = _vehiclePose.at(1)+P_dist*sin(_vehicleSideslip+_vehiclePose.at(2));
}

/* Private functions defining point P velocity */
void feedback_linearization::circle_trajectory(double& vPx, double& vPy, double time, double omega, double radius, double Pdist)
{
 double vx = -omega*radius*sin(omega*time);
 double vy = omega*radius*cos(omega*time);

 vPx = vx-Pdist*omega*sin(omega*time);
 vPy = vy+Pdist*omega*cos(omega*time);
}
