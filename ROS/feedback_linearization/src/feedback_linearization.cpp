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

 FullParamName = ros::this_node::getName()+"/vel_filt_coeff";
 if (false == Handle.getParam(FullParamName, vel_filt_coeff))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str()); 
 else
  vel_filt_order = (unsigned int)vel_filt_coeff.size();

 FullParamName = ros::this_node::getName()+"/KPx";
 if (false == Handle.getParam(FullParamName, KPx))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str()); 

 FullParamName = ros::this_node::getName()+"/KPy";
 if (false == Handle.getParam(FullParamName, KPy))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str()); 

 // Other parameters
 FullParamName = ros::this_node::getName()+"/theta_offset";
 if (false == Handle.getParam(FullParamName, theta_offset))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str()); 

 /* ROS topics */
 vehiclePose_subscriber = Handle.subscribe("/car/ground_pose", 1, &feedback_linearization::vehiclePose_MessageCallback, this);
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

 _pointPposition.push_back(0.0);
 _pointPposition.push_back(0.0);

 _vehicleVelocity.push_back(0.0);
 _vehicleVelocity.push_back(0.0);

 _vehicleAngularVelocity.push_back(0.0);
 _vehicleAngularVelocity.push_back(0.0);
 _vehicleAngularVelocity.push_back(0.0);

 _vehicleAcceleration.push_back(0.0);
 _vehicleAcceleration.push_back(0.0);
 _vehicleAcceleration.push_back(0.0);

 _vehiclePositionXBuffer.set_capacity(vel_filt_order);
 _vehiclePositionYBuffer.set_capacity(vel_filt_order);
 for (int i=0; i<vel_filt_order; i++)
 {
   _vehiclePositionXBuffer.push_back(0.0);
   _vehiclePositionYBuffer.push_back(0.0);
 }

 _linearizer = NULL;

 #ifdef SPALIVIERO
 _linearizer = new fblin_spaliviero(P_dist,speed_thd);
 #endif
 #ifdef LOPEZ_I
 _linearizer = new fblin_lopez_I(P_dist,RunPeriod,speed_thd);
 #endif
 #ifdef LOPEZ_II
 _linearizer = new fblin_lopez_II(P_dist,RunPeriod);
 #endif

 if (_linearizer)
  _linearizer->set_bicycleParam(m, Jz, Cf, Cr, a, b);

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
 if (_linearizer)
 {
   delete _linearizer;
   _linearizer = NULL;
 }

 ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void feedback_linearization::vehiclePose_MessageCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
 /* Updating position buffer */
 _vehiclePositionXBuffer.push_back(msg->x);
 _vehiclePositionYBuffer.push_back(msg->y);

 /* Vehicle cog velocity */
 _vehicleVelocity.at(0) = 0.0;
 _vehicleVelocity.at(1) = 0.0;

 std::vector<double>::iterator it_coeff = vel_filt_coeff.begin();
 for(boost::circular_buffer<double>::reverse_iterator it_posX = _vehiclePositionXBuffer.rbegin(); it_posX != _vehiclePositionXBuffer.rend(); it_posX++, it_coeff++)
  _vehicleVelocity.at(0) += (*it_coeff)*(*it_posX/RunPeriod);

 it_coeff = vel_filt_coeff.begin();
 for(boost::circular_buffer<double>::reverse_iterator it_posY = _vehiclePositionYBuffer.rbegin(); it_posY != _vehiclePositionYBuffer.rend(); it_posY++, it_coeff++)
  _vehicleVelocity.at(1) += (*it_coeff)*(*it_posY/RunPeriod);

 /* Vehicle 2D pose */
 _vehiclePose.at(0) = msg->x;
 _vehiclePose.at(1) = msg->y;
 _vehiclePose.at(2) = msg->theta+theta_offset;

 /* Vehicle sideslip */
 _vehicleSideslip = atan2( -_vehicleVelocity.at(0)*sin(_vehiclePose.at(2))+_vehicleVelocity.at(1)*cos(_vehiclePose.at(2)),
                            _vehicleVelocity.at(0)*cos(_vehiclePose.at(2))+_vehicleVelocity.at(1)*sin(_vehiclePose.at(2)) );
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
 /* Reference trajectory generation */
 double xref, yref;
 // generate reference

 double xPref, yPref;
 if (_linearizer)
  _linearizer->reference_transformation(xref, yref, xPref, yPref);
 else
  ROS_ERROR("Error, no feedback linearization has been activated");

 /* Position controller */
 double vPx, vPy;
 vPx = 0.0; //KPx*(xPref-_pointPposition.at(0));
 vPy = 0.0; //KPy*(yPref-_pointPposition.at(1));

 /* Compute feedback linearization */
 if (_linearizer)
 {
  _linearizer->set_bicycleState(_vehiclePose.at(0), _vehiclePose.at(1), _vehiclePose.at(3), _vehicleSideslip, _vehicleAngularVelocity.at(2));
  _linearizer->set_bicycleAbsoluteVelocity(sqrt(pow(_vehicleVelocity.at(0),2)+pow(_vehicleVelocity.at(1),2)));
 }
 else
  ROS_ERROR("Error, no feedback linearization has been activated");

 double speed, steer, xP, yP;
 if (_linearizer)
 {
  _linearizer->control_transformation(vPx, vPy, speed, steer);
  _linearizer->ouput_transformation(xP, yP);

  _pointPposition.at(0) = xP;
  _pointPposition.at(1) = yP;
 }
 else
  ROS_ERROR("Error, no feedback linearization has been activated");

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
 vehicleStateMsg.data.push_back(_vehiclePose.at(0));
 vehicleStateMsg.data.push_back(_vehiclePose.at(1));
 vehicleStateMsg.data.push_back(_vehiclePose.at(2));
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
 pointPvelocity_publisher.publish(pointPvelocity);
}
