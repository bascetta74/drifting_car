#include "feedback_linearization/feedback_linearization.h"

#include <tf/transform_datatypes.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
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

 FullParamName = ros::this_node::getName()+"/lowpass_filt_coeff";
 if (false == Handle.getParam(FullParamName, lowpass_filt_coeff))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str()); 
 else
  lowpass_filt_order = (unsigned int)lowpass_filt_coeff.size();

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

 FullParamName = ros::this_node::getName()+"/use_ideal_sim";
 if (false == Handle.getParam(FullParamName, use_ideal_sim))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str()); 

 FullParamName = "/use_sim_time";
 if (false == Handle.getParam(FullParamName, use_sim_time))
  use_sim_time = false;

 /* ROS topics */
 vehiclePose_subscriber = Handle.subscribe("/car/ground_pose", 1, &feedback_linearization::vehiclePose_MessageCallback, this);
 vehicleIMU_subscriber = Handle.subscribe("/imu/data", 1, &feedback_linearization::vehicleIMU_MessageCallback, this);
 telemetry_subscriber = Handle.subscribe("/car_simulator/telemetry", 1, &feedback_linearization::simulated_telemetry_MessageCallback, this);

 controllerCommand_publisher = Handle.advertise<car_msgs::car_cmd>("/controller_cmd", 1);
 vehicleState_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/feedback_linearization/vehicleState", 1);
 pointPact_publisher = Handle.advertise<geometry_msgs::PointStamped>("/feedback_linearization/pointP_actPos", 1);
 pointPref_publisher = Handle.advertise<geometry_msgs::PointStamped>("/feedback_linearization/pointP_refPos", 1);
 pointPvelocity_publisher = Handle.advertise<geometry_msgs::TwistStamped>("/feedback_linearization/pointP_actVel", 1);
 vehicleRef_publisher = Handle.advertise<geometry_msgs::PointStamped>("/feedback_linearization/vehicle_refPos", 1);

 /* Initialize node state */
 _time = 0.0;

 _vehicleSideslip = _vehicleAngularVelocity = 0.0;

 _vehiclePose.assign(3, 0.0);
 _vehicleVelocity.assign(2, 0.0);
 _vehicleAcceleration.assign(2, 0.0);

 _vehiclePositionXBuffer.set_capacity(vel_filt_order);
 std::fill(_vehiclePositionXBuffer.begin(), _vehiclePositionXBuffer.end(), 0.0);
 _vehiclePositionYBuffer.set_capacity(vel_filt_order);
 std::fill(_vehiclePositionYBuffer.begin(), _vehiclePositionYBuffer.end(), 0.0);
 _vehicleHeadingBuffer.set_capacity(lowpass_filt_order);
 std::fill(_vehicleHeadingBuffer.begin(), _vehicleHeadingBuffer.end(), 0.0);
 _vehicleAccelerationXBuffer.set_capacity(lowpass_filt_order);
 std::fill(_vehicleAccelerationXBuffer.begin(), _vehicleAccelerationXBuffer.end(), 0.0);
 _vehicleAccelerationYBuffer.set_capacity(lowpass_filt_order);
 std::fill(_vehicleAccelerationYBuffer.begin(), _vehicleAccelerationYBuffer.end(), 0.0);
 _vehicleYawRateBuffer.set_capacity(lowpass_filt_order);
 std::fill(_vehicleYawRateBuffer.begin(), _vehicleYawRateBuffer.end(), 0.0);

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

 if (use_sim_time)
   ROS_INFO("Node %s running in simulation mode.", ros::this_node::getName().c_str());
 else
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

void feedback_linearization::vehiclePose_MessageCallback(const geometry_msgs::Pose2D::ConstPtr &msg)
{
  if (!use_ideal_sim)
  {
    /* Updating position and heading buffer */
    _vehiclePositionXBuffer.push_back(msg->x);
    _vehiclePositionYBuffer.push_back(msg->y);
    _vehicleHeadingBuffer.push_back(msg->theta + theta_offset);

    /* Compute vehicle cog velocity (vx, vy) through a low-pass differentiator
     * FIR filter */
    _vehicleVelocity.at(0) = 0.0;
    _vehicleVelocity.at(1) = 0.0;

    std::vector<double>::iterator it_coeff = vel_filt_coeff.begin();
    for (boost::circular_buffer<double>::reverse_iterator it_posX = _vehiclePositionXBuffer.rbegin(); it_posX != _vehiclePositionXBuffer.rend(); it_posX++, it_coeff++)
      _vehicleVelocity.at(0) += (*it_coeff) * (*it_posX / RunPeriod);

    it_coeff = vel_filt_coeff.begin();
    for (boost::circular_buffer<double>::reverse_iterator it_posY = _vehiclePositionYBuffer.rbegin(); it_posY != _vehiclePositionYBuffer.rend(); it_posY++, it_coeff++)
      _vehicleVelocity.at(1) += (*it_coeff) * (*it_posY / RunPeriod);

    /* Low-pass filtered vehicle 2D pose */
    _vehiclePose.at(0) = 0.0;
    _vehiclePose.at(1) = 0.0;
    _vehiclePose.at(2) = 0.0;

    it_coeff = lowpass_filt_coeff.begin();
    for (boost::circular_buffer<double>::reverse_iterator it_posX = _vehiclePositionXBuffer.rbegin(); it_posX != _vehiclePositionXBuffer.rend(); it_posX++, it_coeff++)
      _vehiclePose.at(0) += (*it_coeff) * (*it_posX);

    it_coeff = lowpass_filt_coeff.begin();
    for (boost::circular_buffer<double>::reverse_iterator it_posY = _vehiclePositionYBuffer.rbegin(); it_posY != _vehiclePositionYBuffer.rend(); it_posY++, it_coeff++)
      _vehiclePose.at(1) += (*it_coeff) * (*it_posY);

    it_coeff = lowpass_filt_coeff.begin();
    for (boost::circular_buffer<double>::reverse_iterator it_heading = _vehicleHeadingBuffer.rbegin(); it_heading != _vehicleHeadingBuffer.rend(); it_heading++, it_coeff++)
      _vehiclePose.at(2) += (*it_coeff) * (*it_heading);

    /* Vehicle sideslip */
    if (sqrt(pow(_vehicleVelocity.at(0), 2) + pow(_vehicleVelocity.at(1), 2)) > speed_thd)
      _vehicleSideslip = atan2(-_vehicleVelocity.at(0) * sin(_vehiclePose.at(2)) + _vehicleVelocity.at(1) * cos(_vehiclePose.at(2)),
                _vehicleVelocity.at(0) * cos(_vehiclePose.at(2)) + _vehicleVelocity.at(1) * sin(_vehiclePose.at(2)));
    else
      _vehicleSideslip = 0.0;
  }
  else
  {
    /* Updating position and heading */
    _vehiclePose.at(0) = msg->x;
    _vehiclePose.at(1) = msg->y;
    _vehiclePose.at(2) = msg->theta;

  }
}

void feedback_linearization::vehicleIMU_MessageCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  if (!use_ideal_sim)
  {
    /* Updating acceleration and yaw rate buffer */
    _vehicleAccelerationXBuffer.push_back(msg->linear_acceleration.x);
    _vehicleAccelerationYBuffer.push_back(msg->linear_acceleration.y);
    _vehicleYawRateBuffer.push_back(msg->angular_velocity.z);
    
    /* Low-pass filtered vehicle acceleration and yaw */
    _vehicleAcceleration.at(0) = 0.0;
    _vehicleAcceleration.at(1) = 0.0;
    _vehicleAngularVelocity    = 0.0;
    
    std::vector<double>::iterator it_coeff = lowpass_filt_coeff.begin();
    for(boost::circular_buffer<double>::reverse_iterator it_accX = _vehicleAccelerationXBuffer.rbegin(); it_accX != _vehicleAccelerationXBuffer.rend(); it_accX++, it_coeff++)
      _vehicleAcceleration.at(0) += (*it_coeff)*(*it_accX);

    it_coeff = lowpass_filt_coeff.begin();
    for(boost::circular_buffer<double>::reverse_iterator it_accY = _vehicleAccelerationYBuffer.rbegin(); it_accY != _vehicleAccelerationYBuffer.rend(); it_accY++, it_coeff++)
      _vehicleAcceleration.at(1) += (*it_coeff)*(*it_accY);

    it_coeff = lowpass_filt_coeff.begin();
    for(boost::circular_buffer<double>::reverse_iterator it_yawRate = _vehicleYawRateBuffer.rbegin(); it_yawRate != _vehicleYawRateBuffer.rend(); it_yawRate++, it_coeff++)
      _vehicleAngularVelocity += (*it_coeff)*(*it_yawRate);
  }
  else
  {
    /* Updating acceleration and yaw rate */
    _vehicleAcceleration.at(0) = msg->linear_acceleration.x;
    _vehicleAcceleration.at(1) = msg->linear_acceleration.y;
    _vehicleAngularVelocity    = msg->angular_velocity.z;
  }
}

void feedback_linearization::simulated_telemetry_MessageCallback(const car_msgs::simulated_telemetry::ConstPtr& msg)
{
 if (use_ideal_sim)
 {
  _vehicleSideslip       = msg->sideslip;
  _vehicleVelocity.at(0) = msg->Vx;
  _vehicleVelocity.at(1) = msg->Vy;
 }
}

void feedback_linearization::PeriodicTask(void)
{
 /* Reference trajectory generation */
 double xref, yref;
 // Line
 xref = yref = 0.5*_time;
 // Circle
 //xref = -0.5 + 1.0*cos(0.1*_time-0.5*M_PI);
 //yref =  1.0 + 1.0*sin(0.1*_time-0.5*M_PI);

 double xPref, yPref;
 if (_linearizer)
  _linearizer->reference_transformation(xref, yref, xPref, yPref);
 else
  ROS_ERROR("Error, no feedback linearization has been activated");

 /* Actual position update */
 if (_linearizer)
 {
  _linearizer->set_bicycleState(_vehiclePose.at(0), _vehiclePose.at(1), _vehiclePose.at(2), _vehicleSideslip, _vehicleAngularVelocity);
  _linearizer->set_bicycleAbsoluteVelocity(sqrt(pow(_vehicleVelocity.at(0),2)+pow(_vehicleVelocity.at(1),2)));
 }
 else
  ROS_ERROR("Error, no feedback linearization has been activated");

 double xP, yP;
 if (_linearizer)
  _linearizer->ouput_transformation(xP, yP);
 else
  ROS_ERROR("Error, no feedback linearization has been activated");

 /* Position controller / open loop test */
 double vPx, vPy;
 #ifdef OPEN_LOOP_TEST
 vPx = fmin(0.8*_time, 0.4);
 #ifdef SPALIVIERO
 vPy = 0.2/(1.0+exp(-20.0*(_time-1.0)));
 #endif
 #ifdef LOPEZ_I
 vPy = 0.15/(1.0+exp(-20.0*(_time-1.0)));
 #endif
 #ifdef LOPEZ_II
 vPy = 0.5/(1.0+exp(-20.0*(_time-1.0)));
 #endif
 #endif
 #ifdef CLOSED_LOOP_TEST
 vPx = KPx*(xPref-xP);
 vPy = KPy*(yPref-yP);
 #endif

 /* Compute feedback linearization */
 double speed, steer;
 if (_linearizer)
  _linearizer->control_transformation(vPx, vPy, speed, steer);
 else
  ROS_ERROR("Error, no feedback linearization has been activated");

 /* Updating time */
 _time = _time+RunPeriod;

 /* Publishing car command values */
 car_msgs::car_cmd msg;
 if (use_sim_time)
 {
   msg.speed_ref = speed;
   msg.steer_ref = steer;
 }
 else
 {
   msg.speed_ref = speed*cos(_vehicleSideslip);
   msg.steer_ref = steer;
 }
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
 vehicleStateMsg.data.push_back(_vehicleAcceleration.at(0));
 vehicleStateMsg.data.push_back(_vehicleAcceleration.at(1));
 vehicleStateMsg.data.push_back(_vehicleAngularVelocity);
 vehicleState_publisher.publish(vehicleStateMsg);

 geometry_msgs::PointStamped pointPact;
 pointPact.header.stamp = ros::Time(_time);
 pointPact.point.x = xP;
 pointPact.point.y = yP;
 pointPact.point.z = 0;
 pointPact_publisher.publish(pointPact);

 geometry_msgs::PointStamped pointPref;
 pointPref.header.stamp = ros::Time(_time);
 pointPref.point.x = xPref;
 pointPref.point.y = yPref;
 pointPref.point.z = 0;
 pointPref_publisher.publish(pointPref);

 geometry_msgs::PointStamped vehicleRef;
 vehicleRef.header.stamp = ros::Time(_time);
 vehicleRef.point.x = xref;
 vehicleRef.point.y = yref;
 vehicleRef.point.z = 0;
 vehicleRef_publisher.publish(vehicleRef);

 geometry_msgs::TwistStamped pointPvelocity;
 pointPvelocity.header.stamp = ros::Time(_time);
 pointPvelocity.twist.linear.x = vPx;
 pointPvelocity.twist.linear.y = vPy;
 pointPvelocity.twist.linear.z = 0;
 pointPvelocity.twist.angular.x = pointPvelocity.twist.angular.y = pointPvelocity.twist.angular.z = 0;
 pointPvelocity_publisher.publish(pointPvelocity);
}
