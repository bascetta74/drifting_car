#include "feedback_linearization/test_feedback_linearization_driftcar.h"

#include <tf/transform_datatypes.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64MultiArray.h>

#include "trajectory.h"
#include "car_msgs/car_cmd.h"

void test_feedback_linearization_driftcar::Prepare(void)
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

 FullParamName = ros::this_node::getName()+"/Kp";
 if (false == Handle.getParam(FullParamName, Kp))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str()); 

 FullParamName = ros::this_node::getName()+"/Ti";
 if (false == Handle.getParam(FullParamName, Ti))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str()); 

 FullParamName = ros::this_node::getName()+"/vp_min";
 if (false == Handle.getParam(FullParamName, vp_min))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str()); 

 FullParamName = ros::this_node::getName()+"/vp_max";
 if (false == Handle.getParam(FullParamName, vp_max))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str()); 

 FullParamName = ros::this_node::getName()+"/car2motor_velocity";
 if (false == Handle.getParam(FullParamName, car2motor_conversion))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str()); 

 // Trajectory parameters
 FullParamName = ros::this_node::getName()+"/radius";
 if (false == Handle.getParam(FullParamName, _radius))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str()); 

 FullParamName = ros::this_node::getName()+"/xcenter";
 if (false == Handle.getParam(FullParamName, _xcenter))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str()); 

 FullParamName = ros::this_node::getName()+"/ycenter";
 if (false == Handle.getParam(FullParamName, _ycenter))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str()); 

 FullParamName = ros::this_node::getName()+"/omega";
 if (false == Handle.getParam(FullParamName, _omega))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str()); 

 FullParamName = ros::this_node::getName()+"/const_speed";
 if (false == Handle.getParam(FullParamName, _const_speed))
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
 vehiclePose_subscriber = Handle.subscribe("/car/ground_pose", 1, &test_feedback_linearization_driftcar::vehiclePose_MessageCallback, this);
 vehicleIMU_subscriber = Handle.subscribe("/imu/data", 1, &test_feedback_linearization_driftcar::vehicleIMU_MessageCallback, this);
 telemetry_subscriber = Handle.subscribe("/car_simulator/telemetry", 1, &test_feedback_linearization_driftcar::simulated_telemetry_MessageCallback, this);
 radiocmd_subscriber = Handle.subscribe("/radio_cmd", 1, &test_feedback_linearization_driftcar::radioCommand_MessageCallback, this);

 controllerCommand_publisher = Handle.advertise<car_msgs::car_cmd>("/controller_cmd", 1);
 vehicleState_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/feedback_linearization/vehicleState", 1);
 pointPact_publisher = Handle.advertise<geometry_msgs::PointStamped>("/feedback_linearization/pointP_actPos", 1);
 pointPref_publisher = Handle.advertise<geometry_msgs::PointStamped>("/feedback_linearization/pointP_refPos", 1);
 pointPvelocity_publisher = Handle.advertise<geometry_msgs::TwistStamped>("/feedback_linearization/pointP_actVel", 1);
 vehicleRef_publisher = Handle.advertise<geometry_msgs::PointStamped>("/feedback_linearization/vehicle_refPos", 1);

 /* Initialize node state */
 _time = 0.0;
 _x0 = _y0 = _theta0 = 0.0;

 _vehicleSideslip = _vehicleAngularVelocity = 0.0;

 _vehiclePose.assign(3, 0.0);
 _vehicleVelocity.assign(2, 0.0);

 _vehiclePositionXBuffer.set_capacity(vel_filt_order);
 std::fill(_vehiclePositionXBuffer.begin(), _vehiclePositionXBuffer.end(), 0.0);
 _vehiclePositionYBuffer.set_capacity(vel_filt_order);
 std::fill(_vehiclePositionYBuffer.begin(), _vehiclePositionYBuffer.end(), 0.0);
 _vehiclePositionTimeBuffer.set_capacity(vel_filt_order);
 std::fill(_vehiclePositionTimeBuffer.begin(), _vehiclePositionTimeBuffer.end(), 0.0);

 _linearizer = NULL;
 _linearizer = new fblin_lopez(P_dist,RunPeriod);

 _PIx = _PIy = NULL;
 _PIx = new PIDcontrol(Kp, Ti, RunPeriod, vp_min, vp_max);
 _PIy = new PIDcontrol(Kp, Ti, RunPeriod, vp_min, vp_max);

 if (_linearizer)
  _linearizer->set_bicycleParam(a, b);

 _car_control_state = car_msgs::car_cmd::STATE_SAFE;

 if (use_sim_time)
   ROS_INFO("Node %s running in simulation mode.", ros::this_node::getName().c_str());
 else
   ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void test_feedback_linearization_driftcar::RunPeriodically(float Period)
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

void test_feedback_linearization_driftcar::Shutdown(void)
{
 if (_linearizer)
 {
   delete _linearizer;
   _linearizer = NULL;
 }

 if (_PIx)
 {
   delete _PIx;
   _PIx = NULL;
 }

 if (_PIy)
 {
   delete _PIy;
   _PIy = NULL;
 }

 ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void test_feedback_linearization_driftcar::vehiclePose_MessageCallback(const geometry_msgs::Pose2D::ConstPtr &msg)
{
  if (!use_ideal_sim)
  {
    /* Updating position buffer */
    _vehiclePositionXBuffer.push_back(msg->x);
    _vehiclePositionYBuffer.push_back(msg->y);

    /* Updating time buffer */
    _vehiclePositionTimeBuffer.push_back((ros::Time::now()).toNSec()*1.0e-9);

    /* Updating 2D pose */
    _vehiclePose.at(0) = msg->x;
    _vehiclePose.at(1) = msg->y;
    _vehiclePose.at(2) = msg->theta + theta_offset;

    if (_car_control_state == car_msgs::car_cmd::STATE_AUTOMATIC)
    {
      /* Compute average position sampling time in the last N samples */
      double averagePeriod = (_vehiclePositionTimeBuffer.back()-_vehiclePositionTimeBuffer.front())/_vehiclePositionTimeBuffer.size();

      /* Compute vehicle cog velocity (vx, vy) through a low-pass differentiator FIR filter */
      _vehicleVelocity.at(0) = 0.0;
      _vehicleVelocity.at(1) = 0.0;

      std::vector<double>::iterator it_coeff = vel_filt_coeff.begin();
      for (boost::circular_buffer<double>::reverse_iterator it_posX = _vehiclePositionXBuffer.rbegin(); it_posX != _vehiclePositionXBuffer.rend(); it_posX++, it_coeff++)
        _vehicleVelocity.at(0) += (*it_coeff) * (*it_posX / averagePeriod);

      it_coeff = vel_filt_coeff.begin();
      for (boost::circular_buffer<double>::reverse_iterator it_posY = _vehiclePositionYBuffer.rbegin(); it_posY != _vehiclePositionYBuffer.rend(); it_posY++, it_coeff++)
        _vehicleVelocity.at(1) += (*it_coeff) * (*it_posY / averagePeriod);

      /* Vehicle sideslip */
      if (sqrt(pow(_vehicleVelocity.at(0), 2) + pow(_vehicleVelocity.at(1), 2)) > speed_thd)
        _vehicleSideslip = atan2(-_vehicleVelocity.at(0) * sin(_vehiclePose.at(2)) + _vehicleVelocity.at(1) * cos(_vehiclePose.at(2)),
                _vehicleVelocity.at(0) * cos(_vehiclePose.at(2)) + _vehicleVelocity.at(1) * sin(_vehiclePose.at(2)));
      else
        _vehicleSideslip = 0.0;
    }
    else
    {
      // Reset vehicle velocity and sideslip
      _vehicleVelocity.at(0) = _vehicleVelocity.at(1) = 0.0;
      _vehicleSideslip = 0.0;
    }
  }
  else
  {
    /* Updating position and heading */
    _vehiclePose.at(0) = msg->x;
    _vehiclePose.at(1) = msg->y;
    _vehiclePose.at(2) = msg->theta;

  }
}

void test_feedback_linearization_driftcar::vehicleIMU_MessageCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  /* Updating yaw rate */
  _vehicleAngularVelocity = msg->angular_velocity.z;
}

void test_feedback_linearization_driftcar::simulated_telemetry_MessageCallback(const car_msgs::simulated_telemetry::ConstPtr& msg)
{
 if (use_ideal_sim)
 {
  _vehicleSideslip       = msg->sideslip;
  _vehicleVelocity.at(0) = msg->Vx;
  _vehicleVelocity.at(1) = msg->Vy;
 }
}

void test_feedback_linearization_driftcar::radioCommand_MessageCallback(const car_msgs::car_cmd::ConstPtr& msg)
{
  _car_control_state = msg->state;
}

void test_feedback_linearization_driftcar::PeriodicTask(void)
{
  double xref, yref, xPref, yPref, xP, yP, vPx, vPy;

  if (_car_control_state == car_msgs::car_cmd::STATE_AUTOMATIC)
  {
    /* Set time variable */
    _time = (ros::Time::now()-_t0).toNSec()*1.0e-9;

    /* Reference trajectory generation */
    // Circle
    circle_position(xref, yref, _time, _xcenter, _ycenter, _radius, _omega, 0.5*M_PI);
    
    // Squircle
    //squircle_position(xref, yref, _time, _xcenter, _ycenter, _radius, _omega, 0.5*M_PI);

    if (_linearizer)
      _linearizer->reference_transformation(xref, yref, xPref, yPref);
    else
      ROS_ERROR("Error, no feedback linearization has been activated");
      
    /* Actual position update */
    if (_linearizer)
    {
      _linearizer->set_bicycleState(_vehiclePose.at(0), _vehiclePose.at(1), _vehiclePose.at(2), _vehicleSideslip, _vehicleAngularVelocity);
    }
    else
      ROS_ERROR("Error, no feedback linearization has been activated");
      
    if (_linearizer)
      _linearizer->ouput_transformation(xP, yP);
    else
      ROS_ERROR("Error, no feedback linearization has been activated");
      
    /* Position controller / open loop test */
    #ifdef OPEN_LOOP_TEST
      stepSeq1_velocity(vPx, vPy, _time, _const_speed);
      //stepSeq2_velocity(vPx, vPy, _time, _const_speed);
    #endif
    #ifdef CLOSED_LOOP_TEST
      if (_time<=1.0)
      {
        vPx = -0.5;
        vPy = 0.0;
      }
      else
      {
        _PIx->execute(xP, xPref, vPx);
        _PIy->execute(yP, yPref, vPy);
      }
    #endif

    /* Compute feedback linearization */
    double speed, steer;
    if (_linearizer)
      _linearizer->control_transformation(vPx, vPy, speed, steer);
    else
      ROS_ERROR("Error, no feedback linearization has been activated");
        
    /* Publishing car command values */
    car_msgs::car_cmd msg;
    if (use_sim_time)
    {
      msg.speed_ref = speed;
      msg.steer_ref = steer;
    }
    else
    {
      msg.speed_ref = speed*cos(_vehicleSideslip)*car2motor_conversion;
      msg.steer_ref = steer;
    }
    controllerCommand_publisher.publish(msg);
  }
  else
  {
    _time = 0;

    _x0 = _vehiclePose.at(0);
    _y0 = _vehiclePose.at(1);
    _theta0 = _vehiclePose.at(2);

    _t0 = ros::Time::now();
    
    xref = yref = 0;
    xPref = yPref = 0;
    xP = yP = 0;
    vPx = vPy = 0;

    _PIx->resetState();
    _PIy->resetState();
  }
  
  /* Publishing for data logging */
  std_msgs::Float64MultiArray vehicleStateMsg;
  vehicleStateMsg.data.clear();
  vehicleStateMsg.data.push_back(_time);
  vehicleStateMsg.data.push_back(_vehiclePose.at(0));
  vehicleStateMsg.data.push_back(_vehiclePose.at(1));
  vehicleStateMsg.data.push_back(_vehiclePose.at(2));
  vehicleStateMsg.data.push_back(_vehicleVelocity.at(0));
  vehicleStateMsg.data.push_back(_vehicleVelocity.at(1));
  vehicleStateMsg.data.push_back(_vehicleSideslip);
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
