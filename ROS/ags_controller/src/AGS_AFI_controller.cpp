#include "ags_controller/AGS_AFI_controller.h"

#include <std_msgs/Float64MultiArray.h>
#include "car_msgs/car_cmd.h"


void AGS_AFI_controller::Prepare(void)
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

 FullParamName = ros::this_node::getName()+"/cornering_front";
 if (false == Handle.getParam(FullParamName, Cf))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 // Controller parameters
 std::string matrix_filename;
 FullParamName = ros::this_node::getName()+"/matrix_filename";
 if (false == Handle.getParam(FullParamName, matrix_filename))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 FullParamName = ros::this_node::getName()+"/car2motor_velocity";
 if (false == Handle.getParam(FullParamName, car2motor_conversion))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 FullParamName = ros::this_node::getName()+"/speed_thd";
 if (false == Handle.getParam(FullParamName, speed_thd))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 FullParamName = ros::this_node::getName()+"/vel_filt_coeff";
 if (false == Handle.getParam(FullParamName, vel_filt_coeff))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
 else
  vel_filt_order = (unsigned int)vel_filt_coeff.size();

 FullParamName = ros::this_node::getName()+"/theta_offset";
 if (false == Handle.getParam(FullParamName, theta_offset))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 /* ROS topics */
 vehiclePose_subscriber = Handle.subscribe("/car/ground_pose", 1, &AGS_AFI_controller::vehiclePose_MessageCallback, this);
 vehicleIMU_subscriber = Handle.subscribe("/imu/data", 1, &AGS_AFI_controller::vehicleIMU_MessageCallback, this);
 radiocmd_subscriber = Handle.subscribe("/radio_cmd", 1, &AGS_AFI_controller::radioCommand_MessageCallback, this);

 controllerCommand_publisher = Handle.advertise<car_msgs::car_cmd>("/controller_cmd", 1);
 controllerState_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/controllerState", 1);

 /* Initialize node state */
 _time = 0.0;

 _vehicleSideslip = _vehicleAngularVelocity = _vehicleLongitudinalVelocity = _speedRef = _steerRef = 0.0;

 _vehiclePose.assign(3, 0.0);
 _vehicleVelocity.assign(2, 0.0);

 _vehiclePositionXBuffer.set_capacity(vel_filt_order);
 std::fill(_vehiclePositionXBuffer.begin(), _vehiclePositionXBuffer.end(), 0.0);
 _vehiclePositionYBuffer.set_capacity(vel_filt_order);
 std::fill(_vehiclePositionYBuffer.begin(), _vehiclePositionYBuffer.end(), 0.0);
 _vehiclePositionTimeBuffer.set_capacity(vel_filt_order);
 std::fill(_vehiclePositionTimeBuffer.begin(), _vehiclePositionTimeBuffer.end(), 0.0);

 _car_control_state = car_msgs::car_cmd::STATE_SAFE;

 /* Construct AGS controller object and load controller matrices */
 _AGS_controller = NULL;
 _AGS_controller = new AGS_controller(matrix_filename.c_str(), RunPeriod, 3, Eigen::VectorXd::Zero(4));
 if (_AGS_controller)
 {
     ROS_INFO("Node %s: controller data loaded from %s.", ros::this_node::getName().c_str(), matrix_filename.c_str());
 }
 else
 {
     ROS_ERROR("Node %s: unable to create controller class.", ros::this_node::getName().c_str());
 }
}

void AGS_AFI_controller::RunPeriodically(float Period)
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

void AGS_AFI_controller::Shutdown(void)
{
  // Delete AGS controller object
  if (_AGS_controller)
  {
    delete _AGS_controller;
  }

  ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void AGS_AFI_controller::vehiclePose_MessageCallback(const geometry_msgs::Pose2D::ConstPtr &msg)
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

    /* Vehicle longitudinal velocity */
    _vehicleLongitudinalVelocity = sqrt(pow(_vehicleVelocity.at(0), 2) + pow(_vehicleVelocity.at(1), 2))*cos(_vehicleSideslip);
  }
  else
  {
    // Reset vehicle velocity and sideslip
    _vehicleVelocity.at(0) = _vehicleVelocity.at(1) = 0.0;
    _vehicleSideslip = _vehicleLongitudinalVelocity = 0.0;
  }
}

void AGS_AFI_controller::vehicleIMU_MessageCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  /* Updating yaw rate */
  _vehicleAngularVelocity = msg->angular_velocity.z;
}

void AGS_AFI_controller::radioCommand_MessageCallback(const car_msgs::car_cmd::ConstPtr& msg)
{
  if (_car_control_state != msg->state)
  {
      switch (msg->state)
      {
          case car_msgs::car_cmd::STATE_AUTOMATIC:
              ROS_INFO("Controller switching to AUTOMATIC mode.");
              break;
          case car_msgs::car_cmd::STATE_MANUAL:
              ROS_INFO("Controller switching to MANUAL mode.");
              break;
          case car_msgs::car_cmd::STATE_SAFE:
              ROS_INFO("Car in SAFE mode.");
              break;
          case car_msgs::car_cmd::STATE_HALT:
              ROS_INFO("Car in HALT mode.");
              break;
      }
  }

  _car_control_state = msg->state;
}

void AGS_AFI_controller::PeriodicTask(void)
{
  double xref, yref, xPref, yPref, xP, yP, vPx, vPy;

  if (_car_control_state == car_msgs::car_cmd::STATE_AUTOMATIC)
  {
    /* Set time variable */
    _time = (ros::Time::now()-_t0).toNSec()*1.0e-9;

    /* Reference trajectory generation */
    _speedRef = 7.0*car2motor_conversion;

    double Fyf_ref;
    if (_time <= 2.0)
    {
        Fyf_ref = 0.0;
    }
    else if (_time <= 4.0)
    {
        Fyf_ref = -2.0;
    }
    else if (_time <= 6.0)
    {
        Fyf_ref = 2.0;
    }
    else if (_time <= 8.0)
    {
        Fyf_ref = 0.0;
    }
    else if (_time <= 10.0)
    {
        Fyf_ref = 2.0;
    }
    else if (_time <= 12.0)
    {
        Fyf_ref = -2.0;
    }
    else
    {
        Fyf_ref = 0.0;
    }

    /* Desired damping */
    _dampingRef = 0.7;

    /* Compute AGS controller */
    Eigen::VectorXd Fyf_AGS = Eigen::VectorXd::Zero(1);
    if (_AGS_controller)
    {
        if (_vehicleLongitudinalVelocity > speed_thd)
        {
            Eigen::Vector2d AGS_input(_vehicleSideslip, _vehicleAngularVelocity);
            Eigen::Vector3d AGS_param(1.0/_vehicleLongitudinalVelocity, 1.0/std::pow(_vehicleLongitudinalVelocity, 2.0), _dampingRef);

            _AGS_controller->evaluate(AGS_input, AGS_param);
            _AGS_controller->get_output(Fyf_AGS);
        }
    }
    else
    {
        ROS_ERROR("Node %s: unable run AGS controller.", ros::this_node::getName().c_str());
    }

    /* Compute total lateral force */
    double Fyf = Fyf_ref + Fyf_AGS(0);

    /* Compute AFI transformation */
    if (_vehicleLongitudinalVelocity > speed_thd)
    {
        _steerRef = Fyf/Cf + _vehicleSideslip + a/_vehicleLongitudinalVelocity*_vehicleAngularVelocity;
    }

    /* Publishing vehicle command values */
    car_msgs::car_cmd msg;
    msg.speed_ref = _speedRef;
    msg.steer_ref = _steerRef;
    controllerCommand_publisher.publish(msg);
  }
  else
  {
    _time = 0;
    _t0 = ros::Time::now();
  }
  
  /* Publishing for data logging */
  std_msgs::Float64MultiArray controllerStateMsg;
  controllerStateMsg.data.clear();
  controllerStateMsg.data.push_back(_time);
  controllerStateMsg.data.push_back(_vehiclePose.at(0));
  controllerStateMsg.data.push_back(_vehiclePose.at(1));
  controllerStateMsg.data.push_back(_vehiclePose.at(2));
  controllerStateMsg.data.push_back(_vehicleVelocity.at(0));
  controllerStateMsg.data.push_back(_vehicleVelocity.at(1));
  controllerStateMsg.data.push_back(_vehicleSideslip);
  controllerStateMsg.data.push_back(_vehicleAngularVelocity);
  controllerStateMsg.data.push_back(_vehicleLongitudinalVelocity);
  controllerStateMsg.data.push_back(_speedRef);
  controllerStateMsg.data.push_back(_steerRef);
  controllerState_publisher.publish(controllerStateMsg);
}
