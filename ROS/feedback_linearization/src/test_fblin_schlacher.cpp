#include "feedback_linearization/test_fblin_schlacher.h"

#include <tf/transform_datatypes.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64MultiArray.h>

#include "trajectory.h"
#include "car_msgs/car_cmd.h"

void test_fblin_schlacher::Prepare(void)
{
 RunPeriod = RUN_PERIOD_DEFAULT;

 /* Retrieve parameters from ROS parameter server */
 std::string FullParamName;

 // run_period
 FullParamName = ros::this_node::getName()+"/run_period";

 if (false == Handle.getParam(FullParamName, RunPeriod))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 // Car parameters
 FullParamName = ros::this_node::getName()+"/mass";
 if (false == Handle.getParam(FullParamName, m))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 FullParamName = ros::this_node::getName()+"/yaw_inertia";
 if (false == Handle.getParam(FullParamName, Iz))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 FullParamName = ros::this_node::getName()+"/corn_stiff_front";
 if (false == Handle.getParam(FullParamName, Cf))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 FullParamName = ros::this_node::getName()+"/corn_stiff_rear";
 if (false == Handle.getParam(FullParamName, Cr))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 FullParamName = ros::this_node::getName()+"/cog_dist_front";
 if (false == Handle.getParam(FullParamName, lf))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 FullParamName = ros::this_node::getName()+"/cog_dist_rear";
 if (false == Handle.getParam(FullParamName, lr))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 // Controller parameters
 FullParamName = ros::this_node::getName()+"/speed_thd";
 if (false == Handle.getParam(FullParamName, speed_thd))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str()); 

 FullParamName = ros::this_node::getName()+"/vel_filt_coeff";
 if (false == Handle.getParam(FullParamName, vel_filt_coeff))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str()); 
 else
  vel_filt_order = (unsigned int)vel_filt_coeff.size();

 FullParamName = ros::this_node::getName()+"/car2motor_force";
 if (false == Handle.getParam(FullParamName, car2motor_conversion))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str()); 

 FullParamName = ros::this_node::getName()+"/Kcx";
 if (false == Handle.getParam(FullParamName, Kcx))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 FullParamName = ros::this_node::getName()+"/Tdx";
 if (false == Handle.getParam(FullParamName, Tdx))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 FullParamName = ros::this_node::getName()+"/Nx";
 if (false == Handle.getParam(FullParamName, Nx))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 FullParamName = ros::this_node::getName()+"/Kcy";
 if (false == Handle.getParam(FullParamName, Kcy))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 FullParamName = ros::this_node::getName()+"/Tiy";
 if (false == Handle.getParam(FullParamName, Tiy))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 FullParamName = ros::this_node::getName()+"/Tdy";
 if (false == Handle.getParam(FullParamName, Tdy))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 FullParamName = ros::this_node::getName()+"/Ny";
 if (false == Handle.getParam(FullParamName, Ny))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 FullParamName = ros::this_node::getName()+"/vp_min";
 if (false == Handle.getParam(FullParamName, vp_min))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 FullParamName = ros::this_node::getName()+"/vp_max";
 if (false == Handle.getParam(FullParamName, vp_max))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 // Trajectory parameters
 FullParamName = ros::this_node::getName()+"/delta_vx";
 if (false == Handle.getParam(FullParamName, delta_vx))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str()); 

 FullParamName = ros::this_node::getName()+"/T";
 if (false == Handle.getParam(FullParamName, T))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 FullParamName = ros::this_node::getName()+"/T0";
 if (false == Handle.getParam(FullParamName, T0))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 FullParamName = ros::this_node::getName()+"/A1";
 if (false == Handle.getParam(FullParamName, A1))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str()); 

 FullParamName = ros::this_node::getName()+"/T1";
 if (false == Handle.getParam(FullParamName, T1))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str()); 

 FullParamName = ros::this_node::getName()+"/A2";
 if (false == Handle.getParam(FullParamName, A2))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str()); 

 FullParamName = ros::this_node::getName()+"/T2";
 if (false == Handle.getParam(FullParamName, T2))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str()); 

 FullParamName = ros::this_node::getName()+"/T3";
 if (false == Handle.getParam(FullParamName, T3))
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
 vehiclePose_subscriber = Handle.subscribe("/car/ground_pose", 1, &test_fblin_schlacher::vehiclePose_MessageCallback, this);
 vehicleIMU_subscriber = Handle.subscribe("/imu/data", 1, &test_fblin_schlacher::vehicleIMU_MessageCallback, this);
 telemetry_subscriber = Handle.subscribe("/car_simulator/telemetry", 1, &test_fblin_schlacher::simulated_telemetry_MessageCallback, this);
 radiocmd_subscriber = Handle.subscribe("/radio_cmd", 1, &test_fblin_schlacher::radioCommand_MessageCallback, this);

 controllerCommand_publisher = Handle.advertise<car_msgs::car_cmd>("/controller_cmd", 1);
 vehicleState_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/feedback_linearization/vehicleState", 1);
 controllerState_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/feedback_linearization/controllerState", 1);

 /* Initialize node state */
 _time = 0.0;
 _x0 = _y0 = _theta0 = _vx0 = 0.0;

 _z1 = _z2 = 0.0;

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
 _linearizer = new fblin_schlacher();

 _PDx = _PIDy = NULL;
 _PDx  = new PIDcontrol(Kcx, Tdx, Nx, RunPeriod, vp_min, vp_max);
 _PIDy = new PIDcontrol(Kcy, Tiy, Tdy, Ny, RunPeriod, vp_min, vp_max);

 if (_linearizer)
  _linearizer->set_bicycleParam(m, Iz, lf, lr, Cf, Cr);

 _car_control_state = car_msgs::car_cmd::STATE_SAFE;

 if (use_sim_time)
 {
   if (use_ideal_sim)
   {
     ROS_INFO("Node %s running in simulation mode with simulated measurements.", ros::this_node::getName().c_str());
   }
   else
   {
     ROS_INFO("Node %s running in simulation mode with real measurements.", ros::this_node::getName().c_str());
   }
 }
 else
 {
   if (use_ideal_sim)
   {
     ROS_ERROR("Node %s running with simulated measurements.", ros::this_node::getName().c_str());
   }
   else
   {
     ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
   }
 }
}

void test_fblin_schlacher::RunPeriodically(float Period)
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

void test_fblin_schlacher::Shutdown(void)
{
 if (_linearizer)
 {
   delete _linearizer;
   _linearizer = NULL;
 }

 if (_PDx)
 {
   delete _PDx;
   _PDx = NULL;
 }

 if (_PIDy)
 {
   delete _PIDy;
   _PIDy = NULL;
 }

 ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void test_fblin_schlacher::vehiclePose_MessageCallback(const geometry_msgs::Pose2D::ConstPtr &msg)
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

    if ((_car_control_state == car_msgs::car_cmd::STATE_AUTOMATIC) || (_car_control_state == car_msgs::car_cmd::STATE_MANUAL))
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
  else /* Simulation scenario */
  {
    /* Updating position and heading */
    _vehiclePose.at(0) = msg->x;
    _vehiclePose.at(1) = msg->y;
    _vehiclePose.at(2) = msg->theta;
  }
}

void test_fblin_schlacher::vehicleIMU_MessageCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  if (!use_ideal_sim)
  {
      /* Updating yaw rate */
      _vehicleAngularVelocity = msg->angular_velocity.z;
  }
}

void test_fblin_schlacher::simulated_telemetry_MessageCallback(const car_msgs::simulated_telemetry::ConstPtr& msg)
{
 if (use_ideal_sim)
 {
  _vehicleSideslip        = msg->sideslip;
  _vehicleVelocity.at(0)  = msg->Vx;
  _vehicleVelocity.at(1)  = msg->Vy;
  _vehicleAngularVelocity = msg->yaw_rate;
 }
}

void test_fblin_schlacher::radioCommand_MessageCallback(const car_msgs::car_cmd::ConstPtr& msg)
{
  _car_control_state = msg->state;
}

void test_fblin_schlacher::PeriodicTask(void)
{
  double Fxr, steer, w1, w2;
  double z1ref, z2ref;

  if (_car_control_state == car_msgs::car_cmd::STATE_AUTOMATIC)
  {
    /* Set time variable */
    _time = (ros::Time::now()-_t0).toNSec()*1.0e-9;

    /* Reference trajectory generation */
    if (_time<=T0)
    {
        z1ref = _vx0 = _vehicleVelocity.at(0);
    }
    else if (_time<=T0+T)
    {
        z1ref = _vx0+(3*std::pow(_time-T0,2.0)*T-2*std::pow(_time-T0,3.0))/std::pow(T,3.0)*delta_vx;
    }
    else
    {
        z1ref = _vx0+(3*std::pow(T,2.0)*T-2*std::pow(T,3.0))/std::pow(T,3.0)*delta_vx;
    }

    if (_time<=T1+T0)
    {
        z2ref = 0;
    }
    else if (_time<=T2+T0)
    {
        z2ref = -A1*std::pow(_time-T0-T1,3.0)*std::pow((T2-T1)-(_time-T0-T1),3.0)/std::pow(T2-T1,6.0);
    }
    else if (_time<=T3+T0)
    {
        z2ref = -A2*std::pow(_time-T0-T2,3.0)*std::pow((T3-T2)-(_time-T0-T2),3.0)/std::pow(T3-T2,6.0);
    }
    else
    {
        z2ref = 0;
    }

    /* Position controller / open loop test */
    #ifdef OPEN_LOOP_TEST
      if (_time<1.0)
      {
          w1 = 1.0;
          w2 = 0.0;
      }
      else if (_time<5.0)
      {
          w1 = 5.0;
          w2 = 0.0;
      }
      else
      {
          w1 = 5.0;
          w2 = 1.0;
      }

    #endif
    #ifdef CLOSED_LOOP_TEST
      if (_time<=T0)
      {
        w1 = 0.5;
        w2 = 0.0;
      }
      else
      {
        _PDx->execute(_z1, z1ref, w1);
        _PIDy->execute(_z2, z2ref, w2);
      }
    #endif

    /* Compute feedback linearization */
    if (_linearizer)
    {
      // Update linerization state
      _linearizer->set_bicycleState(std::sqrt(std::pow(_vehicleVelocity.at(0), 2.0)+std::pow(_vehicleVelocity.at(1), 2.0)), _vehicleSideslip, _vehicleAngularVelocity);

      // Compute control signal
      _linearizer->control_transformation(w1, w2, Fxr, steer);

      // Compute output transformation
      _linearizer->ouput_transformation(_z1, _z2);
    }
    else
      ROS_ERROR("Error, no feedback linearization has been activated");
        
    /* Publishing car command values */
    car_msgs::car_cmd msg;
    if (use_sim_time)
    {
      msg.speed_ref = Fxr;
      msg.steer_ref = steer;
    }
    else
    {
      msg.speed_ref = Fxr*car2motor_conversion;
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

    w1 = w2 = 0.0;
    _z1 = _z2 = 0.0;
    Fxr = steer = 0.0;
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

  std_msgs::Float64MultiArray controllerStateMsg;
  controllerStateMsg.data.clear();
  controllerStateMsg.data.push_back(_time);
  controllerStateMsg.data.push_back(w1);
  controllerStateMsg.data.push_back(w2);
  controllerStateMsg.data.push_back(Fxr);
  controllerStateMsg.data.push_back(steer);
  controllerStateMsg.data.push_back(_z1);
  controllerStateMsg.data.push_back(_z2);
  controllerStateMsg.data.push_back(z1ref);
  controllerStateMsg.data.push_back(z2ref);
  controllerState_publisher.publish(controllerStateMsg);
}
