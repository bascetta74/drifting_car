#include "serial_comm/serial_comm.h"

#include <iostream>

void serial_comm::Prepare(void) {
  /* Retrieve parameters from ROS parameter server */
  std::string FullParamName;

  FullParamName = ros::this_node::getName() + "/serial_port";
  if (false == Handle.getParam(FullParamName, _serial_port))
    ROS_ERROR("Node %s: unable to retrieve parameter %s.",
              ros::this_node::getName().c_str(), FullParamName.c_str());

  FullParamName = ros::this_node::getName() + "/baudrate";
  if (false == Handle.getParam(FullParamName, _baudrate))
    ROS_ERROR("Node %s: unable to retrieve parameter %s.",
              ros::this_node::getName().c_str(), FullParamName.c_str());

  FullParamName = ros::this_node::getName() + "/timeout";
  if (false == Handle.getParam(FullParamName, _timeout))
    ROS_ERROR("Node %s: unable to retrieve parameter %s.",
              ros::this_node::getName().c_str(), FullParamName.c_str());

  FullParamName = ros::this_node::getName() + "/message_size";
  if (false == Handle.getParam(FullParamName, _message_size))
    ROS_ERROR("Node %s: unable to retrieve parameter %s.",
              ros::this_node::getName().c_str(), FullParamName.c_str());

  FullParamName = ros::this_node::getName() + "/steer_us_range";
  if (false == Handle.getParam(FullParamName, _steer_us_range))
    ROS_ERROR("Node %s: unable to retrieve parameter %s.",
              ros::this_node::getName().c_str(), FullParamName.c_str());

  FullParamName = ros::this_node::getName() + "/steer_rad_range";
  if (false == Handle.getParam(FullParamName, _steer_rad_range))
    ROS_ERROR("Node %s: unable to retrieve parameter %s.",
              ros::this_node::getName().c_str(), FullParamName.c_str());

  FullParamName = ros::this_node::getName() + "/speed_us_range";
  if (false == Handle.getParam(FullParamName, _speed_us_range))
    ROS_ERROR("Node %s: unable to retrieve parameter %s.",
              ros::this_node::getName().c_str(), FullParamName.c_str());

  FullParamName = ros::this_node::getName() + "/speed_mps_range";
  if (false == Handle.getParam(FullParamName, _speed_mps_range))
    ROS_ERROR("Node %s: unable to retrieve parameter %s.",
              ros::this_node::getName().c_str(), FullParamName.c_str());

  /* ROS topics */
  controllerCommand_subscriber = Handle.subscribe("/controller_cmd", 1, &serial_comm::controllerCommand_MessageCallback, this);

  radioCommand_publisher = Handle.advertise<car_msgs::car_cmd>("/radio_cmd", 1);
  wheelSpeed_publisher = Handle.advertise<car_msgs::wheel_spd>("/wheel_speed", 1);

#ifdef TEST_COMM
  controllerCommand_publisher = Handle.advertise<car_msgs::car_cmd>("/test_comm_controller_cmd", 1);
#endif

  /* Initialize node state */
  _speed_ref = 0.0;
  _steer_ref = 0.0;
  _wheel_speed = 0.0;
  _statemachine.state = SAFE;
  _statemachine.info = 0;
  _enteringSafe = _enteringManual = _enteringAutomatic = _enteringHalt = true;

  _message_number = 0;

  /* Open the serial port */
  try {
    _serial = new serial::Serial(_serial_port, _baudrate,
                                 serial::Timeout::simpleTimeout(_timeout));
    _serial->flush();
  } catch (std::exception &e) {
    ROS_ERROR("Node %s: cannot open serial port %s.",
              ros::this_node::getName().c_str(), _serial_port.c_str());
  }
  _telemetry = new telemetryProtocol(_serial, _message_size);

  ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void serial_comm::RunPeriodically(void) {
  while (ros::ok()) {
    /* ROS node is timed by serial read from Arduino */
    PeriodicTask();

    ros::spinOnce();
  }
}

void serial_comm::Shutdown(void) {
  ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());

  /* Close the serial communication */
  try {
    _serial->close();
  } catch (std::exception &e) {
    ROS_ERROR("Node %s: cannot close serial port %s.",
              ros::this_node::getName().c_str(), _serial_port.c_str());
  }

  /* Deleting serial class */
  delete _serial;
  delete _telemetry;
}

void serial_comm::controllerCommand_MessageCallback(
    const car_msgs::car_cmd::ConstPtr &msg) {
  /* Data from controller */
  if (_statemachine.state == AUTOMATIC) {
    _speed_ref = msg->speed_ref;
    _steer_ref = msg->steer_ref;
  }

#ifdef TEST_COMM
  controllerCommand_publisher.publish(msg);
#endif
}

void serial_comm::PeriodicTask(void) {
  // Try to receive a message
  _telemetry->receive();

  if (_telemetry->message_available()) {
    /* If a message is available, read it from serial port */
    _telemetry->read_message(&message);

    /* Run different actions depending on the system state */
    bool wheel_dx_ccw, wheel_sx_ccw;
    unsigned int wheel_sx_speed, wheel_dx_speed;
    uint8_t arduino_state, arduino_state_info, message_number;
    unsigned int steer_cmd, speed_cmd;
    _telemetry->get_message(&message, &steer_cmd, &speed_cmd, &wheel_dx_speed, &wheel_dx_ccw, &wheel_sx_speed, &wheel_sx_ccw,	&arduino_state, &arduino_state_info, &message_number);

    switch (_statemachine.state) {
    case AUTOMATIC:
      /* Writing message to notify state change */
      if (_enteringAutomatic) {
        ROS_INFO("Node %s: Arduino in AUTOMATIC mode.",
                 ros::this_node::getName().c_str());
        _enteringAutomatic = false;

        _enteringManual = _enteringSafe = _enteringHalt = true;
      }

      /* Wheel speed */
      _wheel_speed =
          (((wheel_dx_ccw) ? static_cast<double>(wheel_dx_speed)
                           : -1.0 * static_cast<double>(wheel_dx_speed)) +
           ((wheel_sx_ccw) ? static_cast<double>(wheel_sx_speed)
                           : -1.0 * static_cast<double>(wheel_sx_speed))) /
          2.0;

      /* Arduino state */
      switch (arduino_state) {
      case SAFE:
        _statemachine.state = SAFE;
        break;

      case MANUAL:
        _statemachine.state = MANUAL;
        break;

      case AUTOMATIC:
        _statemachine.state = AUTOMATIC;
        break;

      case HALT:
        _statemachine.state = HALT;
        break;
      }
      _statemachine.info = arduino_state_info;
    break;

    case MANUAL:
      /* Writing message to notify state change */
      if (_enteringManual) {
        ROS_INFO("Node %s: Arduino in MANUAL mode.",
                 ros::this_node::getName().c_str());
        _enteringManual = false;

        _enteringAutomatic = _enteringSafe = _enteringHalt = true;
      }

      /* Steer ref */
      if (!us_to_SIunits(steer_cmd, _steer_ref, _steer_us_range,
                         _steer_rad_range))
        ROS_ERROR(
            "Node %s: steer ref value (%u) in reading serial message is "
            "out of range.",
            ros::this_node::getName().c_str(), (unsigned int)steer_cmd);

      /* Speed ref */
      if (!us_to_SIunits(speed_cmd, _speed_ref, _speed_us_range,
                         _speed_mps_range))
        ROS_ERROR(
            "Node %s: speed ref value (%u) in reading serial message is "
            "out of range.",
            ros::this_node::getName().c_str(), (unsigned int)speed_cmd);

      /* Wheel speed */
      _wheel_speed =
          (((wheel_dx_ccw) ? static_cast<double>(wheel_dx_speed)
                           : -1.0 * static_cast<double>(wheel_dx_speed)) +
           ((wheel_sx_ccw) ? static_cast<double>(wheel_sx_speed)
                           : -1.0 * static_cast<double>(wheel_sx_speed))) /
          2.0;

      /* Arduino state */
      switch (arduino_state) {
      case SAFE:
        _statemachine.state = SAFE;
        break;

      case MANUAL:
        _statemachine.state = MANUAL;
        break;

      case AUTOMATIC:
        _statemachine.state = AUTOMATIC;
        break;

      case HALT:
        _statemachine.state = HALT;
        break;
      }
      _statemachine.info = arduino_state_info;
    break;

    case SAFE:
      /* Writing message on state change */
      if (_enteringSafe) {
        ROS_INFO("Node %s: Arduino in SAFE mode.",
                 ros::this_node::getName().c_str());
        _enteringSafe = false;

        _enteringAutomatic = _enteringManual = _enteringHalt = true;
      }

      /* Set commands to zero */
      _speed_ref = _steer_ref = _wheel_speed = 0.0;

      /* Arduino state */
      switch (arduino_state) {
      case SAFE:
        _statemachine.state = SAFE;
        break;

      case MANUAL:
        _statemachine.state = MANUAL;
        break;

      case AUTOMATIC:
        _statemachine.state = AUTOMATIC;
        break;

      case HALT:
        _statemachine.state = HALT;
        break;
      }
      _statemachine.info = arduino_state_info;
    break;

    case HALT:
      /* Writing message to notify state change */
      if (_enteringHalt) {
        ROS_INFO("Node %s: Arduino in HALT mode.",
                 ros::this_node::getName().c_str());
        _enteringHalt = false;

        _enteringAutomatic = _enteringManual = _enteringSafe = true;
      }

      serial_comm::Shutdown();
      break;
    }

    /* Publish car data on /radio_cmd */
    car_msgs::car_cmd car_msg;
    car_msg.header.seq = _message_number;
    car_msg.header.stamp = ros::Time::now();
    car_msg.speed_ref = _speed_ref;
    car_msg.steer_ref = _steer_ref;
    switch (_statemachine.state) {
    case AUTOMATIC:
      car_msg.state = car_msgs::car_cmd::STATE_AUTOMATIC;
      break;

    case MANUAL:
      car_msg.state = car_msgs::car_cmd::STATE_MANUAL;
      break;

    case SAFE:
      car_msg.state = car_msgs::car_cmd::STATE_SAFE;
      break;

    case HALT:
      car_msg.state = car_msgs::car_cmd::STATE_HALT;
      break;
    }
    radioCommand_publisher.publish(car_msg);

    /* Publish speed on /wheel_speed */
    car_msgs::wheel_spd wheel_speed_msg;
    wheel_speed_msg.header.seq = _message_number;
    wheel_speed_msg.header.stamp = ros::Time::now();
    wheel_speed_msg.speed_fr = wheel_speed_msg.speed_fl =
        wheel_speed_msg.speed_rr = wheel_speed_msg.speed_rl = _wheel_speed;
    wheelSpeed_publisher.publish(wheel_speed_msg);

    /* Write a message to the serial port */

    /* Steer ref */
    if (!SIunits_to_us(steer_cmd, _steer_ref, _steer_us_range,
                       _steer_rad_range))
      ROS_ERROR(
          "Node %s: steer ref value in writing serial message is out of range.",
          ros::this_node::getName().c_str());

    /* Speed ref */
    if (!SIunits_to_us(speed_cmd, _speed_ref, _speed_us_range,
                       _speed_mps_range))
      ROS_ERROR(
          "Node %s: speed ref value in writing serial message is out of range.",
          ros::this_node::getName().c_str());

    _telemetry->set_message(&message, steer_cmd, speed_cmd,	0, false, 0, false, _statemachine.state, _statemachine.info, _message_number);
    if (!_telemetry->send(&message))
      ROS_ERROR("Node %s: cannot write a message to port %s.",
                ros::this_node::getName().c_str(), _serial_port.c_str());
  }
}

bool serial_comm::us_to_SIunits(unsigned int value_us, double &value_SIunits,
                                std::vector<int> us_range,
                                std::vector<double> SIunits_range) {
  if ((value_us < us_range.at(0)) || (value_us > us_range.at(1)))
    return false;
  else
    value_SIunits = static_cast<double>(value_us - us_range.at(0)) /
                        static_cast<double>(us_range.at(1) - us_range.at(0)) *
                        (SIunits_range.at(1) - SIunits_range.at(0)) +
                    SIunits_range.at(0);

  return true;
}

bool serial_comm::SIunits_to_us(unsigned int &value_us, double value_SIunits,
                                std::vector<int> us_range,
                                std::vector<double> SIunits_range) {
  if ((value_SIunits < SIunits_range.at(0)) ||
      (value_SIunits > SIunits_range.at(1)))
    return false;
  else
    value_us = static_cast<unsigned int>(
                   (value_SIunits - SIunits_range.at(0)) /
                   (SIunits_range.at(1) - SIunits_range.at(0)) *
                   static_cast<double>(us_range.at(1) - us_range.at(0))) +
               us_range.at(0);

  return true;
}
