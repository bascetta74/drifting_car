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
  controllerCommand_subscriber =
      Handle.subscribe("/controller_cmd", 1,
                       &serial_comm::controllerCommand_MessageCallback, this);
  radioCommand_publisher = Handle.advertise<car_msgs::car_cmd>("/radio_cmd", 1);
  wheelSpeed_publisher =
      Handle.advertise<car_msgs::wheel_spd>("/wheel_speed", 1);

#ifdef TEST_COMM
  controllerCommand_publisher =
      Handle.advertise<car_msgs::car_cmd>("/test_comm_controller_cmd", 1);
#endif

  /* Initialize node state */
  _speed_ref = 0.0;
  _steer_ref = 0.0;
  _wheel_speed = 0.0;
  _statemachine.state = SAFE;
  _statemachine.info = 0;
  _enteringSafe = _enteringManual = _enteringAutomatic = _enteringHalt = true;

  _bytes_read = _bytes_wrote = 0;
  _message_number = 0;
  _message_buffer = new uint8_t[_message_size];

  /* Open the serial port */
  try {
    _serial = new serial::Serial(_serial_port, _baudrate,
                                 serial::Timeout::simpleTimeout(_timeout));
    _serial->flush();
  } catch (std::exception &e) {
    ROS_ERROR("Node %s: cannot open serial port %s.",
              ros::this_node::getName().c_str(), _serial_port.c_str());
  }

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
  delete _message_buffer;
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
  if (_serial->available() >= _message_size) {
    /* Read a message from serial port */
    try {
      memset(&(_message_buffer[0]), '\0', _message_size * sizeof(uint8_t));

      _bytes_read =
          _serial->read(&(_message_buffer[0]), _message_size * sizeof(uint8_t));
    } catch (std::exception &e) {
      ROS_ERROR("Node %s: cannot read a message from port %s.",
                ros::this_node::getName().c_str(), _serial_port.c_str());
    }

    /* Run different actions depending on the system state */
    int message_decode_err = 0;
    bool wheel_dx_ccw, wheel_sx_ccw;
    uint16_t wheel_sx_speed, wheel_dx_speed;
    uint16_t arduino_state, arduino_state_info;
    uint16_t steer_cmd, speed_cmd;
    switch (_statemachine.state) {
    case AUTOMATIC:
      /* Writing message to notify state change */
      if (_enteringAutomatic) {
        ROS_INFO("Node %s: Arduino in AUTOMATIC mode.",
                 ros::this_node::getName().c_str());
        _enteringAutomatic = false;

        _enteringManual = _enteringSafe = _enteringHalt = true;
      }

      /* Decode the message */
      if ((message_decode_err = message_decode(
               steer_cmd, speed_cmd, wheel_sx_speed, wheel_dx_speed,
               wheel_sx_ccw, wheel_dx_ccw, arduino_state, arduino_state_info,
               _message_number)) == 0) {

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
      } else {
        switch (message_decode_err) {
        case 1:
          ROS_ERROR("Node %s: message with wrong checksum from port %s.",
                    ros::this_node::getName().c_str(), _serial_port.c_str());
          break;

        case 2:
          ROS_ERROR(
              "Node %s: message with wrong number of bytes (%d instead of %d)"
              "from port %s.",
              ros::this_node::getName().c_str(), (int)_bytes_read,
              (int)_message_size, _serial_port.c_str());
          break;
        }
      }
      break;

    case MANUAL:
      /* Writing message to notify state change */
      if (_enteringManual) {
        ROS_INFO("Node %s: Arduino in MANUAL mode.",
                 ros::this_node::getName().c_str());
        _enteringManual = false;

        _enteringAutomatic = _enteringSafe = _enteringHalt = true;
      }

      /* Decode the message */
      if ((message_decode_err = message_decode(
               steer_cmd, speed_cmd, wheel_sx_speed, wheel_dx_speed,
               wheel_sx_ccw, wheel_dx_ccw, arduino_state, arduino_state_info,
               _message_number)) == 0) {

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
      } else {
        switch (message_decode_err) {
        case 1:
          ROS_ERROR("Node %s: message with wrong checksum from port %s.",
                    ros::this_node::getName().c_str(), _serial_port.c_str());
          break;

        case 2:
          ROS_ERROR(
              "Node %s: message with wrong number of bytes (%d instead of %d)"
              "from port %s.",
              ros::this_node::getName().c_str(), (int)_bytes_read,
              (int)_message_size, _serial_port.c_str());
          break;
        }
      }
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

      /* Decode the message */
      if ((message_decode_err = message_decode(
               steer_cmd, speed_cmd, wheel_sx_speed, wheel_dx_speed,
               wheel_sx_ccw, wheel_dx_ccw, arduino_state, arduino_state_info,
               _message_number)) == 0) {

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
      } else {
        switch (message_decode_err) {
        case 1:
          ROS_ERROR("Node %s: message with wrong checksum from port %s.",
                    ros::this_node::getName().c_str(), _serial_port.c_str());
          break;

        case 2:
          ROS_ERROR(
              "Node %s: message with wrong number of bytes (%d instead of %d)"
              "from port %s.",
              ros::this_node::getName().c_str(), (int)_bytes_read,
              (int)_message_size, _serial_port.c_str());
          break;
        }
      }
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

    message_encode(steer_cmd, speed_cmd, _statemachine.state,
                   _statemachine.info, _message_number);

    size_t _bytes_wrote;
    try {
      _bytes_wrote = _serial->write(&(_message_buffer[0]),
                                    _message_size * sizeof(uint8_t));
    } catch (std::exception &e) {
      ROS_ERROR("Node %s: cannot write a message to port %s.",
                ros::this_node::getName().c_str(), _serial_port.c_str());
    }
  }
}

bool serial_comm::checksum_verify() {
  int result = 0;
  unsigned int sum = 0;

  for (int i = 0; i < (_message_size - 1); i++)
    sum += _message_buffer[i];
  result = sum & 0xFF;

  if (_message_buffer[_message_size - 1] != result)
    return false;

  return true;
}

void serial_comm::checksum_calculate() {
  unsigned int sum = 0;

  for (int i = 0; i < (_message_size - 1); i++)
    sum += _message_buffer[i];

  _message_buffer[_message_size - 1] = sum & 0xFF;
}

int serial_comm::message_decode(uint16_t &steer_cmd, uint16_t &speed_cmd,
                                uint16_t &wheel_sx_speed,
                                uint16_t &wheel_dx_speed, bool &wheel_sx_ccw,
                                bool &wheel_dx_ccw, uint16_t &arduino_state,
                                uint16_t &arduino_state_info,
                                uint16_t &message_num) {
  /* Return 0 if everything is ok, 1 if there is a checksum error, 2 if the
   * number of bytes is wrong */

  if (!checksum_verify()) {
    return 1;
  } else if (_bytes_read != _message_size)
    return 2;
  else {
    /* Steer ref */
    memcpy(&steer_cmd, &(_message_buffer[1]), sizeof(uint16_t));

    /* Speed ref */
    memcpy(&speed_cmd, &(_message_buffer[3]), sizeof(uint16_t));

    /* Wheel speed */
    memcpy(&wheel_dx_speed, &(_message_buffer[5]), sizeof(uint16_t));
    memcpy(&wheel_sx_speed, &(_message_buffer[8]), sizeof(uint16_t));
    wheel_dx_ccw = (_message_buffer[7] == 0x00) ? false : true;
    wheel_sx_ccw = (_message_buffer[10] == 0x00) ? false : true;

    /* Arduino state messages */
    arduino_state = _message_buffer[11];
    arduino_state_info = _message_buffer[12];

    /* Message number */
    message_num = _message_buffer[13];
  }

  return 0;
}

void serial_comm::message_encode(uint16_t steer_cmd, uint16_t speed_cmd,
                                 uint16_t arduino_state,
                                 uint16_t arduino_state_info,
                                 uint16_t message_num) {
  /* Encode a message into the ouput buffer */
  memset(&(_message_buffer[0]), 0, _message_size * sizeof(uint8_t));

  /* Initial code */
  _message_buffer[0] = 0x7E;

  /* Steer command */
  memcpy(&(_message_buffer[1]), &steer_cmd, sizeof(uint16_t));

  /* Speed command */
  memcpy(&(_message_buffer[3]), &speed_cmd, sizeof(uint16_t));

  /* Arduino state */
  _message_buffer[11] = arduino_state;
  _message_buffer[12] = arduino_state_info;

  /* Message number */
  _message_buffer[13] = message_num;

  /* Checksum */
  checksum_calculate();
}

bool serial_comm::us_to_SIunits(uint16_t value_us, double &value_SIunits,
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

bool serial_comm::SIunits_to_us(uint16_t &value_us, double value_SIunits,
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
