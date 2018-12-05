#include "ethernet_comm/ethernet_comm.h"

#include <iostream>

void ethernet_comm::Prepare(void) {
  /* Retrieve parameters from ROS parameter server */
  std::string FullParamName;

  FullParamName = ros::this_node::getName() + "/server_ip";
  if (false == Handle.getParam(FullParamName, _server_ip))
    ROS_ERROR("Node %s: unable to retrieve parameter %s.",
              ros::this_node::getName().c_str(), FullParamName.c_str());

  FullParamName = ros::this_node::getName() + "/server_port";
  if (false == Handle.getParam(FullParamName, _server_port))
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
                       &ethernet_comm::controllerCommand_MessageCallback, this);
  radioCommand_publisher = Handle.advertise<car_msgs::car_cmd>("/radio_cmd", 1);
  wheelSpeed_publisher = Handle.advertise<car_msgs::wheel_spd>("/wheel_speed", 1);
  arduinoTelemetry_publisher =
      Handle.advertise<car_msgs::arduino_telemetry>("/arduino_telemetry", 1);

  /* Initialize node state */
  _speed_ref = 0.0;
  _steer_ref = 0.0;
  _wheel_speed = 0.0;
  _statemachine.state = SAFE;
  _statemachine.info = 0;
  _enteringSafe = _enteringManual = _enteringAutomatic = _enteringHalt = true;
  _message_number = 0;

  _message_buffer = new uint8_t[_message_size];

  /* Create a UDP socket */
  if ((_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
    ROS_ERROR("Node %s: cannot create a UDP socket.",
              ros::this_node::getName().c_str());
  }

  memset((char *)&sockaddr_server, 0, sizeof(sockaddr_server));
  sockaddr_server.sin_family = AF_INET;
  sockaddr_server.sin_port = htons(_server_port);

  if (inet_aton(_server_ip.c_str(), &sockaddr_server.sin_addr) == 0) {
    ROS_ERROR("Node %s: inet_aton() failed.",
              ros::this_node::getName().c_str());
  }

  /* Send a message to start the communication with the server */
  sockaddr_len = sizeof(sockaddr_server);
  sprintf((char *)_message_buffer, "%s", "start");
  if (sendto(_socket, (char *)_message_buffer, _message_size, 0,
             (struct sockaddr *)&sockaddr_server, sockaddr_len) == -1) {
    ROS_ERROR("Node %s: start communication message cannot be sent.",
              ros::this_node::getName().c_str());
  }

  /* Receive an ACK from the server */
  memset(&(_message_buffer[0]), 0, _message_size * sizeof(uint8_t));
  if (recvfrom(_socket, _message_buffer, _message_size, 0,
               (struct sockaddr *)&sockaddr_server, &sockaddr_len) == -1) {
    ROS_ERROR("Node %s: ack to start communication message cannot be received.",
              ros::this_node::getName().c_str());
  }

  ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void ethernet_comm::RunPeriodically(void) {
  while (ros::ok()) {
    /* ROS node is timed by serial read from Arduino */
    PeriodicTask();

    ros::spinOnce();
  }
}

void ethernet_comm::Shutdown(void) {
  ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());

  /* Closing UDP socket */
  close(_socket);

  /* Deleting serial class */
  delete _message_buffer;
}

void ethernet_comm::controllerCommand_MessageCallback(
    const car_msgs::car_cmd::ConstPtr &msg) {
  /* Data from controller */
  if (_statemachine.state == AUTOMATIC) {
    _speed_ref = msg->speed_ref;
    _steer_ref = msg->steer_ref;
  }
}

void ethernet_comm::PeriodicTask(void) {
  int message_decode_err = 0;
  bool wheel_dx_ccw, wheel_sx_ccw;
  uint16_t wheel_sx_speed, wheel_dx_speed;
  uint16_t arduino_state, arduino_state_info;
  uint16_t steer_cmd, speed_cmd;

  /* Send a message to Arduino */

  /* Steer ref */
  if (!SIunits_to_us(steer_cmd, _steer_ref, _steer_us_range, _steer_rad_range))
    ROS_ERROR(
        "Node %s: steer ref value in writing ethernet message is out of range.",
        ros::this_node::getName().c_str());

  /* Speed ref */
  if (!SIunits_to_us(speed_cmd, _speed_ref, _speed_us_range, _speed_mps_range))
    ROS_ERROR(
        "Node %s: speed ref value in writing ethernet message is out of range.",
        ros::this_node::getName().c_str());

  message_encode(steer_cmd, speed_cmd, _statemachine.state, _statemachine.info);

  if ((_bytes_wrote =
           sendto(_socket, _message_buffer, _message_size, 0,
                  (struct sockaddr *)&sockaddr_server, sockaddr_len)) == -1) {
    ROS_ERROR("Node %s: cannot write a message to port.",
              ros::this_node::getName().c_str());
  }

  /* Manage incoming messages from the server */
  memset(&(_message_buffer[0]), 0, _message_size * sizeof(uint8_t));

  if ((_bytes_read = recvfrom(_socket, _message_buffer, _message_size,
                              MSG_WAITALL, (struct sockaddr *)&sockaddr_server,
                              &sockaddr_len)) == -1) {
    ROS_ERROR("Node %s: cannot receive a message from the server.",
              ros::this_node::getName().c_str());
  }

  /* Run different actions depending on the system state */
  switch (_statemachine.state) {
  case AUTOMATIC:
    /* Writing message to notify state change */
    if (_enteringAutomatic) {
      ROS_INFO("Node %s: Arduino in AUTOMATIC mode.",
               ros::this_node::getName().c_str());
      _enteringAutomatic = false;
    }

    /* Decode the message */
    if ((message_decode_err = message_decode(
             steer_cmd, speed_cmd, wheel_sx_speed, wheel_dx_speed, wheel_sx_ccw,
             wheel_dx_ccw, arduino_state, arduino_state_info)) == 0) {
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
        ROS_ERROR("Node %s: message with wrong checksum.",
                  ros::this_node::getName().c_str());
        break;

      case 2:
        ROS_ERROR(
            "Node %s: message with wrong number of bytes (%d instead of %d).",
            ros::this_node::getName().c_str(), (int)_bytes_read,
            (int)_message_size);
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
    }

    /* Decode the message */
    if ((message_decode_err = message_decode(
             steer_cmd, speed_cmd, wheel_sx_speed, wheel_dx_speed, wheel_sx_ccw,
             wheel_dx_ccw, arduino_state, arduino_state_info)) == 0) {
      /* Steer ref */
      if (!us_to_SIunits(steer_cmd, _steer_ref, _steer_us_range,
                         _steer_rad_range))
        ROS_ERROR("Node %s: steer ref value in reading ethernet message is out "
                  "of range.",
                  ros::this_node::getName().c_str());

      /* Speed ref */
      if (!us_to_SIunits(speed_cmd, _speed_ref, _speed_us_range,
                         _speed_mps_range))
        ROS_ERROR("Node %s: speed ref value in reading ethernet message is out "
                  "of range.",
                  ros::this_node::getName().c_str());

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
        ROS_ERROR("Node %s: message with wrong checksum.",
                  ros::this_node::getName().c_str());
        break;

      case 2:
        ROS_ERROR(
            "Node %s: message with wrong number of bytes (%d instead of %d).",
            ros::this_node::getName().c_str(), (int)_bytes_read,
            (int)_message_size);
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
    }

    /* Set commands to zero */
    _speed_ref = _steer_ref = _wheel_speed = 0.0;

    /* Decode the message */
    if ((message_decode_err = message_decode(
             steer_cmd, speed_cmd, wheel_sx_speed, wheel_dx_speed, wheel_sx_ccw,
             wheel_dx_ccw, arduino_state, arduino_state_info)) == 0) {
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
    }
    break;

  case HALT:
    /* Writing message to notify state change */
    if (_enteringHalt) {
      ROS_INFO("Node %s: Arduino in HALT mode.",
               ros::this_node::getName().c_str());
      _enteringHalt = false;
    }

    ethernet_comm::Shutdown();
    break;
  }

  /* Publish car data on /radio_cmd */
  car_msgs::car_cmd car_msg;
  car_msg.header.seq = _message_number++;
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

  /* Publish a telemetry message */
  car_msgs::arduino_telemetry telemetry_msg;
  memcpy(&telemetry_msg.steer_ref, &(_message_buffer[1]), sizeof(uint16_t));
  memcpy(&telemetry_msg.speed_ref, &(_message_buffer[3]), sizeof(uint16_t));
  memcpy(&telemetry_msg.wheel_dx_speed, &(_message_buffer[5]),
         sizeof(uint16_t));
  memcpy(&telemetry_msg.wheel_sx_speed, &(_message_buffer[8]),
         sizeof(uint16_t));
  telemetry_msg.wheel_dx_ccw = _message_buffer[7];
  telemetry_msg.wheel_sx_ccw = _message_buffer[10];
  telemetry_msg.arduino_state = _message_buffer[11];
  ;
  telemetry_msg.arduino_state_info = _message_buffer[12];

  arduinoTelemetry_publisher.publish(telemetry_msg);
}

bool ethernet_comm::checksum_verify() {
  int result = 0;
  uint16_t sum = 0;

  for (unsigned int i = 0; i < (_message_size - 1); i++)
    sum += _message_buffer[i];
  result = sum & 0xFF;

  if (_message_buffer[_message_size - 1] == result)
    return true;
  else
    return false;
}

void ethernet_comm::checksum_calculate() {
  uint16_t sum = 0;

  for (unsigned int i = 0; i < (_message_size - 1); i++)
    sum += _message_buffer[i];

  _message_buffer[_message_size - 1] = sum & 0xFF;
}

int ethernet_comm::message_decode(uint16_t &steer_cmd, uint16_t &speed_cmd,
                                  uint16_t &wheel_sx_speed,
                                  uint16_t &wheel_dx_speed, bool &wheel_sx_ccw,
                                  bool &wheel_dx_ccw, uint16_t &arduino_state,
                                  uint16_t &arduino_state_info) {
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
  }

  return 0;
}

void ethernet_comm::message_encode(uint16_t steer_cmd, uint16_t speed_cmd,
                                   uint16_t arduino_state,
                                   uint16_t arduino_state_info) {
  /* Encode a message into the output buffer */
  memset(&(_message_buffer[0]), 0, _message_size * sizeof(uint8_t));

  /* Initial code */
  _message_buffer[0] = 0x7E;

  /* Steer command */
  memcpy(&(_message_buffer[1]), &steer_cmd, sizeof(uint16_t));

  /* Speed command */
  memcpy(&(_message_buffer[3]), &speed_cmd, sizeof(uint16_t));

  /* Arduino state */
  _message_buffer[11] = _statemachine.state;
  _message_buffer[12] = _statemachine.info;

  /* Checksum */
  checksum_calculate();
}

bool ethernet_comm::us_to_SIunits(uint16_t value_us, double &value_SIunits,
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

bool ethernet_comm::SIunits_to_us(uint16_t &value_us, double value_SIunits,
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
