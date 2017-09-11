#include "serial_comm/serial_comm.h"

#include <iostream>
#include "std_msgs/Float64.h"


void serial_comm::Prepare(void)
{
	/* Retrieve parameters from ROS parameter server */
	std::string FullParamName;

	FullParamName = ros::this_node::getName()+"/serial_port";
	if (false == Handle.getParam(FullParamName, _serial_port))
         ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

	FullParamName = ros::this_node::getName()+"/baudrate";
	if (false == Handle.getParam(FullParamName, _baudrate))
         ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

	FullParamName = ros::this_node::getName()+"/timeout";
	if (false == Handle.getParam(FullParamName, _timeout))
         ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

	FullParamName = ros::this_node::getName()+"/message_size";
	if (false == Handle.getParam(FullParamName, _message_size))
         ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

	FullParamName = ros::this_node::getName()+"/steer_us_range";
	if (false == Handle.getParam(FullParamName, _steer_us_range))
         ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

	FullParamName = ros::this_node::getName()+"/steer_rad_range";
	if (false == Handle.getParam(FullParamName, _steer_rad_range))
         ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

	FullParamName = ros::this_node::getName()+"/speed_us_range";
	if (false == Handle.getParam(FullParamName, _speed_us_range))
         ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

	FullParamName = ros::this_node::getName()+"/speed_mps_range";
	if (false == Handle.getParam(FullParamName, _speed_mps_range))
         ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

	/* ROS topics */
	controllerCommand_subscriber = Handle.subscribe("/controller_cmd", 1, &serial_comm::controllerCommand_MessageCallback, this);
	radioCommand_publisher = Handle.advertise<car_msgs::car_cmd>("/radio_cmd", 1);
	wheelSpeed_publisher = Handle.advertise<std_msgs::Float64>("/wheel_speed", 1);

        /* Initialize node state */
        _speed_ref = 0.0;
        _steer_ref = 0.0;
        _wheel_speed = 0.0;
        _statemachine.state = SAFE;
        _statemachine.info = 0;

        _message_buffer = new uint8_t[_message_size];

	/* Open the serial port */
	try {
		_serial = new serial::Serial(_serial_port, _baudrate, serial::Timeout::simpleTimeout(_timeout));
	} catch(std::exception & e)
	{
		ROS_ERROR("Node %s: cannot open serial port %s.", ros::this_node::getName().c_str(), _serial_port.c_str());
	}

	ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}


void serial_comm::RunPeriodically(void)
{
 while (ros::ok())
    {
        /* ROS node is timed by serial read from Arduino */
        PeriodicTask();

        ros::spinOnce();
    }
}


void serial_comm::Shutdown(void)
{
    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());

    /* Deleting serial class */
    delete _serial;
    delete _message_buffer;
}


void serial_comm::controllerCommand_MessageCallback(const car_msgs::car_cmd::ConstPtr& msg)
{
    /* Data from controller */
    if (_statemachine.state == AUTOMATIC)
    {
        _speed_ref = msg->speed_ref;
        _steer_ref = msg->steer_ref;
    }
}


void serial_comm::PeriodicTask(void)
{
    /* Manage incoming messages from serial port */
    size_t bytes_read;
    try
    {
        bytes_read = _serial->read(&(_message_buffer[0]), _message_size*sizeof(uint8_t));
    }
    catch(std::exception & e)
    {
        ROS_ERROR("Node %s: cannot read a message from port %s.", ros::this_node::getName().c_str(), _serial_port.c_str());
    }

    switch (_statemachine.state)
    {
     case AUTOMATIC:
     case MANUAL:
      if (_statemachine.state == MANUAL)
       ROS_INFO("Node %s: Arduino in MANUAL mode, do nothing.", ros::this_node::getName().c_str());
      else
       ROS_INFO("Node %s: Arduino in AUTOMATIC mode, do nothing.", ros::this_node::getName().c_str());
      
      /* Verify the message and decode it */
      if (checksum_verify() && (bytes_read >= _message_size))
      {
       /* Steer ref */
       uint16_t tmp_steer_ref;
       memcpy(&tmp_steer_ref, &(_message_buffer[1]), sizeof(uint16_t));
       if (!us_to_SIunits(tmp_steer_ref, _steer_ref, _steer_us_range, _steer_rad_range))
        ROS_ERROR("Node %s: steer ref value in reading serial message is out of range.", ros::this_node::getName().c_str());
       
       /* Speed ref */
       uint16_t tmp_speed_ref;
       memcpy(&tmp_speed_ref, &(_message_buffer[3]), sizeof(uint16_t));
       if (!us_to_SIunits(tmp_speed_ref, _speed_ref, _speed_us_range, _speed_mps_range))
        ROS_ERROR("Node %s: speed ref value in reading serial message is out of range.", ros::this_node::getName().c_str());
       
       /* Wheel speed */
       bool wheel_dx_ccw, wheel_sx_ccw;
       uint16_t  wheel_speed_sx, wheel_speed_dx;
       memcpy(&wheel_speed_dx, &(_message_buffer[5]), sizeof(uint16_t));
       memcpy(&wheel_speed_sx, &(_message_buffer[8]), sizeof(uint16_t));
       wheel_dx_ccw = (_message_buffer[7]==0x00) ? false : true;
       wheel_sx_ccw = (_message_buffer[10]==0x00) ? false : true;
       _wheel_speed = ( ((wheel_dx_ccw) ? static_cast<double>(wheel_speed_dx) : -1.0*static_cast<double>(wheel_speed_dx)) +
       ((wheel_sx_ccw) ? static_cast<double>(wheel_speed_sx) : -1.0*static_cast<double>(wheel_speed_sx)) ) / 2.0;
       
       /* Arduino state */
       switch (_message_buffer[11])
       {
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
       _statemachine.info = _message_buffer[12];
      }
      else
       ROS_ERROR("Node %s: message with wrong checksum or too few bytes from port %s.", ros::this_node::getName().c_str(), _serial_port.c_str());
      break;
      
     case SAFE:
      /* Set commands to zero */
      _speed_ref = _steer_ref = _wheel_speed = 0.0;

      /* Verify the message and decode it */
      if (checksum_verify() && (bytes_read >= _message_size))
      {
       /* Arduino state */
       switch (_message_buffer[11])
       {
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
       _statemachine.info = _message_buffer[12];     
      }
      
      ROS_INFO("Node %s: Arduino in SAFE mode, do nothing.", ros::this_node::getName().c_str());
      break;
      
     case HALT:
      ROS_ERROR("Node %s: Arduino in HALT mode, error code %d.", ros::this_node::getName().c_str(), _statemachine.info);
      serial_comm::Shutdown();
      break;
    }
    
    /* Publish car data on /radio_cmd */
    car_msgs::car_cmd car_msg;
    car_msg.speed_ref = _speed_ref;
    car_msg.steer_ref = _steer_ref;
    switch (_statemachine.state)
    {
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
    std_msgs::Float64 speed_msg;
    speed_msg.data = _wheel_speed;
    wheelSpeed_publisher.publish(speed_msg);

    /* Write a message to the serial port */
    memset(&(_message_buffer[0]), 0, _message_size*sizeof(uint8_t));

    /* Initial code */
    _message_buffer[0] = 0x7E;

    /* Steer ref */
    uint16_t tmp_steer_ref;
    if (!SIunits_to_us(tmp_steer_ref, _steer_ref, _steer_us_range, _steer_rad_range))
        ROS_ERROR("Node %s: steer ref value in writing serial message is out of range.", ros::this_node::getName().c_str());
    memcpy(&(_message_buffer[1]), &tmp_steer_ref, sizeof(uint16_t));

    /* Speed ref */
    uint16_t tmp_speed_ref;
    if (!SIunits_to_us(tmp_speed_ref, _speed_ref, _speed_us_range, _speed_mps_range))
        ROS_ERROR("Node %s: speed ref value in writing serial message is out of range.", ros::this_node::getName().c_str());
    memcpy(&(_message_buffer[3]), &tmp_speed_ref, sizeof(uint16_t));

    /* Arduino state */
    _message_buffer[11] = _statemachine.state;
    _message_buffer[12] = _statemachine.info;

    /* Checksum */
    _message_buffer[_message_size-1] = checksum_calculate();

    size_t bytes_wrote;
    try {
        bytes_wrote = _serial->write(&(_message_buffer[0]), _message_size*sizeof(uint8_t));
    }
    catch(std::exception & e)
    {
        ROS_ERROR("Node %s: cannot write a message to port %s.", ros::this_node::getName().c_str(), _serial_port.c_str());
    }
}


bool serial_comm::checksum_verify()
{
	int result = 0;
	unsigned int sum = 0;

	for (unsigned int i = 0; i < (_message_size - 1); i++)
		sum += _message_buffer[i];
	result = sum & 0xFF;

	if (_message_buffer[_message_size - 1] == result)
		return true;
	else
		return false;
}


char serial_comm::checksum_calculate()
{
  char result = 0;
  unsigned int sum = 0;

  for (unsigned int i = 0; i < (_message_size - 1); i++)
    sum += _message_buffer[i];

  result = sum & 0xFF;

  return result;
}


bool serial_comm::us_to_SIunits(uint16_t value_us, double& value_SIunits, std::vector<int> us_range, std::vector<double> SIunits_range)
{
    if ((value_us<us_range.at(0)) || (value_us>us_range.at(1)))
        return false;
    else
        value_SIunits = static_cast<double>(value_us-us_range.at(0))/static_cast<double>(us_range.at(1)-us_range.at(0))*(SIunits_range.at(1)-SIunits_range.at(0)) + SIunits_range.at(0);

    return true;
}


bool serial_comm::SIunits_to_us(uint16_t& value_us, double value_SIunits, std::vector<int> us_range, std::vector<double> SIunits_range)
{
    if ((value_SIunits<SIunits_range.at(0)) || (value_SIunits>SIunits_range.at(1)))
        return false;
    else
        value_us = static_cast<unsigned int>((value_SIunits-SIunits_range.at(0))/(SIunits_range.at(1)-SIunits_range.at(0))*static_cast<double>(us_range.at(1)-us_range.at(0))) + us_range.at(0);

    return true;
}
