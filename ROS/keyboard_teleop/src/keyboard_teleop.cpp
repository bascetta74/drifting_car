#include "keyboard_teleop/keyboard_teleop.h"

#include "car_msgs/car_cmd.h"


void keyboard_teleop::Prepare(void)
{
    RunPeriod = RUN_PERIOD_DEFAULT;

    /* Retrieve parameters from ROS parameter server */
    std::string FullParamName;

    // run_period
    FullParamName = ros::this_node::getName()+"/run_period";
    if (false == Handle.getParam(FullParamName, RunPeriod))
	ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    // delta_steer
    FullParamName = ros::this_node::getName()+"/delta_steer";
    if (false == Handle.getParam(FullParamName, delta_steer))
	ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    // delta_speed
    FullParamName = ros::this_node::getName()+"/delta_speed";
    if (false == Handle.getParam(FullParamName, delta_speed))
	ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    // max_steer
    FullParamName = ros::this_node::getName()+"/max_steer";
    if (false == Handle.getParam(FullParamName, max_steer))
	ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    // max_speed
    FullParamName = ros::this_node::getName()+"/max_speed";
    if (false == Handle.getParam(FullParamName, max_speed))
	ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    /* ROS topics */
    radioCommand_publisher = Handle.advertise<car_msgs::car_cmd>("/radio_cmd", 1);
    keyUp_subscriber = Handle.subscribe("/keyboard/keydown", 1, &keyboard_teleop::keyDown_MessageCallback, this);
    keyDown_subscriber = Handle.subscribe("/keyboard/keyup", 1, &keyboard_teleop::keyUp_MessageCallback, this);

    /* Initialize node state */
    _arrowUp_pressed = _arrowDown_pressed = _arrowLeft_pressed = _arrowRight_pressed = false;
    _speed_ref = _steer_ref = 0.0;
    _state = car_msgs::car_cmd::STATE_SAFE;

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}


void keyboard_teleop::RunPeriodically(float Period)
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


void keyboard_teleop::Shutdown(void)
{
    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}


void keyboard_teleop::keyUp_MessageCallback(const keyboard::Key::ConstPtr& msg)
{
    /* Check if an arrow is released and deactivate the corresponding flag */
    switch (msg->code)
    {
    case keyboard::Key::KEY_UP:
     _arrowUp_pressed = false;
     break;
    case keyboard::Key::KEY_DOWN:
     _arrowDown_pressed = false;
     break;
    case keyboard::Key::KEY_RIGHT:
     _arrowRight_pressed = false;
     break;
    case keyboard::Key::KEY_LEFT:
     _arrowLeft_pressed = false;
     break;
    }
}


void keyboard_teleop::keyDown_MessageCallback(const keyboard::Key::ConstPtr& msg)
{
    /* Check if an arrow is pressed and activate the corresponding flag */
    switch (msg->code)
    {
    case keyboard::Key::KEY_UP:
     _arrowUp_pressed = true;
     break;
    case keyboard::Key::KEY_DOWN:
     _arrowDown_pressed = true;
     break;
    case keyboard::Key::KEY_RIGHT:
     _arrowRight_pressed = true;
     break;
    case keyboard::Key::KEY_LEFT:
     _arrowLeft_pressed = true;
     break;
    }

    /* Check if A key is pressed */
    if (msg->code == keyboard::Key::KEY_a)
    {
	if (_state == car_msgs::car_cmd::STATE_AUTOMATIC)
	{
             _state = car_msgs::car_cmd::STATE_MANUAL;

             /* Reset speed/steer reference */
             _speed_ref = _steer_ref = 0.0;
	}
        else
         _state = car_msgs::car_cmd::STATE_AUTOMATIC;
    }
}


void keyboard_teleop::PeriodicTask(void)
{
    /* Check arrow flags and increase/decrease speed and steer */
    if (_arrowUp_pressed)
	_speed_ref += delta_speed;
    if (_arrowDown_pressed)
	_speed_ref -= delta_speed;
    if (_arrowLeft_pressed)
	_steer_ref += delta_steer;
    if (_arrowRight_pressed)
	_steer_ref -= delta_steer;

    /* Saturate to maximum speed/steer value */
    _speed_ref = fmin(fmax(_speed_ref, -max_speed), max_speed);
    _steer_ref = fmin(fmax(_steer_ref, -max_steer), max_steer);

    /* Publish sideslip value */
    car_msgs::car_cmd msg;
    msg.steer_ref = _steer_ref;
    msg.speed_ref = _speed_ref;
    msg.state     = _state;
    radioCommand_publisher.publish(msg);
}
