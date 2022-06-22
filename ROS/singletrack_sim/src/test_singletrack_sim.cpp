#include "singletrack_sim/test_singletrack_sim.h"

#include "car_msgs/car_cmd.h"

void test_singletrack_sim::Prepare(void) {
    /* Retrieve parameters from ROS parameter server */
    std::string FullParamName;

    FullParamName = ros::this_node::getName() + "/input_cmd";
    if (false == Handle.getParam(FullParamName, input_cmd))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                  FullParamName.c_str());

    /* ROS topics */
    vehicleCommand_publisher = Handle.advertise<car_msgs::car_cmd>("/controller_cmd", 1);

    /* Initialize node state */
    RunPeriod = RUN_PERIOD_DEFAULT;

    _time = -1.0;

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void test_singletrack_sim::RunPeriodically(float Period) {
    ros::Rate LoopRate(1.0 / Period);

    ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period,
             1.0 / Period);

    while (ros::ok()) {
        PeriodicTask();

        ros::spinOnce();

        LoopRate.sleep();
    }
}

void test_singletrack_sim::Shutdown(void) {

    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void test_singletrack_sim::PeriodicTask(void) {
    /* Update time variable */
    if (_time < 0.0) {
        _t0 = ros::Time::now();
        _time = 0.0;
    }
    else {
        _time = (ros::Time::now()-_t0).toNSec()*1.0e-9;
    }

    /* Vehicle commands */
    if (input_cmd == 0) {
        const double car2motor_conversion = 0.195;
        double speed = (1.5+0.5*_time) * car2motor_conversion;
        double steer = 0.0+std::sin(0.1*_time);

        /* Publishing vehicle commands */
        car_msgs::car_cmd msg;
        msg.speed_ref = speed;
        msg.steer_ref = steer;
        vehicleCommand_publisher.publish(msg);
    } else {
        double force = 1.5+0.5*_time;
        double steer = 0.0+std::sin(0.1*_time);

        /* Publishing vehicle commands */
        car_msgs::car_cmd msg;
        msg.speed_ref = force;
        msg.steer_ref = steer;
        vehicleCommand_publisher.publish(msg);
    }
}
