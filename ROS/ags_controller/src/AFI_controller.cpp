#include "ags_controller/AFI_controller.h"

#include <std_msgs/Float64MultiArray.h>

#include "car_msgs/car_cmd.h"

void AFI_controller::Prepare(void) {
    RunPeriod = RUN_PERIOD_DEFAULT;

    /* Retrieve parameters from ROS parameter server */
    std::string FullParamName;

    // run_period
    FullParamName = ros::this_node::getName() + "/run_period";

    if (false == Handle.getParam(FullParamName, RunPeriod))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                  FullParamName.c_str());

    // Car parameters
    FullParamName = ros::this_node::getName() + "/cog_dist_front";
    if (false == Handle.getParam(FullParamName, a))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                  FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/cornering_front";
    if (false == Handle.getParam(FullParamName, Cf))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                  FullParamName.c_str());

    // Controller parameters
    FullParamName = ros::this_node::getName() + "/car2motor_velocity";
    if (false == Handle.getParam(FullParamName, car2motor_conversion))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                  FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/speed_thd";
    if (false == Handle.getParam(FullParamName, speed_thd))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                  FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/vel_filt_coeff";
    if (false == Handle.getParam(FullParamName, vel_filt_coeff))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                  FullParamName.c_str());
    else
        vel_filt_order = (unsigned int) vel_filt_coeff.size();

    FullParamName = ros::this_node::getName() + "/theta_offset";
    if (false == Handle.getParam(FullParamName, theta_offset))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                  FullParamName.c_str());

    // Sideslip estimator parameters
#ifdef VEL_BETA_EST
    FullParamName = ros::this_node::getName() + "/P";
    if (false == Handle.getParam(FullParamName, P))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                  FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/Kpv";
    if (false == Handle.getParam(FullParamName, Kpv))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                  FullParamName.c_str());
#endif
#ifdef ACC_BETA_EST
    FullParamName = ros::this_node::getName() + "/Kpa";
    if (false == Handle.getParam(FullParamName, Kpa))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                  FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/Kda";
    if (false == Handle.getParam(FullParamName, Kda))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                  FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/Ta";
    if (false == Handle.getParam(FullParamName, Ta))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                  FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/v_thd";
    if (false == Handle.getParam(FullParamName, v_thd))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                  FullParamName.c_str());
#endif

    /* ROS topics */
    vehiclePose_subscriber = Handle.subscribe("/car/ground_pose", 1, &AFI_controller::vehiclePose_MessageCallback,
                                              this);
    vehicleIMU_subscriber = Handle.subscribe("/imu/data", 1, &AFI_controller::vehicleIMU_MessageCallback, this);
    radiocmd_subscriber = Handle.subscribe("/radio_cmd", 1, &AFI_controller::radioCommand_MessageCallback, this);

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

    /* Construct sideslip estimator object */
#ifdef VEL_BETA_EST
    _timePose = 0.0;
    _sideslip_estimator = NULL;

    _sideslip_estimator = new velocity_sideslip_estimator(P, Kpv);
#endif
#ifdef ACC_BETA_EST
    _timePose = 0.0;
    _sideslip_estimator = NULL;

    _sideslip_estimator = new acceleration_sideslip_estimator(Kpa, Kda, Ta, v_thd);
#endif
}

void AFI_controller::RunPeriodically(float Period) {
    ros::Rate LoopRate(1.0 / Period);

    ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period,
             1.0 / Period);

    while (ros::ok()) {
        PeriodicTask();

        ros::spinOnce();

        LoopRate.sleep();
    }
}

void AFI_controller::Shutdown(void) {
#if defined(VEL_BETA_EST) || defined(ACC_BETA_EST)
    // Delete sideslip estimator object
    if (_sideslip_estimator) {
        delete _sideslip_estimator;
    }
#endif

    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void AFI_controller::vehiclePose_MessageCallback(const geometry_msgs::Pose2D::ConstPtr &msg) {
#if defined(VEL_BETA_EST) || defined(ACC_BETA_EST)
    /* Updating time pose */
    if (_timePose == 0.0) {
        _tPose0 = ros::Time::now();
    }
    else {
        _timePose = (ros::Time::now() - _tPose0).toNSec() * 1.0e-9;
    }
#endif

    /* Updating position buffer */
    _vehiclePositionXBuffer.push_back(msg->x);
    _vehiclePositionYBuffer.push_back(msg->y);

    /* Updating time buffer */
    _vehiclePositionTimeBuffer.push_back((ros::Time::now()).toNSec() * 1.0e-9);

    /* Updating 2D pose */
    _vehiclePose.at(0) = msg->x;
    _vehiclePose.at(1) = msg->y;
    _vehiclePose.at(2) = msg->theta + theta_offset;

    if (_car_control_state == car_msgs::car_cmd::STATE_AUTOMATIC) {
        /* Compute average position sampling time in the last N samples */
        double averagePeriod = (_vehiclePositionTimeBuffer.back() - _vehiclePositionTimeBuffer.front()) /
                               (_vehiclePositionTimeBuffer.size() - 1);

        /* Compute vehicle cog velocity (vx, vy) through a low-pass differentiator FIR filter */
        _vehicleVelocity.at(0) = 0.0;
        _vehicleVelocity.at(1) = 0.0;

        std::vector<double>::iterator it_coeff = vel_filt_coeff.begin();
        for (boost::circular_buffer<double>::reverse_iterator it_posX = _vehiclePositionXBuffer.rbegin();
             it_posX != _vehiclePositionXBuffer.rend(); it_posX++, it_coeff++)
            _vehicleVelocity.at(0) += (*it_coeff) * (*it_posX / averagePeriod);

        it_coeff = vel_filt_coeff.begin();
        for (boost::circular_buffer<double>::reverse_iterator it_posY = _vehiclePositionYBuffer.rbegin();
             it_posY != _vehiclePositionYBuffer.rend(); it_posY++, it_coeff++)
            _vehicleVelocity.at(1) += (*it_coeff) * (*it_posY / averagePeriod);

        /* Vehicle sideslip */
#if !defined(VEL_BETA_EST) && !defined(ACC_BETA_EST)
        if (sqrt(pow(_vehicleVelocity.at(0), 2) + pow(_vehicleVelocity.at(1), 2)) > speed_thd)
            _vehicleSideslip = std::atan2(-_vehicleVelocity.at(0) * std::sin(_vehiclePose.at(2)) +
                                          _vehicleVelocity.at(1) * std::cos(_vehiclePose.at(2)),
                                          _vehicleVelocity.at(0) * std::cos(_vehiclePose.at(2)) +
                                          _vehicleVelocity.at(1) * std::sin(_vehiclePose.at(2)));
        else
            _vehicleSideslip = 0.0;
#endif

        /* Vehicle longitudinal velocity */
        _vehicleLongitudinalVelocity =
                std::sqrt(std::pow(_vehicleVelocity.at(0), 2) + std::pow(_vehicleVelocity.at(1), 2)) *
                std::cos(_vehicleSideslip);
    } else {
        // Reset vehicle velocity and sideslip
        _vehicleVelocity.at(0) = _vehicleVelocity.at(1) = 0.0;
        _vehicleSideslip = _vehicleLongitudinalVelocity = 0.0;
    }

#if defined(VEL_BETA_EST) || defined(ACC_BETA_EST)
    _sideslip_estimator->setReferencePositions(_vehiclePose.at(0), _vehiclePose.at(1));
    _sideslip_estimator->integrate(_timePose);
    _sideslip_estimator->getSideslip(_vehicleSideslip, _vehiclePose.at(2));
#endif
}

void AFI_controller::vehicleIMU_MessageCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    /* Updating yaw rate */
    _vehicleAngularVelocity = msg->angular_velocity.z;
}

void AFI_controller::radioCommand_MessageCallback(const car_msgs::car_cmd::ConstPtr &msg) {
    if (_car_control_state != msg->state) {
        switch (msg->state) {
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

void AFI_controller::PeriodicTask(void) {
    double xref, yref, xPref, yPref, xP, yP, vPx, vPy;

    if (_car_control_state == car_msgs::car_cmd::STATE_AUTOMATIC) {
        /* Set time variable */
        _time = (ros::Time::now() - _t0).toNSec() * 1.0e-9;

        /* Reference trajectory generation */
        _speedRef = 4.0 * car2motor_conversion;

        if (_time <= 2.0) {
            _FyfRef = 0.0;
        } else if (_time <= 4.0) {
            _FyfRef = -2.0;
        } else if (_time <= 6.0) {
            _FyfRef = 2.0;
        } else if (_time <= 8.0) {
            _FyfRef = 0.0;
        } else if (_time <= 10.0) {
            _FyfRef = 2.0;
        } else if (_time <= 12.0) {
            _FyfRef = -2.0;
        } else {
            _FyfRef = 0.0;
        }

        /* Compute AFI transformation */
        if (_vehicleLongitudinalVelocity > speed_thd) {
            _steerRef = _FyfRef / Cf + _vehicleSideslip + a / _vehicleLongitudinalVelocity * _vehicleAngularVelocity;
        }

        /* Publishing vehicle command values */
        car_msgs::car_cmd msg;
        msg.speed_ref = _speedRef;
        msg.steer_ref = _steerRef;
        controllerCommand_publisher.publish(msg);
    } else {
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
    controllerStateMsg.data.push_back(_speedRef / car2motor_conversion);
    controllerStateMsg.data.push_back(_steerRef);
    controllerStateMsg.data.push_back(_FyfRef);
    controllerState_publisher.publish(controllerStateMsg);
}
