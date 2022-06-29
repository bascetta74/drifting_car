#include "singletrack_sim/singletrack_beta_sim.h"

#include <sensor_msgs/Imu.h>
#include <rosgraph_msgs/Clock.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float64MultiArray.h>

#include "car_msgs/simulated_telemetry.h"

#include <unistd.h>


void singletrack_beta_sim::Prepare(void) {
    /* Retrieve parameters from ROS parameter server */
    std::string FullParamName;

    // Simulator parameters
    FullParamName = ros::this_node::getName() + "/actuator_model";
    if (false == Handle.getParam(FullParamName, actuator_model))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                  FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/tyre_model";
    if (false == Handle.getParam(FullParamName, tyre_model))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                  FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/input_cmd";
    if (false == Handle.getParam(FullParamName, input_cmd))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                  FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/automode_delay";
    if (false == Handle.getParam(FullParamName, automode_delay))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                  FullParamName.c_str());

    // Vehicle parameters
    FullParamName = ros::this_node::getName() + "/cog_a";
    if (false == Handle.getParam(FullParamName, a))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                  FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/cog_b";
    if (false == Handle.getParam(FullParamName, b))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                  FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/m";
    if (false == Handle.getParam(FullParamName, m))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                  FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/mu";
    if (false == Handle.getParam(FullParamName, mu))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                  FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/Cf";
    if (false == Handle.getParam(FullParamName, Cf))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                  FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/Cr";
    if (false == Handle.getParam(FullParamName, Cr))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                  FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/Iz";
    if (false == Handle.getParam(FullParamName, Iz))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                  FullParamName.c_str());

    // Steering actuator model parameters
    FullParamName = ros::this_node::getName() + "/mu_steer";
    if (false == Handle.getParam(FullParamName, mu_steer))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                  FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/wn_steer";
    if (false == Handle.getParam(FullParamName, wn_steer))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                  FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/csi_steer";
    if (false == Handle.getParam(FullParamName, csi_steer))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                  FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/tau_steer";
    if (false == Handle.getParam(FullParamName, tau_steer))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                  FullParamName.c_str());

    // Speed/force actuator parameters
    if (input_cmd == 0) {        // Speed & steer
        // Velocity actuator model parameters
        FullParamName = ros::this_node::getName() + "/mu_speed";
        if (false == Handle.getParam(FullParamName, mu_speed))
            ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                      FullParamName.c_str());

        FullParamName = ros::this_node::getName() + "/wn_speed";
        if (false == Handle.getParam(FullParamName, wn_speed))
            ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                      FullParamName.c_str());

        FullParamName = ros::this_node::getName() + "/csi_speed";
        if (false == Handle.getParam(FullParamName, csi_speed))
            ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                      FullParamName.c_str());

        FullParamName = ros::this_node::getName() + "/tau_speed";
        if (false == Handle.getParam(FullParamName, tau_speed))
            ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                      FullParamName.c_str());
    } else {                     // Fxr & steer
        // Velocity actuator model parameters
        FullParamName = ros::this_node::getName() + "/mu_force";
        if (false == Handle.getParam(FullParamName, mu_force))
            ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                      FullParamName.c_str());

        FullParamName = ros::this_node::getName() + "/wn_force";
        if (false == Handle.getParam(FullParamName, wn_force))
            ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                      FullParamName.c_str());

        FullParamName = ros::this_node::getName() + "/csi_force";
        if (false == Handle.getParam(FullParamName, csi_force))
            ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                      FullParamName.c_str());

        FullParamName = ros::this_node::getName() + "/tau_force";
        if (false == Handle.getParam(FullParamName, tau_force))
            ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                      FullParamName.c_str());
    }

    // Vehicle initial state
    FullParamName = ros::this_node::getName() + "/r0";
    if (false == Handle.getParam(FullParamName, r0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                  FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/Vx0";
    if (false == Handle.getParam(FullParamName, Vx0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                  FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/Vy0";
    if (false == Handle.getParam(FullParamName, Vy0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                  FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/x0";
    if (false == Handle.getParam(FullParamName, x0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                  FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/y0";
    if (false == Handle.getParam(FullParamName, y0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                  FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/psi0";
    if (false == Handle.getParam(FullParamName, psi0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                  FullParamName.c_str());

    // Measurement publishing
    FullParamName = ros::this_node::getName() + "/pose_pub_dt";
    if (false == Handle.getParam(FullParamName, pose_pub_dt))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                  FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/imu_pub_dt";
    if (false == Handle.getParam(FullParamName, imu_pub_dt))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(),
                  FullParamName.c_str());

    /* ROS topics */
    vehicleCommand_subscriber = Handle.subscribe("/controller_cmd", 1,
                                                 &singletrack_beta_sim::vehicleCommand_MessageCallback, this);
    vehiclePose_publisher = Handle.advertise<geometry_msgs::Pose2D>("/car/ground_pose", 1);
    vehicleIMU_publisher = Handle.advertise<sensor_msgs::Imu>("/imu/data", 1);
    vehicleState_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/car/state", 1);
    telemetry_publisher = Handle.advertise<car_msgs::simulated_telemetry>("/car_simulator/telemetry", 1);
    clock_publisher = Handle.advertise<rosgraph_msgs::Clock>("/clock", 1);
    radioCommand_publisher = Handle.advertise<car_msgs::car_cmd>("/radio_cmd", 1);

    /* Create simulator class */
    if (actuator_model == 0) {         // ideal
        if (tyre_model == 0) {         // linear
            if (input_cmd == 0) {      // velocity
                sim_velocity = new singletrack_beta_velocity_ode((double)DT_SIMULATOR*1.0E-6, singletrack_beta_velocity_ode::LINEAR,
                                                                 singletrack_beta_velocity_ode::IDEAL, 0.05);
                ROS_INFO("Node %s running beta_velocity model with linear tyres and ideal actuators.",
                         ros::this_node::getName().c_str());
            } else {                   // force
                sim_force = new singletrack_beta_force_ode((double)DT_SIMULATOR*1.0E-6, singletrack_beta_force_ode::LINEAR,
                                                           singletrack_beta_force_ode::IDEAL, 0.01);
                ROS_INFO("Node %s running beta_force model with linear tyres and ideal actuators.",
                         ros::this_node::getName().c_str());
            }
        } else if (tyre_model == 1) {  // fiala with saturation
            if (input_cmd == 0) {      // velocity
                sim_velocity = new singletrack_beta_velocity_ode((double)DT_SIMULATOR*1.0E-6,
                                                                 singletrack_beta_velocity_ode::FIALA_WITH_SATURATION,
                                                                 singletrack_beta_velocity_ode::IDEAL, 0.05);
                ROS_INFO("Node %s running beta_velocity model with Fiala with saturation tyres and ideal actuators.",
                         ros::this_node::getName().c_str());
            } else {                   // force
                sim_force = new singletrack_beta_force_ode((double)DT_SIMULATOR*1.0E-6, singletrack_beta_force_ode::FIALA_WITH_SATURATION,
                                                           singletrack_beta_force_ode::IDEAL, 0.01);
                ROS_INFO("Node %s running beta_force model with Fiala with saturation tyres and ideal actuators.",
                         ros::this_node::getName().c_str());
            }
        } else if (tyre_model == 2) {  // fiala witout saturation
            if (input_cmd == 0) {      // velocity
                sim_velocity = new singletrack_beta_velocity_ode((double)DT_SIMULATOR*1.0E-6,
                                                                 singletrack_beta_velocity_ode::FIALA_WITHOUT_SATURATION,
                                                                 singletrack_beta_velocity_ode::IDEAL, 0.05);
                ROS_INFO("Node %s running beta_velocity model with Fiala without saturation tyres and ideal actuators.",
                         ros::this_node::getName().c_str());
            } else {                   // force
                sim_force = new singletrack_beta_force_ode((double)DT_SIMULATOR*1.0E-6, singletrack_beta_force_ode::FIALA_WITHOUT_SATURATION,
                                                           singletrack_beta_force_ode::IDEAL, 0.01);
                ROS_INFO("Node %s running beta_force model with Fiala without saturation tyres and ideal actuators.",
                         ros::this_node::getName().c_str());
            }
        } else {                       // error
            ROS_ERROR("Node %s: unknown tyre model.", ros::this_node::getName().c_str());
        }
    } else if (actuator_model == 1) {  // real
        if (tyre_model == 0) {         // linear
            if (input_cmd == 0) {      // velocity
                sim_velocity = new singletrack_beta_velocity_ode((double)DT_SIMULATOR*1.0E-6, singletrack_beta_velocity_ode::LINEAR,
                                                                 singletrack_beta_velocity_ode::REAL, 0.05);
                ROS_INFO("Node %s running beta_velocity model with linear tyres and real actuators.",
                         ros::this_node::getName().c_str());
            } else {                   // force
                sim_force = new singletrack_beta_force_ode((double)DT_SIMULATOR*1.0E-6, singletrack_beta_force_ode::LINEAR,
                                                           singletrack_beta_force_ode::REAL, 0.01);
                ROS_INFO("Node %s running beta_force model with linear tyres and real actuators.",
                         ros::this_node::getName().c_str());
            }
        } else if (tyre_model == 1) {  // fiala with saturation
            if (input_cmd == 0) {      // velocity
                sim_velocity = new singletrack_beta_velocity_ode((double)DT_SIMULATOR*1.0E-6,
                                                                 singletrack_beta_velocity_ode::FIALA_WITH_SATURATION,
                                                                 singletrack_beta_velocity_ode::REAL, 0.05);
                ROS_INFO("Node %s running beta_velocity model with Fiala with saturation tyres and real actuators.",
                         ros::this_node::getName().c_str());
            } else {                   // force
                sim_force = new singletrack_beta_force_ode((double)DT_SIMULATOR*1.0E-6, singletrack_beta_force_ode::FIALA_WITH_SATURATION,
                                                           singletrack_beta_force_ode::REAL, 0.01);
                ROS_INFO("Node %s running beta_force model with Fiala with saturation tyres and real actuators.",
                         ros::this_node::getName().c_str());
            }
        } else if (tyre_model == 2) {  // fiala witout saturation
            if (input_cmd == 0) {      // velocity
                sim_velocity = new singletrack_beta_velocity_ode((double)DT_SIMULATOR*1.0E-6,
                                                                 singletrack_beta_velocity_ode::FIALA_WITHOUT_SATURATION,
                                                                 singletrack_beta_velocity_ode::REAL, 0.05);
                ROS_INFO("Node %s running beta_velocity model with Fiala without saturation tyres and real actuators.",
                         ros::this_node::getName().c_str());
            } else {                   // force
                sim_force = new singletrack_beta_force_ode((double)DT_SIMULATOR*1.0E-6, singletrack_beta_force_ode::FIALA_WITHOUT_SATURATION,
                                                           singletrack_beta_force_ode::REAL, 0.01);
                ROS_INFO("Node %s running beta_force model with Fiala without saturation tyres and real actuators.",
                         ros::this_node::getName().c_str());
            }
        } else {                       // error
            ROS_ERROR("Node %s: unknown tyre model.", ros::this_node::getName().c_str());
        }
    } else {                           // error
        ROS_ERROR("Node %s: unknown actuator model.", ros::this_node::getName().c_str());
    }

    /* Initialize simulator class */
    if (input_cmd == 0) {   // Speed & steer
        sim_velocity->setInitialState(r0, std::atan2(Vy0,Vx0), x0, y0, psi0);
        sim_velocity->setActuatorParams(mu_steer, wn_steer, csi_steer, tau_steer, 0.0,
                                        mu_speed, wn_speed, csi_speed, tau_speed, std::sqrt(std::pow(Vx0, 2.0)+std::pow(Vy0, 2.0)));
        sim_velocity->setVehicleParams(m, a, b, Cf, Cr, mu, Iz);

        speed_ref = std::sqrt(std::pow(Vx0, 2.0)+std::pow(Vy0, 2.0));
        steer_ref = 0.0;
    } else {                // Fxr & steer
        sim_force->setInitialState(r0, std::atan2(Vy0,Vx0), std::sqrt(std::pow(Vx0,2.0)+std::pow(Vy0,2.0)), x0, y0, psi0);
        sim_force->setActuatorParams(mu_steer, wn_steer, csi_steer, tau_steer, 0.0,
                                     mu_force, wn_force, csi_force, tau_force, 0.0);
        sim_force->setVehicleParams(m, a, b, Cf, Cr, mu, Iz);

        force_ref = std::sqrt(std::pow(Vx0, 2.0)+std::pow(Vy0, 2.0));
        steer_ref = 0.0;
    }

    /* Initialize node state */
    time = 0;
    manual_mode = true;

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void singletrack_beta_sim::RunPeriodically(void) {
    ROS_INFO("Node %s running.", ros::this_node::getName().c_str());

    // Wait other nodes start
    sleep(1.0);

    // Set the car in manual mode
    car_msgs::car_cmd msg;
    msg.state = car_msgs::car_cmd::STATE_MANUAL;
    msg.speed_ref = 0.0;
    msg.steer_ref = 0.0;
    radioCommand_publisher.publish(msg);

    while (ros::ok()) {
        PeriodicTask();

        ros::spinOnce();

        usleep(10);
    }
}

void singletrack_beta_sim::Shutdown(void) {
    // Delete ode object
    if (input_cmd == 0) {
        delete sim_velocity;
    } else {
        delete sim_force;
    }

    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void singletrack_beta_sim::vehicleCommand_MessageCallback(const car_msgs::car_cmd::ConstPtr &msg) {
    /*  Store vehicle commands */
    if (input_cmd == 0) {   // Speed & steer
        speed_ref = msg->speed_ref;
        steer_ref = msg->steer_ref;
    } else {                // Fxr & steer
        force_ref = msg->speed_ref;
        steer_ref = msg->steer_ref;
    }
}

void singletrack_beta_sim::PeriodicTask(void) {
    auto dv = std::div(time, (long) DT_SIMULATOR);
    if (std::abs(dv.rem) < DT_CLOCK) {
        /*  Set vehicle commands */
        if (input_cmd == 0) {   // Speed & steer
            sim_velocity->setReferenceCommands(speed_ref, steer_ref);
        } else {                // Fxr & steer
            sim_force->setReferenceCommands(force_ref, steer_ref);
        }

        /*  Integrate the model */
        if (input_cmd == 0) {
            sim_velocity->integrate();
        } else {
            sim_force->integrate();
        }

        /*  Extract measurement from simulator */
        double x, y, theta;
        if (input_cmd == 0) {
            sim_velocity->getPose(x, y, theta);
        } else {
            sim_force->getPose(x, y, theta);
        }

        double ay, yawrate, vy;
        if (input_cmd == 0) {
            sim_velocity->getLateralDynamics(ay, yawrate, vy);
        } else {
            sim_force->getLateralDynamics(ay, yawrate, vy);
        }

        double vx, v;
        if (input_cmd == 1) {
            sim_force->getLongitudinalDynamics(vx);
            sim_force->getAbsoluteVelocity(v);
        }

        double sideslip;
        if (input_cmd == 0) {
            sim_velocity->getSideslip(sideslip);
        } else {
            sim_force->getSideslip(sideslip);
        }

        double slip_front, slip_rear;
        if (input_cmd == 0) {
            sim_velocity->getSlip(slip_front, slip_rear);
        } else {
            sim_force->getSlip(slip_front, slip_rear);
        }

        double Fy_front, Fy_rear;
        if (input_cmd == 0) {
            sim_velocity->getLateralForce(Fy_front, Fy_rear);
        } else {
            sim_force->getLateralForce(Fy_front, Fy_rear);
        }

        double velocity_act, Fx_act, steer_act;
        if (input_cmd == 0) {
            sim_velocity->getCommands(velocity_act, steer_act);
        } else {
            sim_force->getCommands(Fx_act, steer_act);
        }

        /*  Publish the change of state from manual to auto */
        if ((time >= automode_delay) && manual_mode) {
            manual_mode = false;

            car_msgs::car_cmd msg;
            msg.state = car_msgs::car_cmd::STATE_AUTOMATIC;
            msg.speed_ref = 0.0;
            msg.steer_ref = 0.0;
            radioCommand_publisher.publish(msg);
        }

        /*  Publish vehicle pose */
        auto dv = std::div(time, (long) pose_pub_dt);
        if (std::abs(dv.rem) < DT_CLOCK) {
            geometry_msgs::Pose2D vehiclePoseMsg;
            vehiclePoseMsg.x = x;
            vehiclePoseMsg.y = y;
            vehiclePoseMsg.theta = theta;
            vehiclePose_publisher.publish(vehiclePoseMsg);
        }

        /*  Publish IMU data */
        dv = std::div(time, (long) imu_pub_dt);
        if (std::abs(dv.rem) < DT_CLOCK) {
            sensor_msgs::Imu imuDataMsg;
            imuDataMsg.header.stamp = ros::Time((double)time*1.0E-6);
            imuDataMsg.angular_velocity.x = 0.0;
            imuDataMsg.angular_velocity.y = 0.0;
            imuDataMsg.angular_velocity.z = yawrate;
            imuDataMsg.linear_acceleration.x = 0.0;  // This is not considered in the model (yet)
            imuDataMsg.linear_acceleration.y = ay;
            imuDataMsg.linear_acceleration.z = 0.0;
            vehicleIMU_publisher.publish(imuDataMsg);
        }

        /*  Publish vehicle state */
        std_msgs::Float64MultiArray vehicleStateMsg;
        if (input_cmd == 0) {   // Speed & steer
            vehicleStateMsg.data.push_back((double)time*1.0E-6);
            vehicleStateMsg.data.push_back(x);
            vehicleStateMsg.data.push_back(y);
            vehicleStateMsg.data.push_back(theta);
            vehicleStateMsg.data.push_back(yawrate);
            vehicleStateMsg.data.push_back(velocity_act);
            vehicleStateMsg.data.push_back(vy);
            vehicleStateMsg.data.push_back(ay);
            vehicleStateMsg.data.push_back(sideslip);
            vehicleStateMsg.data.push_back(slip_front);
            vehicleStateMsg.data.push_back(slip_rear);
            vehicleStateMsg.data.push_back(Fy_front);
            vehicleStateMsg.data.push_back(Fy_rear);
            vehicleStateMsg.data.push_back(velocity_act);
            vehicleStateMsg.data.push_back(steer_act);
            vehicleState_publisher.publish(vehicleStateMsg);
        } else {                // Fxr & steer
            vehicleStateMsg.data.push_back((double)time*1.0E-6);
            vehicleStateMsg.data.push_back(x);
            vehicleStateMsg.data.push_back(y);
            vehicleStateMsg.data.push_back(theta);
            vehicleStateMsg.data.push_back(yawrate);
            vehicleStateMsg.data.push_back(vx);
            vehicleStateMsg.data.push_back(vy);
            vehicleStateMsg.data.push_back(ay);
            vehicleStateMsg.data.push_back(sideslip);
            vehicleStateMsg.data.push_back(slip_front);
            vehicleStateMsg.data.push_back(slip_rear);
            vehicleStateMsg.data.push_back(Fy_front);
            vehicleStateMsg.data.push_back(Fy_rear);
            vehicleStateMsg.data.push_back(Fx_act);
            vehicleStateMsg.data.push_back(steer_act);
            vehicleState_publisher.publish(vehicleStateMsg);
        }

        /*  Publish telemetry */
        car_msgs::simulated_telemetry vehicleTelemetryMsg;
        if (input_cmd == 0) {
            vehicleTelemetryMsg.sideslip = sideslip;
            vehicleTelemetryMsg.Vx = velocity_act;
            vehicleTelemetryMsg.Vy = vy;
            vehicleTelemetryMsg.yaw_rate = yawrate;
        } else {
            vehicleTelemetryMsg.sideslip = sideslip;
            vehicleTelemetryMsg.Vx = vx;
            vehicleTelemetryMsg.Vy = vy;
            vehicleTelemetryMsg.yaw_rate = yawrate;
        }
        telemetry_publisher.publish(vehicleTelemetryMsg);
    }

    /*  Publish clock */
    rosgraph_msgs::Clock clockMsg;
    clockMsg.clock = ros::Time((double)time*1.0E-6);
    clock_publisher.publish(clockMsg);

    /*  Print simulation time every 5 sec */
    dv = std::div(time, (long) 5000000);
    if (std::abs(dv.rem) < DT_CLOCK) {
        ROS_INFO("Simulator time: %.10f seconds", (double)time*1.0E-6);
    }

    /* Update time */
    time += DT_CLOCK;
}
