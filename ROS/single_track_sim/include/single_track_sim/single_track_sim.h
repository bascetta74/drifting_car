#ifndef SINGLE_TRACK_SIM_H_
#define SINGLE_TRACK_SIM_H_

#include "ros/ros.h"
#include "car_msgs/car_cmd.h"

#include "single_track_force_ode.h"
#include "single_track_velocity_ode.h"

#define NAME_OF_THIS_NODE "single_track_sim"


class single_track_sim
{
  private: 
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Subscriber vehicleCommand_subscriber;
    ros::Publisher vehiclePose_publisher;
    ros::Publisher vehicleIMU_publisher;
    ros::Publisher vehicleState_publisher;
    ros::Publisher clock_publisher;
    ros::Publisher radioCommand_publisher;
    
    /* Parameters from ROS parameter server */
    double dt;
    int actuator_model, tyre_model, input_cmd;
    double r0, Vx0, Vy0, x0, y0, psi0;
    double mu_steer, wn_steer, csi_steer, tau_steer, mu_speed;
    double m, a, b, Cf, Cr, mu, Iz;
    int pose_decimation, imu_decimation;

    /* ROS topic callbacks */
    void vehicleCommand_MessageCallback(const car_msgs::car_cmd::ConstPtr& msg);

    /* Estimator periodic task */
    void PeriodicTask(void);
    
    /* Node state variables */
    single_track_velocity_ode* sim_velocity;
    single_track_force_ode* sim_force;
    int pose_pub_idx, imu_pub_idx;

  public:

    void Prepare(void);
    
    void RunPeriodically(void);
    
    void Shutdown(void);

};

#endif /* SINGLE_TRACK_SIM_H_ */
