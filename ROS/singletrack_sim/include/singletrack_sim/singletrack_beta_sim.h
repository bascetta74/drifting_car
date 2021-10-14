#ifndef SINGLETRACK_BETA_SIM_H_
#define SINGLETRACK_BETA_SIM_H_

#include "ros/ros.h"
#include "car_msgs/car_cmd.h"

#include "singletrack_beta_force_ode.h"

#define NAME_OF_THIS_NODE "singletrack_beta_sim"


class singletrack_beta_sim
{
  private: 
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Subscriber vehicleCommand_subscriber;
    ros::Publisher vehiclePose_publisher;
    ros::Publisher vehicleIMU_publisher;
    ros::Publisher vehicleState_publisher;
    ros::Publisher telemetry_publisher;
    ros::Publisher clock_publisher;
    ros::Publisher radioCommand_publisher;
    
    /* Parameters from ROS parameter server */
    bool manual_mode;
    double dt, automode_delay;
    int actuator_model, tyre_model, input_cmd;
    double r0, beta0, V0, x0, y0, psi0;
    double mu_steer, wn_steer, csi_steer, tau_steer, mu_speed;
    double m, a, b, Cf, Cr, mu, Iz;
    int pose_decimation, imu_decimation;

    /* ROS topic callbacks */
    void vehicleCommand_MessageCallback(const car_msgs::car_cmd::ConstPtr& msg);

    /* Estimator periodic task */
    void PeriodicTask(void);
    
    /* Node state variables */
    // singletrack_beta_velocity_ode* sim_velocity;
    singletrack_beta_force_ode* sim_force;
    int pose_pub_idx, imu_pub_idx;

  public:

    void Prepare(void);
    
    void RunPeriodically(void);
    
    void Shutdown(void);

};

#endif /* SINGLETRACK_BETA_SIM_H_ */
