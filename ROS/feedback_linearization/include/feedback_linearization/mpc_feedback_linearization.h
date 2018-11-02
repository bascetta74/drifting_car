#ifndef MPC_FEEDBACK_LINEARIZATION_H_
#define MCP_FEEDBACK_LINEARIZATION_H_

#include "ros/ros.h"
#include <vector>
#include <Eigen/Dense>

#define RUN_PERIOD_DEFAULT 0.1
/* Used only if the actual value of the period is not retrieved from the ROS parameter server */
 
#define NAME_OF_THIS_NODE "mpc_feedback_linearization"

#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Pose2D.h"

        
#include <ackermann_mpc_local_planner/MPCcontroller.h>
#include <ackermann_mpc_local_planner/mpc_config.h>

 
class mpc_feedback_linearization
{
  private: 
    ros::NodeHandle Handle;
    ackermann_mpc_local_planner::MPCconfig _cfg;
    MPC::MPCcontroller _mpc;
    Eigen::VectorXd state;
    Eigen::VectorXd goal;
    
    /* ROS topics */
    ros::Subscriber vehiclePose_subscriber;
    ros::Subscriber vehicleIMU_subscriber;
    ros::Publisher controllerCommand_publisher;
    ros::Publisher vehicleState_publisher, pointP_publisher, pointPvelocity_publisher;
    
    /* Parameters from ROS parameter server */
    double  a, b;
    double theta_offset, deltas_thd; 

    /* ROS topic callbacks */
    void vehiclePose_MessageCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
    void vehicleIMU_MessageCallback(const sensor_msgs::Imu::ConstPtr& msg);
 
    /* Estimator periodic task */
    void PeriodicTask(void);
    
    /* Node state variables */
    double _time;
    double _vehicleSideslip;
    std::vector<double> _vehiclePose, _vehicleVelocity;
    std::vector<double> _vehicleAcceleration, _vehicleAngularVelocity;

    
  public:
    double RunPeriod;
    
    void Prepare(void);
    
    void RunPeriodically(float Period);
    
    void Shutdown(void);

  private:
    void fblin_control_singletrack_pointP(double vPx, double vPy, double& speed, double& steer);
    void fblin_output_singletrack_pointP(double& xP, double& yP);
};

#endif /* FEEDBACK_LINEARIZATION_H_ */