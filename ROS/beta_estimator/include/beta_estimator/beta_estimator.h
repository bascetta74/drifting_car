#ifndef BETA_ESTIMATOR_H_
#define BETA_ESTIMATOR_H_

#include "ros/ros.h"
#include <vector>

#define RUN_PERIOD_DEFAULT 0.1
/* Used only if the actual value of the period is not retrieved from the ROS parameter server */
 
#define NAME_OF_THIS_NODE "beta_estimator"

#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Float64.h"

 
class beta_estimator
{
  private: 
    ros::NodeHandle Handle;
    
    /* ROS topics */
    ros::Subscriber vehiclePose_subscriber;
    ros::Publisher vehicleSideslip_publisher;
    
    /* Parameters from ROS parameter server */
    double Kp, Kd, T;

    /* ROS topic callbacks */
    void vehiclePose_MessageCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
 
    /* Estimator periodic task */
    void PeriodicTask(void);
    
    /* Node state variables */
    double		_vehicleSideslip;
    std::vector<double> _vehiclePose;
    
  public:
    double RunPeriod;
    
    void Prepare(void);
    
    void RunPeriodically(float Period);
    
    void Shutdown(void);
};

#endif /* BETA_ESTIMATOR_H_ */