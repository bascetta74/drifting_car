#ifndef AFI_CONTROLLER_H_
#define AFI_CONTROLLER_H_

#include "ros/ros.h"

#define RUN_PERIOD_DEFAULT 0.1
/* Used only if the actual value of the period is not retrieved from the ROS parameter server */
 
#define NAME_OF_THIS_NODE "AFI_controller"

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose2D.h>
#include "car_msgs/car_cmd.h"

#include <vector>
#include <boost/circular_buffer.hpp>


class AFI_controller
{
  private: 
    ros::NodeHandle Handle;
    
    /* ROS topics */
    ros::Subscriber vehiclePose_subscriber;
    ros::Subscriber vehicleIMU_subscriber, radiocmd_subscriber;
    ros::Publisher controllerCommand_publisher;
    ros::Publisher controllerState_publisher;
    
    /* Parameters from ROS parameter server */
    double a, Cf, car2motor_conversion;
    unsigned int vel_filt_order;
    std::vector<double> vel_filt_coeff;
    double theta_offset, speed_thd;

    /* ROS topic callbacks */
    void vehiclePose_MessageCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
    void vehicleIMU_MessageCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void radioCommand_MessageCallback(const car_msgs::car_cmd::ConstPtr& msg);
 
    /* Estimator periodic task */
    void PeriodicTask(void);
    
    /* Node state variables */
    ros::Time _t0;
    unsigned int _car_control_state;

    double _time, _vehicleAngularVelocity, _vehicleSideslip, _vehicleLongitudinalVelocity;
    double _speedRef, _steerRef, _FyfRef;
    std::vector<double> _vehiclePose, _vehicleVelocity;

    boost::circular_buffer<double> _vehiclePositionXBuffer, _vehiclePositionYBuffer, _vehiclePositionTimeBuffer;

  public:
    double RunPeriod;
    
    void Prepare(void);
    
    void RunPeriodically(float Period);
    
    void Shutdown(void);
};

#endif /* AFI_CONTROLLER_H_ */
