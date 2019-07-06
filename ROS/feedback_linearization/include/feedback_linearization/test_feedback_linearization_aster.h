#ifndef TEST_FEEDBACK_LINEARIZATION_ASTER_H_
#define TEST_FEEDBACK_LINEARIZATION_ASTER_H_

#include "ros/ros.h"
#include <vector>

//#define OPEN_LOOP_TEST
#define CLOSED_LOOP_TEST

#define RUN_PERIOD_DEFAULT 0.1
/* Used only if the actual value of the period is not retrieved from the ROS parameter server */
 
#define NAME_OF_THIS_NODE "test_feedback_linearization"

#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include "aster_msgs/Crio.h"

#include <boost/circular_buffer.hpp>

#include "PIDcontrol.h"
#include "fblin_lopez.h"

 
class test_feedback_linearization_aster
{
  private: 
    ros::NodeHandle Handle;
    
    /* ROS topics */
    ros::Subscriber vehiclePose_subscriber, vehicleIMU_subscriber, startcmd_subscriber;
    ros::Publisher commandVel_publisher;
    ros::Publisher vehicleState_publisher, pointPact_publisher, pointPref_publisher, pointPvelocity_publisher, vehicleRef_publisher;
    
    /* Parameters from ROS parameter server */
    double a, b;
    double P_dist, speed_thd, Kp, Ti, vp_min, vp_max;
    unsigned int vel_filt_order;
    std::vector<double> vel_filt_coeff;

    /* ROS topic callbacks */
    void vehiclePose_MessageCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void vehicleIMU_MessageCallback(const aster_msgs::Crio::ConstPtr& msg);
    void startCommand_MessageCallback(const std_msgs::Bool::ConstPtr& msg);
 
    /* Estimator periodic task */
    void PeriodicTask(void);
    
    /* Node state variables */
    double _time, _x0, _y0, _theta0;
    double _radius, _xcenter, _ycenter, _omega, _const_speed;
    
    ros::Time _t0;
    bool _start_control;
    double _vehicleSideslip, _vehicleAngularVelocity;
    std::vector<double> _vehiclePose, _vehicleVelocity;
    boost::circular_buffer<double> _vehiclePositionXBuffer, _vehiclePositionYBuffer;

    fblin_lopez* _linearizer;
    PIDcontrol* _PIx;
    PIDcontrol* _PIy;
    
  public:
    double RunPeriod;
    
    void Prepare(void);
    
    void RunPeriodically(float Period);
    
    void Shutdown(void);
};

#endif /* TEST_FEEDBACK_LINEARIZATION_ASTER_H_ */
