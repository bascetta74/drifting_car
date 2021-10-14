#ifndef TEST_FBLIN_SCHLACHER_H_
#define TEST_FBLIN_SCHLACHER_H_

#include "ros/ros.h"
#include <vector>

//#define OPEN_LOOP_TEST
#define CLOSED_LOOP_TEST

#define RUN_PERIOD_DEFAULT 0.1
/* Used only if the actual value of the period is not retrieved from the ROS parameter server */
 
#define NAME_OF_THIS_NODE "test_feedback_linearization"

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose2D.h>
#include "car_msgs/simulated_telemetry.h"
#include "car_msgs/car_cmd.h"

#include <boost/circular_buffer.hpp>

#include "PIDcontrol.h"
#include "fblin_schlacher.h"

 
class test_fblin_schlacher
{
  private: 
    ros::NodeHandle Handle;
    
    /* ROS topics */
    ros::Subscriber vehiclePose_subscriber, telemetry_subscriber;
    ros::Subscriber vehicleIMU_subscriber, radiocmd_subscriber;
    ros::Publisher controllerCommand_publisher;
    ros::Publisher vehicleState_publisher, controllerState_publisher;
    
    /* Parameters from ROS parameter server */
    double m, Iz, Cf, Cr, lf, lr, car2motor_conversion;
    double speed_thd, theta_offset;
    double Kcx, Tdx, Nx, Kcy, Tiy, Tdy, Ny, vp_min, vp_max;
    double T0, A1, T1, A2, T2, T3, delta_vx, T;
    bool use_ideal_sim, use_sim_time;
    unsigned int vel_filt_order;
    std::vector<double> vel_filt_coeff;

    /* ROS topic callbacks */
    void vehiclePose_MessageCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
    void vehicleIMU_MessageCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void simulated_telemetry_MessageCallback(const car_msgs::simulated_telemetry::ConstPtr& msg);
    void radioCommand_MessageCallback(const car_msgs::car_cmd::ConstPtr& msg);
 
    /* Estimator periodic task */
    void PeriodicTask(void);
    
    /* Node state variables */
    double _time, _x0, _y0, _theta0, _vx0;
    double _z1, _z2;

    ros::Time _t0;
    unsigned int _car_control_state;
    double _vehicleSideslip, _vehicleAngularVelocity;
    std::vector<double> _vehiclePose, _vehicleVelocity;
    boost::circular_buffer<double> _vehiclePositionXBuffer, _vehiclePositionYBuffer, _vehiclePositionTimeBuffer;

    fblin_schlacher* _linearizer;
    PIDcontrol* _PDx;
    PIDcontrol* _PIDy;

  public:
    double RunPeriod;
    
    void Prepare(void);
    
    void RunPeriodically(float Period);
    
    void Shutdown(void);
};

#endif /* TEST_FBLIN_SCHLACHER_H_ */
