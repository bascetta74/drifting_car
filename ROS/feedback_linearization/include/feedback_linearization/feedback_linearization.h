#ifndef FEEDBACK_LINEARIZATION_H_
#define FEEDBACK_LINEARIZATION_H_

#include "ros/ros.h"
#include <vector>

#define SPALIVIERO
//#define LOPEZ_I
//#define LOPEZ_II

#define RUN_PERIOD_DEFAULT 0.1
/* Used only if the actual value of the period is not retrieved from the ROS parameter server */
 
#define NAME_OF_THIS_NODE "feedback_linearization"

#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Pose2D.h"

#include <boost/circular_buffer.hpp>

#ifdef SPALIVIERO
#include "fblin_spaliviero.h"
#endif
#ifdef LOPEZ_I
#include "fblin_lopez_I.h"
#endif
#ifdef LOPEZ_II
#include "fblin_lopez_II.h"
#endif

 
class feedback_linearization
{
  private: 
    ros::NodeHandle Handle;
    
    /* ROS topics */
    ros::Subscriber vehiclePose_subscriber;
    ros::Subscriber vehicleIMU_subscriber;
    ros::Publisher controllerCommand_publisher;
    ros::Publisher vehicleState_publisher, pointP_publisher, pointPvelocity_publisher;
    
    /* Parameters from ROS parameter server */
    double Cf, Cr, a, b, m, Jz;
    double P_dist, speed_thd, KPx, KPy;
    double theta_offset;
    unsigned int vel_filt_order;
    std::vector<double> vel_filt_coeff;

    /* ROS topic callbacks */
    void vehiclePose_MessageCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
    void vehicleIMU_MessageCallback(const sensor_msgs::Imu::ConstPtr& msg);
 
    /* Estimator periodic task */
    void PeriodicTask(void);
    
    /* Node state variables */
    double _time;
    double _vehicleSideslip;
    std::vector<double> _vehiclePose, _vehicleVelocity, _pointPposition;
    std::vector<double> _vehicleAcceleration, _vehicleAngularVelocity;
    boost::circular_buffer<double> _vehiclePositionXBuffer, _vehiclePositionYBuffer;

    #ifdef SPALIVIERO
    fblin_spaliviero* _linearizer;
    #endif
    #ifdef LOPEZ_I
    fblin_lopez_I* _linearizer;
    #endif
    #ifdef LOPEZ_II
    fblin_lopez_II* _linearizer;
    #endif
    
  public:
    double RunPeriod;
    
    void Prepare(void);
    
    void RunPeriodically(float Period);
    
    void Shutdown(void);
};

#endif /* FEEDBACK_LINEARIZATION_H_ */