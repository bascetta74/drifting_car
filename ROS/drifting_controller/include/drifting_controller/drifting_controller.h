#ifndef DRIFTING_CONTROLLER_H_
#define DRIFTING_CONTROLLER_H_

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Vector3.h"
#include "car_msgs/car_cmd.h"

#define RUN_PERIOD_DEFAULT 0.1
/* Used only if the actual value of the period is not retrieved from the ROS parameter server */

#define NAME_OF_THIS_NODE "drifting_controller"


class drifting_controller
{
  private:
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Subscriber imu_subscriber;
    ros::Subscriber radioCommand_subscriber;
    ros::Subscriber wheelSpeed_subscriber;
    ros::Publisher  controllerCommand_publisher;

    /* ROS topic callbacks */
    void imu_MessageCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void radioCommand_MessageCallback(const car_msgs::car_cmd::ConstPtr& msg);
    void wheelSpeed_MessageCallback(const std_msgs::Float64::ConstPtr& msg);

    /* Estimator periodic task */
    void PeriodicTask(void);

    /* Node state variables */
    double                      _steer_ref;
    double                      _speed_ref;
    unsigned int                _state;
    double                      _wheel_speed;
    geometry_msgs::Vector3      _angular_velocity;
    geometry_msgs::Vector3      _linear_acceleration;

  public:

    double RunPeriod;

    void Prepare(void);

    void RunPeriodically(float Period);

    void Shutdown(void);
};

#endif /* DRIFTING_CONTROLLER_H_ */
