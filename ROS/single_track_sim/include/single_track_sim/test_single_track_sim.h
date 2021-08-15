#ifndef TEST_SINGLE_TRACK_SIM_H_
#define TEST_SINGLE_TRACK_SIM_H_

#include "ros/ros.h"

#define RUN_PERIOD_DEFAULT 0.01
/* Used only if the actual value of the period is not retrieved from the ROS parameter server */

#define NAME_OF_THIS_NODE "test_single_track_sim"


class test_single_track_sim
{
  private: 
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Publisher vehicleCommand_publisher;
    
    /* Estimator periodic task */
    void PeriodicTask(void);
    
  public:
    double RunPeriod;

    void Prepare(void);
    
    void RunPeriodically(float Period);
    
    void Shutdown(void);

};

#endif /* TEST_SINGLE_TRACK_SIM_H_ */
