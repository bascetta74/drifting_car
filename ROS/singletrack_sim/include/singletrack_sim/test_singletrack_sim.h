#ifndef TEST_SINGLETRACK_SIM_H_
#define TEST_SINGLETRACK_SIM_H_

#include "ros/ros.h"

#define RUN_PERIOD_DEFAULT 0.01
/* Used only if the actual value of the period is not retrieved from the ROS parameter server */

#define NAME_OF_THIS_NODE "test_singletrack_sim"


class test_singletrack_sim
{
  private: 
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Publisher vehicleCommand_publisher;
    
    /* Node periodic task */
    void PeriodicTask(void);

    /* Node parameters */
    int input_cmd;

  public:
    double RunPeriod;

    void Prepare(void);
    
    void RunPeriodically(float Period);
    
    void Shutdown(void);

};

#endif /* TEST_SINGLETRACK_SIM_H_ */
