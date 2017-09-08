#ifndef KEYBOARD_TELEOP_H_
#define KEYBOARD_TELEOP_H_

#include "ros/ros.h"
#include "keyboard/Key.h"


#define RUN_PERIOD_DEFAULT 0.1
/* Used only if the actual value of the period is not retrieved from the ROS parameter server */

#define NAME_OF_THIS_NODE "keyboard_teleop"


class keyboard_teleop
{
  private:
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Publisher radioCommand_publisher;
    ros::Subscriber keyUp_subscriber, keyDown_subscriber;

    /* ROS topic callbacks */
    void keyUp_MessageCallback(const keyboard::Key::ConstPtr& msg);
    void keyDown_MessageCallback(const keyboard::Key::ConstPtr& msg);

    /* Parameters from ROS parameter server */
    double delta_speed, delta_steer;
    double max_speed, max_steer;

    /* Estimator periodic task */
    void PeriodicTask(void);

    /* Node state variables */
    bool			_arrowUp_pressed, _arrowDown_pressed;
    bool			_arrowLeft_pressed, _arrowRight_pressed;
    double			_speed_ref;
    double			_steer_ref;
    unsigned int                _state;

  public:
    double RunPeriod;

    void Prepare(void);

    void RunPeriodically(float Period);

    void Shutdown(void);
};

#endif /* KEYBOARD_TELEOP_H_ */
