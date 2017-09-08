#ifndef SERIAL_COMM_ARDUINOSTUB_H_
#define SERIAL_COMM_ARDUINOSTUB_H_

#include "ros/ros.h"
#include <serial/serial.h>

#define RUN_PERIOD			0.001
#define SERIAL_PORT			"/dev/pts/5"
#define BAUDRATE			115200
#define TIMEOUT				1

class serial_comm_arduinoStub
{
  private: 
    ros::NodeHandle Handle;
    
    // ros::Subscriber Subscriber;
    
    ros::Publisher _Publisher;
    
    serial::Serial *_serial;
     
    // param_type ParamVar;
     
    // void MessageCallback(const msg_pkg::msg_type::ConstPtr& msg);

    void PeriodicTask(void);
     
  public:
    double RunPeriod;

    void Prepare(void);
    void RunPeriodically(float Period);
    void Shutdown(void);
};

#endif /* SERIAL_COMM_ARDUINOSTUB_H_ */
