#ifndef SERIAL_COMM_H_
#define SERIAL_COMM_H_

#include <serial/serial.h> // Based on serial class http://docs.ros.org/jade/api/serial/html/index.html

#include "telemetry_protocol.h"
#include "car_msgs/car_cmd.h"
#include "car_msgs/wheel_spd.h"
#include "ros/ros.h"

#define USE_KEYBOARD_TELEOP true
/* Used only if the actual values are not retrieved from the ROS parameter
 * server */

//#define TEST_COMM
/* Activate this define to test data received by serial_comm (published back on
 * test topics) */

#define NAME_OF_THIS_NODE "serial_comm"

class serial_comm {
private:
  ros::NodeHandle Handle;

  /* ROS topics */
  ros::Subscriber controllerCommand_subscriber;
  ros::Publisher radioCommand_publisher;
  ros::Publisher wheelSpeed_publisher;

#ifdef TEST_COMM
  ros::Publisher controllerCommand_publisher;
#endif

  /* ROS topic callbacks */
  void controllerCommand_MessageCallback(const car_msgs::car_cmd::ConstPtr &msg);

  /* Node periodic task */
  void PeriodicTask(void);

  /* State machine state type */
  typedef enum States {
    SAFE = 0,
    MANUAL = 1,
    AUTOMATIC = 2,
    HALT = 3
  } statemachine_state;
  typedef struct {
    statemachine_state state;
    unsigned char info;
  } state_info;

  /* Conversion functions */
  bool us_to_SIunits(unsigned int value_us, double &value_SIunits,
                     std::vector<int> us_range,
                     std::vector<double> SIunits_range);
  bool SIunits_to_us(unsigned int &value_us, double value_SIunits,
                     std::vector<int> us_range,
                     std::vector<double> SIunits_range);

  /* Node state variables */
  double _steer_ref;
  double _speed_ref;
  double _wheel_speed;
  state_info _statemachine;
  bool _enteringSafe, _enteringManual, _enteringAutomatic, _enteringHalt;

  /* Serial message */
  serial::Serial *_serial;
  telemetryProtocol *_telemetry;
  telemetry_message message;
  uint16_t _message_number;

  /* Node parameters */
  std::string _serial_port;
  int _baudrate;
  int _timeout;
  int _message_size;
  bool _steer_invert_command;
  std::vector<int> _steer_us_range, _speed_us_range;
  std::vector<double> _steer_rad_range, _speed_mps_range;

public:
  void Prepare(void);

  void RunPeriodically(void);

  void Shutdown(void);
};

#endif /* SERIAL_COMM_H_ */
