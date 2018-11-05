#ifndef ETHERNET_COMM_H_
#define ETHERNET_COMM_H_

#include "car_msgs/car_cmd.h"
#include "car_msgs/arduino_telemetry.h"
#include "ros/ros.h"

#include <arpa/inet.h>
#include <sys/socket.h>

#define USE_KEYBOARD_TELEOP true
/* Used only if the actual values are not retrieved from the ROS parameter
 * server */

#define NAME_OF_THIS_NODE "ethernet_comm"

class ethernet_comm {
private:
  ros::NodeHandle Handle;

  /* ROS topics */
  ros::Subscriber controllerCommand_subscriber;
  ros::Publisher radioCommand_publisher;
  ros::Publisher wheelSpeed_publisher;
  ros::Publisher arduinoTelemetry_publisher;

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

  /* Checksum functions */
  bool checksum_verify();
  void checksum_calculate();

  /* Conversion functions */
  bool us_to_SIunits(uint16_t value_us, double &value_SIunits,
                     std::vector<int> us_range,
                     std::vector<double> SIunits_range);
  bool SIunits_to_us(uint16_t &value_us, double value_SIunits,
                     std::vector<int> us_range,
                     std::vector<double> SIunits_range);

  /* Node state variables */
  double _steer_ref;
  double _speed_ref;
  double _wheel_speed;
  state_info _statemachine;
  bool _enteringSafe, _enteringManual, _enteringAutomatic, _enteringHalt;
  unsigned long _time;

  /* Communication */
  struct sockaddr_in sockaddr_server;
  uint8_t *_message_buffer;
  std::string _server_ip;
  socklen_t sockaddr_len;
  int _server_port;
  int _message_size;
  int _socket;

  /* Node parameters */
  std::vector<int> _steer_us_range, _speed_us_range;
  std::vector<double> _steer_rad_range, _speed_mps_range;

public:
  void Prepare(void);

  void RunPeriodically(void);

  void Shutdown(void);
};

#endif /* ETHERNET_COMM_H_ */
