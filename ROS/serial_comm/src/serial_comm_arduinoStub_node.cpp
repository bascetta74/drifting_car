#include "serial_comm/serial_comm_arduinoStub.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "serial_comm_arduinoStub");
  
  serial_comm_arduinoStub serial_comm_node;
   
  serial_comm_node.Prepare();
  
  serial_comm_node.RunPeriodically(serial_comm_node.RunPeriod);
   
  serial_comm_node.Shutdown();
  
  return (0);
}

