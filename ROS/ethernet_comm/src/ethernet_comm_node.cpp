#include "ethernet_comm/ethernet_comm.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "ethernet_comm");

  ethernet_comm ethernet_comm_node;

  ethernet_comm_node.Prepare();

  ethernet_comm_node.RunPeriodically();

  ethernet_comm_node.Shutdown();

  return (0);
}
