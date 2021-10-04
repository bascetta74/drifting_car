#include "singletrack_sim/test_singletrack_sim.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  test_singletrack_sim test_singletrack_sim_node;
   
  test_singletrack_sim_node.Prepare();
  
  test_singletrack_sim_node.RunPeriodically(test_singletrack_sim_node.RunPeriod);
  
  test_singletrack_sim_node.Shutdown();
  
  return (0);
}

