#include "single_track_sim/test_single_track_sim.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  test_single_track_sim test_single_track_sim_node;
   
  test_single_track_sim_node.Prepare();
  
  test_single_track_sim_node.RunPeriodically(test_single_track_sim_node.RunPeriod);
  
  test_single_track_sim_node.Shutdown();
  
  return (0);
}

