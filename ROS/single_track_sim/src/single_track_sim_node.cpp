#include "single_track_sim/single_track_sim.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  single_track_sim single_track_sim_node;
   
  single_track_sim_node.Prepare();
  
  single_track_sim_node.RunPeriodically();
  
  single_track_sim_node.Shutdown();
  
  return (0);
}

