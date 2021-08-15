#include "ags_controller/AFI_controller.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  AFI_controller AFI_controller_node;
   
  AFI_controller_node.Prepare();
  
  AFI_controller_node.RunPeriodically(AFI_controller_node.RunPeriod);
   
  AFI_controller_node.Shutdown();
  
  return (0);
}

