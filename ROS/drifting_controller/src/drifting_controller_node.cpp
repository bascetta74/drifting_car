#include "drifting_controller/drifting_controller.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, NAME_OF_THIS_NODE);
  
    drifting_controller drifting_controller_node;
  
    drifting_controller_node.Prepare();
  
    drifting_controller_node.RunPeriodically(drifting_controller_node.RunPeriod);
  
    drifting_controller_node.Shutdown();
  
    return (0);
}
