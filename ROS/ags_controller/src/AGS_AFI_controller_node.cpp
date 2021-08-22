#include "ags_controller/AGS_AFI_controller.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  AGS_AFI_controller AGS_AFI_controller_node;
   
  AGS_AFI_controller_node.Prepare();
  
  AGS_AFI_controller_node.RunPeriodically(AGS_AFI_controller_node.RunPeriod);
   
  AGS_AFI_controller_node.Shutdown();
  
  return (0);
}

