#include "keyboard_teleop/keyboard_teleop.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, NAME_OF_THIS_NODE);
  
    keyboard_teleop keyboard_teleop_node;
   
    keyboard_teleop_node.Prepare();
  
    keyboard_teleop_node.RunPeriodically(keyboard_teleop_node.RunPeriod);
   
    keyboard_teleop_node.Shutdown();
  
    return (0);
}

