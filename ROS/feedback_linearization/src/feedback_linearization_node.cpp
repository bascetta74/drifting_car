#include "feedback_linearization/feedback_linearization.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  feedback_linearization feedback_linearization_node;
   
  feedback_linearization_node.Prepare();
  
  feedback_linearization_node.RunPeriodically(feedback_linearization_node.RunPeriod);
   
  feedback_linearization_node.Shutdown();
  
  return (0);
}

