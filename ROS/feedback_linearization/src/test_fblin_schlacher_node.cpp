#include "feedback_linearization/test_fblin_schlacher.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  test_fblin_schlacher test_feedback_linearization_node;
   
  test_feedback_linearization_node.Prepare();
  
  test_feedback_linearization_node.RunPeriodically(test_feedback_linearization_node.RunPeriod);
   
  test_feedback_linearization_node.Shutdown();
  
  return (0);
}

