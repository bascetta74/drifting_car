#include "beta_estimator/beta_estimator.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  beta_estimator beta_estimator_node;
   
  beta_estimator_node.Prepare();
  
  beta_estimator_node.RunPeriodically(beta_estimator_node.RunPeriod);
   
  beta_estimator_node.Shutdown();
  
  return (0);
}

