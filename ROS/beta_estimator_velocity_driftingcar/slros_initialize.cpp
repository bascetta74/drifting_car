#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "beta_estimator_velocity_driftingcar";

// For Block beta_estimator_velocity_driftingcar/Subscriber_To_Optitrack/Subscribe
SimulinkSubscriber<geometry_msgs::Pose2D, SL_Bus_beta_estimator_velocity_d_Pose2D_nguzzk> Sub_beta_estimator_velocity_driftingcar_77;

// For Block beta_estimator_velocity_driftingcar/Publisher/Publish1
SimulinkPublisher<std_msgs::Float64, SL_Bus_beta_estimator_velocity_driftingcar_std_msgs_Float64> Pub_beta_estimator_velocity_driftingcar_75;

// For Block beta_estimator_velocity_driftingcar/Parameters from parameter server/Get Parameter3
SimulinkParameterGetter<real64_T, double> ParamGet_beta_estimator_velocity_driftingcar_63;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

