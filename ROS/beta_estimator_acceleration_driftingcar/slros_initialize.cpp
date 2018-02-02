#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "beta_estimator_acceleration_driftingcar";

// For Block beta_estimator_acceleration_driftingcar/Subscriber_To_Optitrack/Subscribe
SimulinkSubscriber<geometry_msgs::Pose2D, SL_Bus_beta_estimator_accelerati_Pose2D_de4qul> Sub_beta_estimator_acceleration_driftingcar_60;

// For Block beta_estimator_acceleration_driftingcar/Publisher/Publish1
SimulinkPublisher<std_msgs::Float64, SL_Bus_beta_estimator_accelerati_Float64_woid8t> Pub_beta_estimator_acceleration_driftingcar_57;

// For Block beta_estimator_acceleration_driftingcar/Parameters from parameter server/Get Parameter1
SimulinkParameterGetter<real64_T, double> ParamGet_beta_estimator_acceleration_driftingcar_74;

// For Block beta_estimator_acceleration_driftingcar/Parameters from parameter server/Get Parameter3
SimulinkParameterGetter<real64_T, double> ParamGet_beta_estimator_acceleration_driftingcar_72;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

