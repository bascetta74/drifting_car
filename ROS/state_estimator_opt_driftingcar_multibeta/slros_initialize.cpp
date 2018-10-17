#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "state_estimator_opt_driftingcar_multibeta";

// For Block state_estimator_opt_driftingcar_multibeta/Subscriber_To_Optitrack/Subscribe
SimulinkSubscriber<geometry_msgs::PoseStamped, SL_Bus_state_estimator_opt_drift_PoseStamped_8suobb> Sub_state_estimator_opt_driftingcar_multibeta_132;

// For Block state_estimator_opt_driftingcar_multibeta/Publisher/Publish1
SimulinkPublisher<std_msgs::Float64, SL_Bus_state_estimator_opt_drift_Float64_pfh567> Pub_state_estimator_opt_driftingcar_multibeta_170;

// For Block state_estimator_opt_driftingcar_multibeta/Publisher/Publish2
SimulinkPublisher<std_msgs::Float64, SL_Bus_state_estimator_opt_drift_Float64_pfh567> Pub_state_estimator_opt_driftingcar_multibeta_171;

// For Block state_estimator_opt_driftingcar_multibeta/Publisher/Publish3
SimulinkPublisher<std_msgs::Float64, SL_Bus_state_estimator_opt_drift_Float64_pfh567> Pub_state_estimator_opt_driftingcar_multibeta_172;

// For Block state_estimator_opt_driftingcar_multibeta/Publisher/Publish4
SimulinkPublisher<std_msgs::Float64, SL_Bus_state_estimator_opt_drift_Float64_pfh567> Pub_state_estimator_opt_driftingcar_multibeta_268;

// For Block state_estimator_opt_driftingcar_multibeta/Publisher/Publish5
SimulinkPublisher<std_msgs::Float64, SL_Bus_state_estimator_opt_drift_Float64_pfh567> Pub_state_estimator_opt_driftingcar_multibeta_318;

// For Block state_estimator_opt_driftingcar_multibeta/Publisher/Publish6
SimulinkPublisher<std_msgs::Float64, SL_Bus_state_estimator_opt_drift_Float64_pfh567> Pub_state_estimator_opt_driftingcar_multibeta_322;

// For Block state_estimator_opt_driftingcar_multibeta/Publisher/Publish7
SimulinkPublisher<std_msgs::Float64, SL_Bus_state_estimator_opt_drift_Float64_pfh567> Pub_state_estimator_opt_driftingcar_multibeta_326;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

