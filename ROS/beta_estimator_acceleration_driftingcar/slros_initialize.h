#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block beta_estimator_acceleration_driftingcar/Subscriber_To_Optitrack/Subscribe
extern SimulinkSubscriber<geometry_msgs::Pose2D, SL_Bus_beta_estimator_accelerati_Pose2D_de4qul> Sub_beta_estimator_acceleration_driftingcar_60;

// For Block beta_estimator_acceleration_driftingcar/Publisher/Publish1
extern SimulinkPublisher<std_msgs::Float64, SL_Bus_beta_estimator_accelerati_Float64_woid8t> Pub_beta_estimator_acceleration_driftingcar_57;

// For Block beta_estimator_acceleration_driftingcar/Parameters from parameter server/Get Parameter1
extern SimulinkParameterGetter<real64_T, double> ParamGet_beta_estimator_acceleration_driftingcar_74;

// For Block beta_estimator_acceleration_driftingcar/Parameters from parameter server/Get Parameter3
extern SimulinkParameterGetter<real64_T, double> ParamGet_beta_estimator_acceleration_driftingcar_72;

void slros_node_init(int argc, char** argv);

#endif
