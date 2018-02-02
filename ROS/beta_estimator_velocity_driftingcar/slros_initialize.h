#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block beta_estimator_velocity_driftingcar/Subscriber_To_Optitrack/Subscribe
extern SimulinkSubscriber<geometry_msgs::Pose2D, SL_Bus_beta_estimator_velocity_d_Pose2D_nguzzk> Sub_beta_estimator_velocity_driftingcar_77;

// For Block beta_estimator_velocity_driftingcar/Publisher/Publish1
extern SimulinkPublisher<std_msgs::Float64, SL_Bus_beta_estimator_velocity_driftingcar_std_msgs_Float64> Pub_beta_estimator_velocity_driftingcar_75;

// For Block beta_estimator_velocity_driftingcar/Parameters from parameter server/Get Parameter3
extern SimulinkParameterGetter<real64_T, double> ParamGet_beta_estimator_velocity_driftingcar_63;

void slros_node_init(int argc, char** argv);

#endif
