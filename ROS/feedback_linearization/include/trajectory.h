#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

void circle_position(double& x, double& y, double t, double xc, double yc, double R, double w, double phi);
void squircle_position(double& x, double& y, double t, double xc, double yc, double R, double w, double phi);

void stepSeq1_velocity(double& vPx, double& vPy, double t, double speed);
void stepSeq2_velocity(double& vPx, double& vPy, double t, double speed);

#endif /* TRAJECTORY_H_ */