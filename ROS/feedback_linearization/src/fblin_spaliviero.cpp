#include "fblin_spaliviero.h"

#include <cmath>

fblin_spaliviero::fblin_spaliviero(double P_distance, double zero_speed_threshold)
{
 _p = P_distance;
 _zero_speed_thd = zero_speed_threshold;
}

fblin_spaliviero::~fblin_spaliviero()
{
 // Do nothing
}

void fblin_spaliviero::control_transformation(double vPx, double vPy, double& speed, double& steer)
{
 speed = vPx*cos(_beta+_psi)+vPy*sin(_beta+_psi);

 if (fabs(speed)<_zero_speed_thd)
    steer = 0;
 else
    steer = _m*speed/(_Cf*_p)*(vPy*cos(_beta+_psi)-vPx*sin(_beta+_psi)) + (_Cr/_Cf+1)*_beta - (_Cr/_Cf*_lr-_lf)/speed*_r;
}

void fblin_spaliviero::ouput_transformation(double& xP, double& yP)
{
 xP = _x + _p*cos(_beta+_psi);
 yP = _y + _p*sin(_beta+_psi);
}

void fblin_spaliviero::reference_transformation(double xref, double yref, double& xPref, double& yPref)
{
 xPref = xref + _p*cos(_beta+_psi);
 yPref = yref + _p*sin(_beta+_psi);
}