#include "fblin_lopez_I.h"

#include <cmath>


fblin_lopez_I::fblin_lopez_I(double P_distance, double sampling_time, double zero_speed_threshold)
{
 _p = P_distance;
 _Ts = sampling_time;
 _zero_speed_thd = zero_speed_threshold;

 _v = 0.0;
}

fblin_lopez_I::~fblin_lopez_I()
{
 // Do nothing
}

void fblin_lopez_I::control_transformation(double vPx, double vPy, double& speed, double& steer)
{
 // Updating integrator state
 _v += _Ts*(vPx*cos(_beta+_psi)+vPy*sin(_beta+_psi)-_v)/_p;
 
 if (fabs(_v)<_zero_speed_thd)
    steer = 0;
 else
    steer = _m/(_Cf*_p)*(vPy*cos(_beta+_psi)-vPx*sin(_beta+_psi)) + (_Cr/_Cf+1)*_beta - (_Cr/_Cf*_lr-_lf)/_v*_r;

 speed = _v*cos(_beta);
}

void fblin_lopez_I::ouput_transformation(double& xP, double& yP)
{
 xP = _x + _p*_absVel*cos(_beta+_psi);
 yP = _y + _p*_absVel*sin(_beta+_psi);
}

void fblin_lopez_I::reference_transformation(double xref, double yref, double& xPref, double& yPref)
{
 xPref = xref + _p*_absVel*cos(_beta+_psi);
 yPref = yref + _p*_absVel*sin(_beta+_psi);
}