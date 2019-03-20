#include "fblin_lopez.h"

#include <cmath>
#include <stdexcept>


fblin_lopez::fblin_lopez(double P_distance, double sampling_time)
{
 // Check parameter consistency
 if (P_distance<=0.0)
        throw std::invalid_argument("The distance of point P should be strictly greater then zero");
 if (sampling_time==0.0)
        throw std::invalid_argument("Feedback linearization sampling time cannot be zero");

 _p = P_distance;
 _Ts = sampling_time;

 _delta = 0.0;
}

fblin_lopez::~fblin_lopez()
{
 // Do nothing
}

void fblin_lopez::control_transformation(double vPx, double vPy, double& speed, double& steer)
{
 // Updating integrator state
 _delta += _Ts*((vPy*cos(_beta+_psi)-vPx*sin(_beta+_psi)-_lf*_r*cos(_beta))/(_p*cos(_beta-_delta))-_r);
 
 steer = _delta;

 speed = (vPx*cos(_delta+_psi)+vPy*sin(_delta+_psi)+_lf*_r*sin(_delta))/cos(_beta-_delta);
}

void fblin_lopez::ouput_transformation(double& xP, double& yP)
{
 xP = _x + _lf*cos(_psi) + _p*cos(_steer+_psi);
 yP = _y + _lf*sin(_psi) + _p*sin(_steer+_psi);
}

void fblin_lopez::reference_transformation(double xref, double yref, double& xPref, double& yPref)
{
 xPref = xref + _lf*cos(_psi) + _p*cos(_steer+_psi);
 yPref = yref + _lf*sin(_psi) + _p*sin(_steer+_psi);
}