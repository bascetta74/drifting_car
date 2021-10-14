#include "fblin_schlacher.h"

#include <cmath>
#include <stdexcept>


fblin_schlacher::fblin_schlacher()
{
 // Do nothing
}

fblin_schlacher::~fblin_schlacher()
{
 // Do nothing
}

void fblin_schlacher::control_transformation(double w1, double w2, double& Fxr, double& steer)
{
 // Intermediate variables
 double l = (_lf+_lr)/(_m*_lf);
 double F = -(_lf*_V)/_Iz*std::cos(_beta)+(_Cr*l)/(_Iz*_m*_V*std::cos(_beta))*(_lf*_lr*_m-_Iz);
 double R = (_lr*_V)/_Iz*std::cos(_beta)-(_Cr*l)/(_Iz*_m*_V*std::cos(_beta))*(_Iz+_m*std::pow(_lr,2.0));
 double H = -_r*w1+(_Cr*l)/std::pow(_V*std::cos(_beta),2.0)*(_V*w1*std::sin(_beta)-_lr*_r*w1-std::pow(_V,2.0)*_r*std::pow(std::sin(_beta),2.0)+_r*std::pow(_V,2.0));

 // Forces and outputs
 double Fry = _Cr*(_lr*_r-_V*std::sin(_beta))/(_V*std::cos(_beta));
 steer = (w2-Fry*R-H)/(F*_Cf)+(_V*std::sin(_beta)+_lf*_r)/(_V*std::cos(_beta));
 double Ffy = _Cf*(steer-(_V*std::sin(_beta)+_lf*_r)/(_V*std::cos(_beta)));
 Fxr   = _m*w1+Ffy*steer-_V*_r*_m*std::sin(_beta);
}

void fblin_schlacher::ouput_transformation(double& z1, double& z2)
{
 z1 = _V*std::cos(_beta);
 z2 = _V*std::sin(_beta)-_Iz*_r/(_m*_lf);
}
