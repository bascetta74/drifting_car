#include "trajectory.h"

#include <cmath>


void circle_position(double& x, double& y, double t, double xc, double yc, double R, double w, double phi)
{
    x = xc + R*cos(w*t+phi);
    y = yc + R*sin(w*t+phi);
}

void squircle_position(double& x, double& y, double t, double xc, double yc, double R, double w, double phi)
{
    x = xc + R*cos(w*t+phi)/pow(pow(cos(w*t+phi),8.0)+pow(sin(w*t+phi),8.0),0.125);
    y = yc + R*sin(w*t+phi)/pow(pow(cos(w*t+phi),8.0)+pow(sin(w*t+phi),8.0),0.125);
}

void stepSeq1_velocity(double& vPx, double& vPy, double t, double speed)
{
    if (t<=2.0)
    {
      vPx = 0.0;
      vPy = 0.5;
    }
    else if (t<=3.0)
    {
      vPx = 0.0;
      vPy = speed;
    }
    else if (t<=4.5)
    {
      vPx = -speed;
      vPy =  speed;
    }
    else if (t<=7.0)
    {
      vPx = -speed;
      vPy =  0.0;
    }
    else
    {
      vPx = 0.0;
      vPy = 0.0;
    }
}

void stepSeq2_velocity(double& vPx, double& vPy, double t, double speed)
{
    if (t<=2.0)
    {
      vPx = -0.5;
      vPy =  0.0;
    }
    else if (t<=3.0)
    {
      vPx = -speed;
      vPy =  0.0;
    }
    else if (t<=4.5)
    {
      vPx = -speed;
      vPy = -speed;
    }
    else if (t<=7.0)
    {
      vPx =  0.0;
      vPy = -speed;
    }
    else
    {
      vPx = 0.0;
      vPy = 0.0;
    }
}
