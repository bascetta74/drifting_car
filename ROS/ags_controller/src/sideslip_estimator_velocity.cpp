#include "sideslip_estimator_velocity.h"

#include <cmath>

velocity_sideslip_estimator::velocity_sideslip_estimator(double P, double Kp, double Ts) :
    P(P), Kp(Kp), x_ref(0.0), y_ref(0.0), theta_ref(0.0), x(0.0), y(0.0), gamma(0.0), vxP(0.0), vyP(0.0)
{
    // Discrete integrators
    dx = dy = dgamma = NULL;
    dx = new discrete_integrator_fwEul(1.0, Ts);
    dy = new discrete_integrator_fwEul(1.0, Ts);
    dgamma = new discrete_integrator_fwEul(1.0, Ts);
}

velocity_sideslip_estimator::velocity_sideslip_estimator(double P, double Kp, double Ts, double x0, double y0, double gamma0) :
    P(P), Kp(Kp), x_ref(0.0), y_ref(0.0), theta_ref(0.0), x(x0), y(y0), gamma(gamma0), vxP(0.0), vyP(0.0)
{
    // Discrete integrators
    dx = dy = dgamma = NULL;
    dx = new discrete_integrator_fwEul(1.0, Ts, x0);
    dy = new discrete_integrator_fwEul(1.0, Ts, y0);
    dgamma = new discrete_integrator_fwEul(1.0, Ts, gamma0);
}

velocity_sideslip_estimator::~velocity_sideslip_estimator()
{
    // Delete discrete integrator objects
    if (dx)
        delete dx;
    if (dy)
        delete dy;
    if (dgamma)
        delete dgamma;
}

void velocity_sideslip_estimator::execute()
{
    // Trajectory tracking controller equations
    vxP = Kp*(x_ref-x);
    vyP = Kp*(y_ref-y);

    // Linearizing controller equations
    double v, w;
    v = vxP*std::cos(gamma)+vyP*std::sin(gamma);
    w = (vyP*std::cos(gamma)-vxP*std::sin(gamma))/P;

    // Feedback linearisation and unicycle model
    dx->evaluate(v*std::cos(gamma), x);
    dy->evaluate(v*std::sin(gamma), y);
    dgamma->evaluate(w, gamma);
}

void velocity_sideslip_estimator::execute(int nStep)
{
    // Execute nStep times the estimator
    for (auto k=0; k<nStep; k++) {
        this->execute();
    }
}
