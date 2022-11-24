#include "sideslip_estimator_acceleration.h"

#include <cmath>

acceleration_sideslip_estimator::acceleration_sideslip_estimator(double Kp, double Kd, double T, double v_thd, double Ts) :
    v_thd(v_thd), x_ref(0.0), y_ref(0.0), theta_ref(0.0), x(0.0), y(0.0), gamma(0.0), ax(0.0), ay(0.0)
{
    // Discrete integrators
    PD_ax = PD_ay = NULL;
    dx = dy = dgamma = dv = NULL;
    dx = new discrete_integrator_fwEul(1.0, Ts);
    dy = new discrete_integrator_fwEul(1.0, Ts);
    dgamma = new discrete_integrator_fwEul(1.0, Ts);
    dv = new discrete_integrator_fwEul(1.0, Ts);
    PD_ax = new PID_controller(Kp, Kd/Kp, Kd/(Kp*T), Ts, -1000, 1000);
    PD_ay = new PID_controller(Kp, Kd/Kp, Kd/(Kp*T), Ts, -1000, 1000);
}

acceleration_sideslip_estimator::acceleration_sideslip_estimator(double Kp, double Kd, double T, double v_thd, double Ts,
                                                                 double x0, double y0, double gamma0, double v0) :
        v_thd(v_thd), x_ref(0.0), y_ref(0.0), theta_ref(0.0), x(x0), y(y0), gamma(gamma0), ax(0.0), ay(0.0)
{
    // Discrete integrators
    PD_ax = PD_ay = NULL;
    dx = dy = dgamma = dv = NULL;
    dx = new discrete_integrator_fwEul(1.0, Ts, x0);
    dy = new discrete_integrator_fwEul(1.0, Ts, y0);
    dgamma = new discrete_integrator_fwEul(1.0, Ts, gamma0);
    dv = new discrete_integrator_fwEul(1.0, Ts, v0);
    PD_ax = new PID_controller(Kp, Kd/Kp, Kd/(Kp*T), Ts, -1.0e+16, 1.0e+16);
    PD_ay = new PID_controller(Kp, Kd/Kp, Kd/(Kp*T), Ts, -1.0e+16, 1.0e+16);
}

acceleration_sideslip_estimator::~acceleration_sideslip_estimator()
{
    // Delete discrete integrator objects
    if (dx)
        delete dx;
    if (dy)
        delete dy;
    if (dgamma)
        delete dgamma;
    if (dv)
        delete dv;
    if (PD_ax)
        delete PD_ax;
    if (PD_ay)
        delete PD_ay;
}

void acceleration_sideslip_estimator::execute()
{
    // Trajectory tracking controller equations
    PD_ax->evaluate(x, x_ref, ax);
    PD_ay->evaluate(y, y_ref, ay);

    // Linearizing controller equations
    double v, w;
    dv->evaluate(ax*std::cos(gamma)+ay*std::sin(gamma), v);
    if (std::abs(v)<v_thd) {
        w = 0.0;
    }
    else {
        w = (ay*std::cos(gamma)-ax*std::sin(gamma))/v;
    }

    // Feedback linearisation and unicycle model
    dx->evaluate(v*std::cos(gamma), x);
    dy->evaluate(v*std::sin(gamma), y);
    dgamma->evaluate(w, gamma);
}

void acceleration_sideslip_estimator::execute(int nStep)
{
    // Execute nStep times the estimator
    for (auto k=0; k<nStep; k++) {
        this->execute();
    }
}
