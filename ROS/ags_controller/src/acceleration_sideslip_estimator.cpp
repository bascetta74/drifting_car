#include "acceleration_sideslip_estimator.h"

acceleration_sideslip_estimator::acceleration_sideslip_estimator(double Kpa, double Kda, double Ta, double v_thd, double dT) :
    Kpa(Kpa), Kda(Kda), Ta(Ta), v_thd(v_thd), dT(dT), t(0.0), state(6), x_ref(0.0), y_ref(0.0), heading(0.0), ax(0.0), ay(0.0), w(0.0)
{
    // state = [ x, y, gamma, v, PD_state_ax, PD_state_ay ]

    // Initial state values
    state[0] = 0.0;
    state[1] = 0.0;
    state[2] = 0.0;
    state[3] = 0.0;
    state[4] = 0.0;
    state[5] = 0.0;
}

void acceleration_sideslip_estimator::setInitialState(double x0, double y0, double gamma0, double v0)
{
    // Initial state values
    state[0] = x0;
    state[1] = y0;
    state[2] = gamma0;
    state[3] = v0;
    state[4] = 0.0;
    state[5] = 0.0;
}

void acceleration_sideslip_estimator::estimate(double time, double &sideslip)
{
    // Integrate the observer
    using namespace std::placeholders;

    const double abs_err = 1.0e-10, rel_err = 1.0e-6;
    integrate_adaptive(make_controlled<error_stepper_type>(abs_err, rel_err),
                       std::bind(&acceleration_sideslip_estimator::estimator_ode, this, _1, _2, _3), state, t, time, dT);

    // Compute sideslip
    sideslip = state[2]-heading;

    // Update time
    t = time;
}

void acceleration_sideslip_estimator::estimator_ode(const state_type &state, state_type &dstate, double t)
{
    using namespace boost::math;

    // Actual state
    const double x      = state[0];
    const double y      = state[1];
    const double gamma  = state[2];
    const double v      = state[3];
    const double xPD_ax = state[4];
    const double xPD_ay = state[5];

    // Trajectory tracking controller equations
    dstate[4] = -xPD_ax/Ta+(x_ref-x)/Ta;  // dxPD_ax
    dstate[5] = -xPD_ay/Ta+(y_ref-y)/Ta;  // dxPD_ay
    ax = -Kda/Ta*xPD_ax+(Kpa+Kda/Ta)*(x_ref-x);
    ay = -Kda/Ta*xPD_ay+(Kpa+Kda/Ta)*(y_ref-y);

    // Linearizing controller equations
    dstate[3] = ax*std::cos(gamma)+ay*std::sin(gamma);  // dv
    if (std::abs(v)<v_thd) {
        w = 0.0;
    }
    else {
        w = (ay*std::cos(gamma)-ax*std::sin(gamma))/v;
    }

    // Vehicle equations
    dstate[0] = v*std::cos(gamma);  // dx
    dstate[1] = v*std::sin(gamma);  // dy
    dstate[2] = w;                  // dgamma
}
