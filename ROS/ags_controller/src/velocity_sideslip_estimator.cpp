#include "velocity_sideslip_estimator.h"

velocity_sideslip_estimator::velocity_sideslip_estimator(double P, double Kpv, double dT) : P(P), Kpv(Kpv), dT(dT),
    t(0.0), state(3), x_ref(0.0), y_ref(0.0), heading(0.0), vxP(0.0), vyP(0.0), v(0.0), w(0.0)
{
    // state = [ x, y, gamma ]

    // Initial state values
    state[0] = 0.0;
    state[1] = 0.0;
    state[2] = 0.0;
}

void velocity_sideslip_estimator::setInitialState(double x0, double y0, double gamma0)
{
    // Initial state values
    state[0] = x0;
    state[1] = y0;
    state[2] = gamma0;
}

void velocity_sideslip_estimator::estimate(double time, double &sideslip)
{
    // Integrate the observer
    using namespace std::placeholders;

    const double abs_err = 1.0e-10, rel_err = 1.0e-6;
    integrate_adaptive(make_controlled<error_stepper_type>(abs_err, rel_err),
                       std::bind(&velocity_sideslip_estimator::estimator_ode, this, _1, _2, _3), state, t, time, dT);

    // Compute sideslip
    sideslip = state[2]-heading;

    // Update time
    t = time;
}

void velocity_sideslip_estimator::estimator_ode(const state_type &state, state_type &dstate, double t)
{
    using namespace boost::math;

    // Actual state
    const double x     = state[0];
    const double y     = state[1];
    const double gamma = state[2];

    // Trajectory tracking controller equations
    vxP = Kpv*(x_ref-x);
    vyP = Kpv*(y_ref-y);

    // Linearizing controller equations
    v = vxP*std::cos(gamma)+vyP*std::sin(gamma);
    w = (vyP*std::cos(gamma)-vxP*std::sin(gamma))/P;

    // Vehicle equations
    dstate[0] = v*std::cos(gamma);  // dx
    dstate[1] = v*std::sin(gamma);  // dy
    dstate[2] = w;                  // dgamma
}
