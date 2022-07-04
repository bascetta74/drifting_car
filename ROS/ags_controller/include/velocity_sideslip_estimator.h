#include <boost/numeric/odeint.hpp>

using namespace boost::numeric::odeint;

typedef std::vector<double> state_type;
typedef runge_kutta_dopri5<state_type> error_stepper_type;

// Implementation of the closed-loop velocity based sideslip estimator

class velocity_sideslip_estimator
{
public:
    velocity_sideslip_estimator(double P, double Kpv, double dT);

    void setInitialState(double x0, double y0, double gamma0);
    void setVehiclePose(double x, double y, double heading) { this->x_ref = x; this->y_ref = y; this->heading = heading; };

    void estimate(double time, double &sideslip);
    
    void getFbLControl(double &vPx, double &vPy) { vPx = vxP; vPy = vyP; };
    void getUnicycleControl(double &v, double &w) { v = this->v; w = this->w; };
    void getUnicycleState(double &x, double &y, double &gamma) { x = state[0]; y = state[1]; gamma = state[2]; };
    void getTrackingError(double &xerr, double &yerr) { xerr = x_ref-state[0]; yerr = y_ref-state[1]; };
    void getTime(double &time) { time = t; };

private:
    // Estimator and integrator variables
    double t, dT;
    double P, Kpv;
    double x_ref, y_ref, heading;
    double vxP, vyP, v, w;

    state_type state;

    // ODE function
    void estimator_ode(const state_type &state, state_type &dstate, double t);
};
