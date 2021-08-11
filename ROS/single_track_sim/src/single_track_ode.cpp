#include "single_track_sim/single_track_ode.h"

#include <boost/math/special_functions/sign.hpp>

single_track_ode::single_track_ode(double deltaT, tyreModel tyre_model, actuatorModel actuator_model) : dt(deltaT),
    t(0.0), state(7), Vx(0.0), delta_ref(0.0), delta(0.0), alphaf(0.0), alphar(0.0), Fyf(0.0), Fyr(0.0),
    vehicleParams_set(false), steeringActuatorParams_set(false)
{
    // state = [ r, Vy, x, y, psi, steer_pos, steer_vel ]

    // Initial state values
    state[0] = 0.0;
    state[1] = 0.0;
    state[2] = 0.0;
    state[3] = 0.0;
    state[4] = 0.0;
    state[5] = 0.0;
    state[6] = 0.0;

    // Tyre and actuator models
    this->tyre_model = tyre_model;
    this->steeringActuator_model = actuator_model;
}

void single_track_ode::setInitialState(double r0, double Vy0, double x0, double y0, double psi0)
{
    // Initial state values
    state[0] = r0;
    state[1] = Vy0;
    state[2] = x0;
    state[3] = y0;
    state[4] = psi0;
    state[5] = 0.0;
    state[6] = 0.0;
}

void single_track_ode::setVehicleParams(double m, double a, double b, double Cf, double Cr, double mu, double Iz)
{
    // Initialize vehicle parameters
    this->m = m;
    this->a = a;
    this->b = b;
    this->Cf = Cf;
    this->Cr = Cr;
    this->mu = mu;
    this->Iz = Iz;

    vehicleParams_set = true;
}

void single_track_ode::setSteeringActuatorParams(double gain, double frequency, double damping, int delay)
{
    // Initialize steering actuator parameters
    mu_steer  = gain;
    wn_steer  = frequency;
    csi_steer = damping;
    tau_steer = delay;      // Multiples of dt

    steeringActuatorParams_set = true;

    // Initialize the FIFO to represent the delay
    for (auto k=0; k<tau_steer; k++) {
        delta_ref_FIFO.push(0.0);
    }
}

void single_track_ode::setReferenceCommands(double velocity, double steer)
{
    Vx = velocity;

    switch (steeringActuator_model)
    {
        case IDEAL:
            delta_ref = steer;
            break;

        case REAL:
            delta_ref_FIFO.push(steer);
            break;

        default:
            throw std::invalid_argument( "Uknown steering actuator model!" );
            break;
    }
}

void single_track_ode::integrate()
{
    // Check vehicle parameters are set
    if (!vehicleParams_set) {
        throw std::invalid_argument( "Vehicle parameters not set!" );
    }

    // Check steering actuator parameters are set
    if ((!steeringActuatorParams_set) && (steeringActuator_model!=single_track_ode::IDEAL)) {
        throw std::invalid_argument( "Steering actuator parameters not set!" );
    }

    // Integrate for one step ahead
    using namespace std::placeholders;
    stepper.do_step(std::bind(&single_track_ode::vehicle_ode, this, _1, _2, _3), state, t, dt);

    // Update time and steering
    t += dt;

    if (steeringActuator_model==REAL) {
        delta_ref_FIFO.pop();
    }
}

void single_track_ode::vehicle_ode(const state_type &state, state_type &dstate, double t)
{
    using namespace boost::math;

    // Actual state
    const double r   = state[0];
    const double Vy  = state[1];
    const double x   = state[2];
    const double y   = state[3];
    const double psi = state[4];
    const double steer_pos = state[5];
    const double steer_vel = state[6];

    // Actuator model
    switch (steeringActuator_model)
    {
        case IDEAL:
            delta = delta_ref;

            // These states are not used
            dstate[5] = 0.0;
            dstate[6] = 0.0;
            break;

        case REAL:
            // Check the delta_ref FIFO is not empty
            if (delta_ref_FIFO.empty()) {
                throw std::length_error( "dela_ref FIFO is empty!");
            }

            // Compute delta command
            delta = mu_steer*std::pow(wn_steer,2.0)*steer_pos;

            // Update the actuator model state
            dstate[5] = steer_vel;
            dstate[6] = -std::pow(wn_steer,2.0)*steer_pos-2*csi_steer*wn_steer*steer_vel+delta_ref_FIFO.front();
            break;

        default:
            throw std::invalid_argument( "Uknown steering actuator model!" );
            break;
    }

    // Slip angles
    if (std::abs(Vx)<=0.01) {
        alphaf = 0.0;
        alphar = 0.0;
    }
    else {
        alphaf = std::atan((Vy+a*r)/Vx)-delta;
        alphar = std::atan((Vy-b*r)/Vx);
    }

    // Tyre forces
    double Fzf = m*9.81*b/(a+b);
    double Fzr = m*9.81*a/(a+b);
    double zf = std::tan(alphaf);
    double zr = std::tan(alphar);
    double zf_sl = 3*mu*Fzf/Cf;
    double zr_sl = 3*mu*Fzr/Cr;

    switch (tyre_model)
    {
        case LINEAR:
            Fyf = -Cf*alphaf;
            Fyr = -Cr*alphar;
            break;

        case FIALA_WITH_SATURATION:
            if (std::abs(zf)<zf_sl) {
                Fyf = Cf*zf*(-1+std::abs(zf)/zf_sl-std::pow(zf,2.0)/(3*std::pow(zf_sl,2.0)));
            }
            else {
                Fyf = -mu*Fzf*(double)sign(alphaf);
            }
            if (std::abs(zr)<zr_sl) {
                Fyr = Cr*zr*(-1+std::abs(zr)/zr_sl-std::pow(zr,2.0)/(3*std::pow(zr_sl,2.0)));
            }
            else {
                 Fyr = -mu*Fzr*(double)sign(alphar);
            }
            break;

        case FIALA_WITHOUT_SATURATION:
            Fyf = Cf*zf*(-1+std::abs(zf)/zf_sl-std::pow(zf,2.0)/(3*std::pow(zf_sl,2.0)));
            Fyr = Cr*zr*(-1+std::abs(zr)/zr_sl-std::pow(zr,2.0)/(3*std::pow(zr_sl,2.0)));
            break;

        default:
            Fyf = 0.0;
            Fyr = 0.0;

            throw std::invalid_argument( "Uknown tyre model!" );
            break;
    }

    // Vehicle equations
    dstate[0] = (a*Fyf*cos(delta)-b*Fyr)/Iz;        // dr
    dstate[1] = (Fyf*cos(delta)+Fyr)/m-r*Vx;        // dVy
    dstate[2] = cos(psi)*Vx-sin(psi)*Vy;            // dx
    dstate[3] = sin(psi)*Vx+cos(psi)*Vy;            // dy
    dstate[4] = r;                                  // r

    // Other variables
    ay = dstate[1]+r*Vx;
    if (std::abs(Vx)<=0.01) {
        sideslip = 0.0;
    }
    else {
        sideslip = std::atan2(Vy,Vx);
    }
}
