#include "singletrack_vxvy_force_ode.h"

#include <boost/math/special_functions/sign.hpp>

singletrack_vxvy_force_ode::singletrack_vxvy_force_ode(double deltaT, tyreModel tyre_model, actuatorModel actuator_model, double vx_thd) : dt(deltaT),
    t(0.0), state(10), delta_ref(0.0), delta(0.0), Fxr_ref(0.0), alphaf(0.0), alphar(0.0), Fyf(0.0), Fyr(0.0),
    vehicleParams_set(false), actuatorParams_set(false)
{
    // state = [ r, Vx, Vy, x, y, psi, steer_pos, steer_vel, force_pos, force_vel ]

    // Initial state values
    state[0] = 0.0;
    state[1] = 0.0;
    state[2] = 0.0;
    state[3] = 0.0;
    state[4] = 0.0;
    state[5] = 0.0;
    state[6] = 0.0;
    state[7] = 0.0;
    state[8] = 0.0;
    state[9] = 0.0;

    // Tyre and actuator models
    this->tyre_model = tyre_model;
    this->actuator_model = actuator_model;

    // Vx threshold for slip/sideslip computation
    this->vx_thd = vx_thd;
}

void singletrack_vxvy_force_ode::setInitialState(double r0, double Vx0, double Vy0, double x0, double y0, double psi0)
{
    // Initial state values
    state[0] = r0;
    state[1] = Vx0;
    state[2] = Vy0;
    state[3] = x0;
    state[4] = y0;
    state[5] = psi0;
    state[6] = 0.0;
    state[7] = 0.0;
    state[8] = 0.0;
    state[9] = 0.0;
}

void singletrack_vxvy_force_ode::setVehicleParams(double m, double a, double b, double Cf, double Cr, double mu, double Iz)
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

void singletrack_vxvy_force_ode::setActuatorParams(double steer_gain, double steer_frequency, double steer_damping, int steer_delay, double steer_initial,
                                                   double force_gain, double force_frequency, double force_damping, int force_delay, double force_initial)
{
    // Initialize steering actuator parameters
    mu_steer  = steer_gain;
    wn_steer  = steer_frequency;
    csi_steer = steer_damping;
    tau_steer = steer_delay;      // Multiples of dt

    // Initialize velocity actuator parameters
    mu_force  = force_gain;
    wn_force  = force_frequency;
    csi_force = force_damping;
    tau_force = force_delay;      // Multiples of dt

    actuatorParams_set = true;

    // Initialize the FIFO to represent the delay
    for (auto k=0; k<tau_steer-1; k++) {
        delta_ref_FIFO.push(steer_initial);
    }
    for (auto k=0; k<tau_force-1; k++) {
        Fxr_ref_FIFO.push(force_initial);
    }

    // Initialize actuator states
    delta = state[6] = steer_initial;
    Fxr   = state[8] = force_initial;
}

void singletrack_vxvy_force_ode::setReferenceCommands(double Fxr, double steer)
{
    switch (actuator_model)
    {
        case IDEAL:
            Fxr_ref   = Fxr;
            delta_ref = steer;
            break;

        case REAL:
            Fxr_ref_FIFO.push(Fxr);
            delta_ref_FIFO.push(steer);
            break;

        default:
            throw std::invalid_argument( "Unknown steering actuator model!" );
            break;
    }
}

void singletrack_vxvy_force_ode::integrate()
{
    // Check vehicle parameters are set
    if (!vehicleParams_set) {
        throw std::invalid_argument( "Vehicle parameters not set!" );
    }

    // Check steering actuator parameters are set
    if ((!actuatorParams_set) && (actuator_model!=singletrack_vxvy_force_ode::IDEAL)) {
        throw std::invalid_argument( "Actuator parameters not set!" );
    }

    // Integrate for one step ahead
    using namespace std::placeholders;
    stepper.do_step(std::bind(&singletrack_vxvy_force_ode::vehicle_ode, this, _1, _2, _3), state, t, dt);

    // Update time and steering
    t += dt;

    if (actuator_model==singletrack_vxvy_force_ode::REAL) {
        Fxr_ref_FIFO.pop();
        delta_ref_FIFO.pop();
    }
}

void singletrack_vxvy_force_ode::vehicle_ode(const state_type &state, state_type &dstate, double t)
{
    using namespace boost::math;

    // Actual state
    const double r   = state[0];
    const double Vx  = state[1];
    const double Vy  = state[2];
    const double x   = state[3];
    const double y   = state[4];
    const double psi = state[5];
    const double steer_pos = state[6];
    const double steer_vel = state[7];
    const double force_pos = state[8];
    const double force_vel = state[9];

    // Steering actuator model
    switch (actuator_model)
    {
        case IDEAL:
            Fxr   = mu_force*Fxr_ref;
            delta = mu_steer*delta_ref;

            // These states are not used
            dstate[6] = 0.0;
            dstate[7] = 0.0;
            dstate[8] = 0.0;
            dstate[9] = 0.0;
            break;

        case REAL:
            // Check the delta_ref FIFO is not empty
            if (delta_ref_FIFO.empty()) {
                throw std::length_error( "delta_ref FIFO is empty!");
            }

            // Check the Fxr_ref FIFO is not empty
            if (Fxr_ref_FIFO.empty()) {
                throw std::length_error( "Fxr_ref FIFO is empty!");
            }

            // Compute delta command
            Fxr   = force_pos;
            delta = steer_pos;

            // Update the actuator model state
            dstate[6] = steer_vel;
            dstate[7] = -std::pow(wn_steer,2.0)*steer_pos-2*csi_steer*wn_steer*steer_vel+mu_steer*std::pow(wn_steer,2.0)*delta_ref_FIFO.front();
            dstate[8] = force_vel;
            dstate[9] = -std::pow(wn_force,2.0)*force_pos-2*csi_force*wn_force*force_vel+mu_force*std::pow(wn_force,2.0)*Fxr_ref_FIFO.front();
            break;

        default:
            throw std::invalid_argument( "Unknown steering actuator model!" );
            break;
    }

    // Slip angles
    if (std::abs(Vx)<=vx_thd) {
        alphaf = 0.0;
        alphar = 0.0;
    }
    else {
        alphaf = std::atan2(Vy+a*r,Vx)-delta;
        alphar = std::atan2(Vy-b*r,Vx);
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

            throw std::invalid_argument( "Unknown tyre model!" );
            break;
    }

    // Vehicle equations
    double Fx = Fxr-Fyf*std::sin(delta);
    double Fy = Fyr+Fyf*std::cos(delta);
    double Mz = a*Fyf*std::cos(delta)-b*Fyr;

    dstate[0] = Mz/Iz;                             // dr
    dstate[1] = Fx/m+r*Vy;                         // dVx
    dstate[2] = Fy/m-r*Vx;                         // dVy
    dstate[3] = std::cos(psi)*Vx-std::sin(psi)*Vy; // dx
    dstate[4] = std::sin(psi)*Vx+std::cos(psi)*Vy; // dy
    dstate[5] = r;                                 // r

    // Other variables
    ay = dstate[2]+r*Vx;
    if (std::abs(Vx)<=vx_thd) {
        sideslip = 0.0;
    }
    else {
        sideslip = std::atan2(Vy,Vx);
    }
}
