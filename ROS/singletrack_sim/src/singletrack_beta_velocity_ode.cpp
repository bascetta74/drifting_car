#include "singletrack_beta_velocity_ode.h"

#include <boost/math/special_functions/sign.hpp>

singletrack_beta_velocity_ode::singletrack_beta_velocity_ode(double deltaT, tyreModel tyre_model, actuatorModel actuator_model, double vx_thd) : dt(deltaT),
    t(0.0), state(9), Vx_ref(0.0), Vx(0.0), delta_ref(0.0), delta(0.0), alphaf(0.0), alphar(0.0), Fyf(0.0), Fyr(0.0),
    vehicleParams_set(false), actuatorParams_set(false)
{
    // state = [ r, beta, x, y, psi, steer_pos, steer_vel, speed_pos, speed_vel ]

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

    // Tyre and actuator models
    this->tyre_model = tyre_model;
    this->actuator_model = actuator_model;

    // Vx threshold for slip/sideslip computation
    this->vx_thd = vx_thd;
}

void singletrack_beta_velocity_ode::setInitialState(double r0, double beta0, double x0, double y0, double psi0)
{
    // Initial state values
    state[0] = r0;
    state[1] = beta0;
    state[2] = x0;
    state[3] = y0;
    state[4] = psi0;
    state[5] = 0.0;
    state[6] = 0.0;
    state[7] = 0.0;
    state[8] = 0.0;
}

void singletrack_beta_velocity_ode::setVehicleParams(double m, double a, double b, double Cf, double Cr, double mu, double Iz)
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

void singletrack_beta_velocity_ode::setActuatorParams(double steer_gain, double steer_frequency, double steer_damping, int steer_delay,
                                                      double speed_gain, double speed_frequency, double speed_damping, int speed_delay)
{
    // Initialize steering actuator parameters
    mu_steer  = steer_gain;
    wn_steer  = steer_frequency;
    csi_steer = steer_damping;
    tau_steer = steer_delay;      // Multiples of dt

    // Initialize velocity actuator parameters
    mu_speed  = speed_gain;
    wn_speed  = speed_frequency;
    csi_speed = speed_damping;
    tau_speed = speed_delay;      // Multiples of dt

    actuatorParams_set = true;

    // Initialize the FIFO to represent the delay
    for (auto k=0; k<tau_steer-1; k++) {
        delta_ref_FIFO.push(0.0);
    }
    for (auto k=0; k<tau_speed-1; k++) {
        Vx_ref_FIFO.push(0.0);
    }
}

void singletrack_beta_velocity_ode::setReferenceCommands(double velocity, double steer)
{
    switch (actuator_model)
    {
        case IDEAL:
            Vx_ref    = velocity;
            delta_ref = steer;
            break;

        case REAL:
            Vx_ref_FIFO.push(velocity);
            delta_ref_FIFO.push(steer);
            break;

        default:
            throw std::invalid_argument( "Unknown steering actuator model!" );
            break;
    }
}

void singletrack_beta_velocity_ode::integrate()
{
    // Check vehicle parameters are set
    if (!vehicleParams_set) {
        throw std::invalid_argument( "Vehicle parameters not set!" );
    }

    // Check steering actuator parameters are set
    if ((!actuatorParams_set) && (actuator_model!=singletrack_beta_velocity_ode::IDEAL)) {
        throw std::invalid_argument( "Actuator parameters not set!" );
    }

    // Integrate for one step ahead
    using namespace std::placeholders;
    stepper.do_step(std::bind(&singletrack_beta_velocity_ode::vehicle_ode, this, _1, _2, _3), state, t, dt);

    // Update time and actuators
    t += dt;

    if (actuator_model==singletrack_beta_velocity_ode::REAL) {
        Vx_ref_FIFO.pop();
        delta_ref_FIFO.pop();
    }
}

void singletrack_beta_velocity_ode::vehicle_ode(const state_type &state, state_type &dstate, double t)
{
    using namespace boost::math;

    // Actual state
    const double r    = state[0];
    const double beta = state[1];
    const double x    = state[2];
    const double y    = state[3];
    const double psi  = state[4];
    const double steer_pos = state[5];
    const double steer_vel = state[6];
    const double speed_pos = state[7];
    const double speed_vel = state[8];

    // Steering actuator model
    switch (actuator_model)
    {
        case IDEAL:
            Vx    = Vx_ref;
            delta = delta_ref;

            // These states are not used
            dstate[5] = 0.0;
            dstate[6] = 0.0;
            dstate[7] = 0.0;
            dstate[8] = 0.0;
            break;

        case REAL:
            // Check the delta_ref FIFO is not empty
            if (delta_ref_FIFO.empty()) {
                throw std::length_error( "delta_ref FIFO is empty!");
            }

            // Check the Vx_ref FIFO is not empty
            if (Vx_ref_FIFO.empty()) {
                throw std::length_error( "Vx_ref FIFO is empty!");
            }

            // Compute delta command
            Vx    = mu_speed*std::pow(wn_speed,2.0)*speed_pos;
            delta = mu_steer*std::pow(wn_steer,2.0)*steer_pos;

            // Update the actuator model state
            dstate[5] = steer_vel;
            dstate[6] = -std::pow(wn_steer,2.0)*steer_pos-2*csi_steer*wn_steer*steer_vel+delta_ref_FIFO.front();
            dstate[7] = speed_vel;
            dstate[8] = -std::pow(wn_speed,2.0)*speed_pos-2*csi_speed*wn_speed*speed_vel+Vx_ref_FIFO.front();
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
        alphaf = std::atan2(beta+a*r,Vx)-delta;
        alphar = std::atan2(beta-b*r,Vx);
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
    dstate[0] = (a*Fyf-b*Fyr)/Iz;                                                                            // dr
    dstate[1] = (Fyf*(std::cos(beta)+std::sin(beta)*delta)+Fyr*std::cos(beta))/(m*Vx)*std::cos(beta)-r;      // dbeta
    dstate[2] = Vx/std::cos(beta)*std::cos(psi+beta);                                                        // dx
    dstate[3] = Vx/std::cos(beta)*std::sin(psi+beta);                                                        // dy
    dstate[4] = r;                                                                                           // dpsi

    // Other variables
    ay = Vx*dstate[1]+r*Vx;
}
