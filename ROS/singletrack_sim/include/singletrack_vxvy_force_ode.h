#include <queue>
#include <boost/numeric/odeint.hpp>

using namespace boost::numeric::odeint;

typedef std::vector<double> state_type;

// Simulation of a single-track dynamic model, rear-wheel drive, with different tyre models and actuator models
// Control inputs: rear longitudinal force, steer

class singletrack_vxvy_force_ode
{
public:
    typedef enum e_tyreModel {
      LINEAR = 0,
      FIALA_WITH_SATURATION = 1,
      FIALA_WITHOUT_SATURATION = 2
    } tyreModel;

    typedef enum e_actuatorModel {
      IDEAL = 0,
      REAL = 1
    } actuatorModel;

    singletrack_vxvy_force_ode(double deltaT, tyreModel tyre_model, actuatorModel actuator_model, double vx_thd);

    void setInitialState(double r0, double Vx0, double Vy0, double x0, double y0, double psi0);
    void setVehicleParams(double m, double a, double b, double Cf, double Cr, double mu, double Iz);
    void setActuatorParams(double steer_gain, double steer_frequency, double steer_damping, int steer_delay, double steer_initial,
                           double force_gain, double force_frequency, double force_damping, int force_delay, double force_initial);
    void integrate();
    
    void setReferenceCommands(double Fxr, double steer);
    
    void getPose(double &x, double &y, double &psi) { x = state[3]; y = state[4]; psi = state[5]; };
    void getLongitudinalDynamics(double &vx) { vx = state[1]; };
    void getLateralDynamics(double &ay, double &yawrate, double &vy) { ay = this->ay; yawrate = state[0]; vy = state[2]; };
    void getSideslip(double &sideslip) { sideslip = this->sideslip; }
    void getSlip(double &slip_front, double &slip_rear) { slip_front = alphaf; slip_rear = alphar; }
    void getLateralForce(double &force_front, double &force_rear) { force_front = Fyf; force_rear = Fyr; }
    void getCommands(double &Fxr, double &steer) { Fxr = this->Fxr; steer = delta; };
    void getTime(double &time) { time = t; };

private:
    // Simulator and integrator variables
    double t, dt;
    double vx_thd;
    double Fxr_ref, Fxr, delta_ref, delta;
    std::queue<double> delta_ref_FIFO, Fxr_ref_FIFO;
    double sideslip, ay, alphaf, alphar, Fyf, Fyr;
    double m, a, b, Cf, Cr, mu, Iz;
    double mu_steer, wn_steer, csi_steer;
    double mu_force, wn_force, csi_force;
    int tau_steer, tau_force;

    bool vehicleParams_set, actuatorParams_set;
    tyreModel tyre_model;
    actuatorModel actuator_model;

    state_type state;
    runge_kutta_dopri5 < state_type > stepper;

    // ODE function
    void vehicle_ode(const state_type &state, state_type &dstate, double t);
};
