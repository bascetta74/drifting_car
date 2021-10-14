#include <queue>
#include <boost/numeric/odeint.hpp>

using namespace boost::numeric::odeint;

typedef std::vector<double> state_type;

// Simulation of a single-track dynamic model, rear-wheel drive, with different tyre models and steer actuator model
// Control inputs: rear longitudinal force, steer

class singletrack_beta_force_ode
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

    singletrack_beta_force_ode(double deltaT, tyreModel tyre_model, actuatorModel actuator_model);

    void setInitialState(double r0, double beta0, double V0, double x0, double y0, double psi0);
    void setVehicleParams(double m, double a, double b, double Cf, double Cr, double mu, double Iz);
    void setSteeringActuatorParams(double gain, double frequency, double damping, int delay);

    void integrate();
    
    void setReferenceCommands(double Fxr, double steer);
    
    void getPose(double &x, double &y, double &psi) { x = state[3]; y = state[4]; psi = state[5]; };
    void getLongitudinalDynamics(double &vx) { vx = state[2]*std::cos(state[1]); };
    void getLateralDynamics(double &ay, double &yawrate, double &vy) { ay = this->ay; yawrate = state[0]; vy = state[2]*std::sin(state[1]); };
    void getAbsoluteVelocity(double &v) { v = state[2]; }
    void getSideslip(double &sideslip) { sideslip = state[1]; }
    void getSlip(double &slip_front, double &slip_rear) { slip_front = alphaf; slip_rear = alphar; }
    void getLateralForce(double &force_front, double &force_rear) { force_front = Fyf; force_rear = Fyr; }
    void getCommands(double &Fxr, double &steer) { Fxr = Fxr_ref; steer = delta; };
    void getTime(double &time) { time = t; };

private:
    // Simulator and integrator variables
    double t, dt;
    double delta_ref, delta, Fxr_ref;
    std::queue<double> delta_ref_FIFO;
    double ay, alphaf, alphar, Fyf, Fyr;
    double m, a, b, Cf, Cr, mu, Iz;
    double mu_steer, wn_steer, csi_steer;
    int tau_steer;

    bool vehicleParams_set, steeringActuatorParams_set;
    tyreModel tyre_model;
    actuatorModel steeringActuator_model;

    state_type state;
    runge_kutta_dopri5 < state_type > stepper;

    // ODE function
    void vehicle_ode(const state_type &state, state_type &dstate, double t);
};
