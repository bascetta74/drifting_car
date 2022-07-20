// Implementation of the closed-loop velocity based sideslip estimator

#include <discrete_integrator.h>

class velocity_sideslip_estimator
{
public:
    velocity_sideslip_estimator(double P, double Kp, double Ts);
    velocity_sideslip_estimator(double P, double Kp, double Ts, double x0, double y0, double gamma0);
    ~velocity_sideslip_estimator();

    void setVehiclePose(double x, double y, double heading) { this->x_ref = x; this->y_ref = y; this->theta_ref = heading; };
    void execute();
    void execute(int nStep);
    void getSideslip(double &sideslip) { sideslip = gamma-theta_ref; };
    
    void getFbLControl(double &vPx, double &vPy) { vPx = this->vxP; vPy = this->vyP; };
    void getUnicycleState(double &x, double &y, double &gamma) { x = this->x; y = this->y; gamma = this->gamma; };

private:
    // Estimator and integrator variables
    double P, Kp;
    double vxP, vyP, x, y, gamma;
    double x_ref, y_ref, theta_ref;

    discrete_integrator_fwEul* dx;
    discrete_integrator_fwEul* dy;
    discrete_integrator_fwEul* dgamma;
};
