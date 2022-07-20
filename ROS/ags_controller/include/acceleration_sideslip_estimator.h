// Implementation of the closed-loop acceleration based sideslip estimator

#include <discrete_integrator.h>
#include <PID_controller.h>

class acceleration_sideslip_estimator
{
public:
    acceleration_sideslip_estimator(double Kp, double Kd, double T, double v_thd, double Ts);
    acceleration_sideslip_estimator(double Kp, double Kd, double T, double v_thd, double Ts,
                                    double x0, double y0, double gamma0, double v0);
    ~acceleration_sideslip_estimator();

    void setVehiclePose(double x, double y, double heading) { this->x_ref = x; this->y_ref = y; this->theta_ref = heading; };
    void execute();
    void execute(int nStep);
    void getSideslip(double &sideslip)  { sideslip = gamma-theta_ref; };
    
    void getFbLControl(double &ax, double &ay) { ax = this->ax; ay = this->ay; };
    void getUnicycleState(double &x, double &y, double &gamma) { x = this->x; y = this->y; gamma = this->gamma; };

private:
    // Estimator and integrator variables
    double v_thd;
    double ax, ay, x, y, gamma;
    double x_ref, y_ref, theta_ref;

    discrete_integrator_fwEul* dx;
    discrete_integrator_fwEul* dy;
    discrete_integrator_fwEul* dgamma;
    discrete_integrator_fwEul* dv;
    PID_controller* PD_ax;
    PID_controller* PD_ay;
};
