#ifndef BICYCLE_DYN_FBLIN
#define BICYCLE_DYN_FBLIN


class bicycle_dyn_fblin
{
public:
    void set_bicycleParam(double mass, double yaw_inertia, double cornering_front, double cornering_rear, double cog_front, double cog_rear) { _m=mass; _Iz=yaw_inertia; _Cf=cornering_front; _Cr=cornering_rear; _lf=cog_front; _lr=cog_rear; };
    void set_bicycleState(double position_x, double position_y, double heading, double sideslip, double yaw_rate) { _x=position_x; _y=position_y; _psi=heading; _beta=sideslip; _r=yaw_rate; };
    void set_bicycleAbsoluteVelocity(double v) { _absVel=v; };
    void set_bicycleSteer(double steer) { _steer=steer; };

    virtual void control_transformation(double vPx, double vPy, double& speed, double& steer) =0;
    virtual void ouput_transformation(double& xP, double& yP) =0;
    virtual void reference_transformation(double xref, double yref, double& xPref, double& yPref) =0;

protected:
    double _x, _y;
    double _psi, _beta, _r, _absVel, _steer;
    double _m, _Iz, _Cr, _Cf, _lf, _lr;
};

#endif /* BICYCLE_DYN_FBLIN */