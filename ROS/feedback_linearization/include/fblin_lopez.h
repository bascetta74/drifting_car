#ifndef FBLIN_LOPEZ
#define FBLIN_LOPEZ


class fblin_lopez
{
    public:
        fblin_lopez(double P_distance, double sampling_time);
        ~fblin_lopez();

        void set_bicycleParam(double cog_front, double cog_rear) { _lf=cog_front; _lr=cog_rear; };
        void set_bicycleState(double position_x, double position_y, double heading, double sideslip, double yaw_rate) { _x=position_x; _y=position_y; _psi=heading; _beta=sideslip; _r=yaw_rate; };
        void set_bicycleSteer(double steer) { _steer=steer; };

        void control_transformation(double vPx, double vPy, double& speed, double& steer);
        void ouput_transformation(double& xP, double& yP);
        void reference_transformation(double xref, double yref, double& xPref, double& yPref);
    private:
        double _p, _lf, _lr, _Ts, _delta;
        double _x, _y, _psi, _beta, _r, _steer;
};

#endif /* FBLIN_LOPEZ */