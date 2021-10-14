#ifndef FBLIN_SCHLACHER
#define FBLIN_SCHLACHER


class fblin_schlacher
{
    public:
        fblin_schlacher();
        ~fblin_schlacher();

        void set_bicycleParam(double m, double Iz, double lf, double lr, double Cf, double Cr) { _m=m; _Iz=Iz; _lf=lf; _lr=lr; _Cf=Cf; _Cr=Cr; };
        void set_bicycleState(double abs_velocity, double sideslip, double yaw_rate) { _V=abs_velocity; _beta=sideslip; _r=yaw_rate; };

        void control_transformation(double w1, double w2, double& Fxr, double& steer);
        void ouput_transformation(double& z1, double& z2);
    private:
        double _m, _Iz, _lf, _lr, _Cf, _Cr;
        double _V, _beta, _r;
};

#endif /* FBLIN_SCHLACHER */