#ifndef FBLIN_LOPEZ_I
#define FBLIN_LOPEZ_I

#include "bicycle_dyn_fblin.h"


class fblin_lopez_I: public bicycle_dyn_fblin
{
    public:
        fblin_lopez_I(double P_distance, double sampling_time, double zero_speed_threshold);
        ~fblin_lopez_I();

        void control_transformation(double vPx, double vPy, double& speed, double& steer);
        void ouput_transformation(double& xP, double& yP);
        void reference_transformation(double xref, double yref, double& xPref, double& yPref);
    private:
        double _p, _Ts, _zero_speed_thd, _v;
};

#endif /* FBLIN_LOPEZ_I */