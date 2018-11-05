#ifndef FBLIN_LOPEZ_II
#define FBLIN_LOPEZ_II

#include "bicycle_dyn_fblin.h"


class fblin_lopez_II: public bicycle_dyn_fblin
{
    public:
        fblin_lopez_II(double P_distance, double sampling_time);
        ~fblin_lopez_II();

        void control_transformation(double vPx, double vPy, double& speed, double& steer);
        void ouput_transformation(double& xP, double& yP);
        void reference_transformation(double xref, double yref, double& xPref, double& yPref);
    private:
        double _p, _Ts, _delta;
};

#endif /* FBLIN_LOPEZ_II */