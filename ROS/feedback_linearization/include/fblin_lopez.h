#ifndef FBLIN_LOPEZ
#define FBLIN_LOPEZ

#include "bicycle_dyn_fblin.h"


class fblin_lopez: public bicycle_dyn_fblin
{
    public:
        fblin_lopez(double P_distance, double sampling_time);
        ~fblin_lopez();

        void control_transformation(double vPx, double vPy, double& speed, double& steer);
        void ouput_transformation(double& xP, double& yP);
        void reference_transformation(double xref, double yref, double& xPref, double& yPref);
    private:
        double _p, _Ts, _delta;
};

#endif /* FBLIN_LOPEZ */