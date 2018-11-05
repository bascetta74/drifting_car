#ifndef FBLIN_SPALIVIERO
#define FBLIN_SPALIVIERO

#include "bicycle_dyn_fblin.h"


class fblin_spaliviero: public bicycle_dyn_fblin
{
    public:
        fblin_spaliviero(double P_distance, double zero_speed_threshold);
        ~fblin_spaliviero();

        void control_transformation(double vPx, double vPy, double& speed, double& steer);
        void ouput_transformation(double& xP, double& yP);
        void reference_transformation(double xref, double yref, double& xPref, double& yPref);
    private:
        double _p, _zero_speed_thd;
};

#endif /* FBLIN_SPALIVIERO */