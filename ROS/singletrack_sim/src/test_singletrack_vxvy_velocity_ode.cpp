#include "singletrack_vxvy_velocity_ode.h"

#include <iostream>
#include <fstream>


int main() {
    // Create simulator class
    const double dT = 0.001;
    singletrack_vxvy_velocity_ode sim = singletrack_vxvy_velocity_ode(dT, singletrack_vxvy_velocity_ode::LINEAR, singletrack_vxvy_velocity_ode::IDEAL);
//    singletrack_vxvy_velocity_ode sim = singletrack_vxvy_velocity_ode(dT, singletrack_vxvy_velocity_ode::LINEAR, singletrack_vxvy_velocity_ode::REAL);
//    singletrack_vxvy_velocity_ode sim = singletrack_vxvy_velocity_ode(dT, singletrack_vxvy_velocity_ode::FIALA_WITH_SATURATION, singletrack_vxvy_velocity_ode::IDEAL);
//    singletrack_vxvy_velocity_ode sim = singletrack_vxvy_velocity_ode(dT, singletrack_vxvy_velocity_ode::FIALA_WITH_SATURATION, singletrack_vxvy_velocity_ode::REAL);
//    singletrack_vxvy_velocity_ode sim = singletrack_vxvy_velocity_ode(dT, singletrack_vxvy_velocity_ode::FIALA_WITHOUT_SATURATION, singletrack_vxvy_velocity_ode::IDEAL);
//    singletrack_vxvy_velocity_ode sim = singletrack_vxvy_velocity_ode(dT, singletrack_vxvy_velocity_ode::FIALA_WITHOUT_SATURATION, singletrack_vxvy_velocity_ode::REAL);

    // Set initial state and vehicle parameters
    const double r0   = 0.0;
    const double Vy0  = 0.0;
    const double x0   = 0.0;
    const double y0   = 0.0;
    const double psi0 = 0.0;
    sim.setInitialState(r0, Vy0, x0, y0, psi0);

    // Set steering actuator parameters
    const double mu_steer  = 1.0;
    const double wn_steer  = 87.62;
    const double csi_steer = 0.75;
    const int tau_steer    = 55;
    sim.setSteeringActuatorParams(mu_steer, wn_steer, csi_steer, tau_steer);

    // Set velocity actuator parameters
    const double mu_speed  = 1.0;
    sim.setVelocityActuatorParams(mu_speed);

    // Set vehicle parameters
    const double a  = 0.1513;
    const double b  = 0.1087;
    const double m  = 2.04;
    const double mu = 0.385;
    const double Cf = 25;
    const double Cr = 50;
    const double Iz = 0.030;
    sim.setVehicleParams(m, a, b, Cf, Cr, mu, Iz);

    // Open file to store simulation results
    std::ofstream result_file;
    result_file.open ("test_singletrack_vxvy_velocity_results.txt");

    // Simulate vehicle motion
    double time = 0.0;
    for (auto k=0; k<10e3; k++) {
        // Vehicle commands
        double velocity_ref = 1.5+0.5*time;
        double steer_ref    = std::sin(0.1*time);
        sim.setReferenceCommands(velocity_ref, steer_ref);

        // Integrate model
        sim.integrate();

        // Get variables
        double t;
        sim.getTime(t);

        double x, y, psi;
        sim.getPose(x, y, psi);

        double ay, r, vy, beta;
        sim.getLateralDynamics(ay, r, vy);
        sim.getSideslip(beta);

        double alphaf, alphar, Fyf, Fyr;
        sim.getSlip(alphaf, alphar);
        sim.getLateralForce(Fyf, Fyr);

        double velocity_cmd, steer_cmd;
        sim.getCommands(velocity_cmd, steer_cmd);

        // Store data to file
        result_file << std::setprecision(16) <<
            t << ";" << velocity_ref << ";" << steer_ref << ";" << velocity_cmd << ";" << steer_cmd << ";" << x << ";" << y << ";" << psi << ";" << ay << ";" << r << ";" << vy << ";" << beta << ";" << alphaf << ";" << alphar << ";" << Fyf << ";" << Fyr << std::endl;

        // Update time
        time += dT;
    }

    // CLose results file
    result_file.close();

    return 0;
}
