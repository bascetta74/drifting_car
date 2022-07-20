#include "acceleration_sideslip_estimator.h"
#include "singletrack_beta_velocity_ode.h"

#include <iostream>
#include <fstream>
#include <string.h>


int main(int argc, char** argv) {
    // Create simulator class
    const double dT = 0.001;
    const double Tend = 30.0;

    singletrack_beta_velocity_ode sim = singletrack_beta_velocity_ode(dT, singletrack_beta_velocity_ode::FIALA_WITH_SATURATION, singletrack_beta_velocity_ode::IDEAL, 0.01);
    std::cout << "Running singletrack_beta_velocity_ode, Fiala with saturation tyre, ideal actuation" << std::endl;;

    // Set initial state and vehicle parameters
    const double r0    = 0.0;
    const double beta0 = 0.0;
    const double x0    = 0.0;
    const double y0    = 0.0;
    const double psi0  = 0.0;
    sim.setInitialState(r0, beta0, x0, y0, psi0);

    // Set steering actuator parameters
    const double mu_steer      = 1.0;
    const double wn_steer      = 87.62;
    const double csi_steer     = 0.75;
    const int tau_steer        = 55;
    const double initial_steer = 0.0;
    const double mu_force      = 1.0;
    const double wn_force      = 650.0;
    const double csi_force     = 0.9;
    const int tau_force        = 0;
    const double initial_force = 1.5;
    sim.setActuatorParams(mu_steer, wn_steer, csi_steer, tau_steer, initial_steer,
                           mu_force, wn_force, csi_force, tau_force, initial_force);

    // Set vehicle parameters
    const double a  = 0.1368;
    const double b  = 0.1232;
    const double m  = 1.9;
    const double mu = 0.385;
    const double Cf = 50.13;
    const double Cr = 122.05;
    const double Iz = 0.029;
    sim.setVehicleParams(m, a, b, Cf, Cr, mu, Iz);

    // Create estimator class
    const double Kpa   = 1000.0;
    const double Kda   = 100.0;
    const double Ta    = 0.001;
    const double v_thd = 0.1;
    const double Ts    = 0.001;

    acceleration_sideslip_estimator est = acceleration_sideslip_estimator(Kpa, Kda, Ta, v_thd, Ts);
    std::cout << "Running acceleration_sideslip_estimator" << std::endl;;

    // Open file to store simulation results
    std::ofstream result_file;
    std::string result_file_name;

    result_file_name = "test_beta_estimator_acceleration.txt";
    result_file.open (result_file_name);

    // Simulate vehicle motion
    double time = 0.0;
    for (auto k=0; k<Tend/dT; k++) {
        // Vehicle commands
        double velocity_ref = std::fmin(2.0,1.0+0.05*time);
        double steer_ref    = 0.0+0.5*std::sin(1.0*time);
        sim.setReferenceCommands(velocity_ref, steer_ref);

        // Store first sample
        if (k==0) {
            // Get variables from simulator
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

            // Integrate and get variables from estimator
            est.setVehiclePose(x, y, psi);

            double beta_est;
            est.execute();
            est.getSideslip(beta_est);

            double axFbl, ayFbl;
            est.getFbLControl(axFbl, ayFbl);

            double xun, yun, gammaun;
            est.getUnicycleState(xun, yun, gammaun);

            // Store data to file
            result_file << std::setprecision(16) <<
                        t << ";" << velocity_ref << ";" << steer_ref << ";" << velocity_cmd << ";" << steer_cmd << ";"
                        << x << ";" << y << ";" << psi << ";" << ay << ";" << r << ";" << vy << ";" << beta << ";"
                        << alphaf << ";" << alphar << ";" << Fyf << ";" << Fyr << ";"
                        << beta_est << ";" << axFbl << ";" << ayFbl << ";" << xun << ";" << yun << ";" << gammaun << std::endl;
        }

        // Integrate model
        sim.integrate();

        // Get variables from simulator
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

        // Integrate and get variables form estimator
        auto dv = std::div(k, 10);
        if (std::abs(dv.rem) < dT) {
            est.setVehiclePose(x, y, psi);
        }

        double beta_est;
        est.execute();
        est.getSideslip(beta_est);

        double axFbl, ayFbl;
        est.getFbLControl(axFbl, ayFbl);

        double xun, yun, gammaun;
        est.getUnicycleState(xun, yun, gammaun);

        // Store data to file
        result_file << std::setprecision(16) <<
                    t << ";" << velocity_ref << ";" << steer_ref << ";" << velocity_cmd << ";" << steer_cmd << ";"
                    << x << ";" << y << ";" << psi << ";" << ay << ";" << r << ";" << vy << ";" << beta << ";"
                    << alphaf << ";" << alphar << ";" << Fyf << ";" << Fyr << ";"
                    << beta_est << ";" << axFbl << ";" << ayFbl << ";" << xun << ";" << yun << ";" << gammaun << std::endl;

        // Update time
        time += dT;
    }

    // CLose results file
    result_file.close();

    return 0;
}
