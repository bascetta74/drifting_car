#include "singletrack_beta_velocity_ode.h"

#include <iostream>
#include <fstream>
#include <string.h>


int main(int argc, char** argv) {
    // Check input arguments
    if (argc<2) {
        std::cout << std::endl << "Usage:" << std::endl;
        std::cout << "test_singletrack_beta_velocity_ode tire_type actuation_type" << std::endl;
        std::cout << "tire_type = {linear, fiala_w_sat, fiala_wo_sat}" << std::endl;
        std::cout << "actuation_type = {ideal, real}" << std::endl << std::endl;

        exit(1);
    }

    // Create simulator class
    const double dT = 0.001;
    const double Tend = 60;

    singletrack_beta_velocity_ode* sim;
    if (!strcmp(argv[1], "linear") && !strcmp(argv[2], "ideal")) {
        sim = new singletrack_beta_velocity_ode(dT, singletrack_beta_velocity_ode::LINEAR, singletrack_beta_velocity_ode::IDEAL, 0.01);
        std::cout << "Running singletrack_beta_velocity_ode, linear tyre, ideal actuation" << std::endl;
    }
    else if (!strcmp(argv[1], "linear") && !strcmp(argv[2], "real")) {
        sim = new singletrack_beta_velocity_ode(dT, singletrack_beta_velocity_ode::LINEAR, singletrack_beta_velocity_ode::REAL, 0.01);
        std::cout << "Running singletrack_beta_velocity_ode, linear tyre, real actuation" << std::endl;
    }
    else if (!strcmp(argv[1], "fiala_w_sat") && !strcmp(argv[2], "ideal")) {
        sim = new singletrack_beta_velocity_ode(dT, singletrack_beta_velocity_ode::FIALA_WITH_SATURATION, singletrack_beta_velocity_ode::IDEAL, 0.01);
        std::cout << "Running singletrack_beta_velocity_ode, Fiala with saturation tyre, ideal actuation" << std::endl;
    }
    else if (!strcmp(argv[1], "fiala_w_sat") && !strcmp(argv[2], "real")) {
        sim = new singletrack_beta_velocity_ode(dT, singletrack_beta_velocity_ode::FIALA_WITH_SATURATION, singletrack_beta_velocity_ode::REAL, 0.01);
        std::cout << "Running singletrack_beta_velocity_ode, Fiala with saturation tyre, real actuation" << std::endl;
    }
    else if (!strcmp(argv[1], "fiala_wo_sat") && !strcmp(argv[2], "ideal")) {
        sim = new singletrack_beta_velocity_ode(dT, singletrack_beta_velocity_ode::FIALA_WITHOUT_SATURATION, singletrack_beta_velocity_ode::IDEAL, 0.01);
        std::cout << "Running singletrack_beta_velocity_ode, Fiala without saturation tyre, ideal actuation" << std::endl;
    }
    else if (!strcmp(argv[1], "fiala_wo_sat") && !strcmp(argv[2], "real")) {
        sim = new singletrack_beta_velocity_ode(dT, singletrack_beta_velocity_ode::FIALA_WITHOUT_SATURATION, singletrack_beta_velocity_ode::REAL, 0.01);
        std::cout << "Running singletrack_beta_velocity_ode, Fiala without saturation tyre, real actuation" << std::endl;
    }
    else {
        std::cout << "Wrong arguments specified, nothing to do!" << std::endl;
        exit(1);
    }

    // Set initial state and vehicle parameters
    const double r0    = 0.0;
    const double beta0 = 0.0;
    const double x0    = 0.0;
    const double y0    = 0.0;
    const double psi0  = 0.0;
    sim->setInitialState(r0, beta0, x0, y0, psi0);

    // Set actuator parameters
    const double mu_steer      = 1.0;
    const double wn_steer      = 87.62;
    const double csi_steer     = 0.75;
    const int tau_steer        = 55;
    const double initial_steer = 0.0;
    const double mu_speed      = 1.0;
    const double wn_speed      = 650.0;
    const double csi_speed     = 0.9;
    const int tau_speed        = 36;
    const double initial_speed = 1.5;
    sim->setActuatorParams(mu_steer, wn_steer, csi_steer, tau_steer, initial_steer,
                           mu_speed, wn_speed, csi_speed, tau_speed, initial_speed);

    // Set vehicle parameters
    const double a  = 0.1368;
    const double b  = 0.1232;
    const double m  = 1.9;
    const double mu = 0.385;
    const double Cf = 50.13;
    const double Cr = 122.05;
    const double Iz = 0.029;
    sim->setVehicleParams(m, a, b, Cf, Cr, mu, Iz);

    // Open file to store simulation results
    std::ofstream result_file;
    std::string result_file_name;

    result_file_name = "test_singletrack_beta_velocity_" + std::string(argv[1]) + "_" + std::string(argv[2]) + ".txt";
    result_file.open (result_file_name);

    // Simulate vehicle motion
    double time = 0.0;
    for (auto k=0; k<Tend/dT; k++) {
        // Vehicle commands
        double velocity_ref = initial_speed+0.5*time;
        double steer_ref    = initial_steer+std::sin(0.1*time);
        sim->setReferenceCommands(velocity_ref, steer_ref);

        // Store first sample
        if (k==0) {
            // Get variables
            double t;
            sim->getTime(t);

            double x, y, psi;
            sim->getPose(x, y, psi);

            double ay, r, vy, beta;
            sim->getLateralDynamics(ay, r, vy);
            sim->getSideslip(beta);

            double alphaf, alphar, Fyf, Fyr;
            sim->getSlip(alphaf, alphar);
            sim->getLateralForce(Fyf, Fyr);

            double velocity_cmd, steer_cmd;
            if (!strcmp(argv[2], "ideal")) {
                velocity_cmd = velocity_ref;
                steer_cmd = steer_ref;
            } else {
                sim->getCommands(velocity_cmd, steer_cmd);
            }

            // Store data to file
            result_file << std::setprecision(16) <<
                        t << ";" << velocity_ref << ";" << steer_ref << ";" << velocity_cmd << ";" << steer_cmd << ";"
                        << x << ";" << y << ";" << psi << ";" << ay << ";" << r << ";" << vy << ";" << beta << ";"
                        << alphaf << ";" << alphar << ";" << Fyf << ";" << Fyr << std::endl;
        }

        // Integrate model
        sim->integrate();

        // Get variables
        double t;
        sim->getTime(t);

        double x, y, psi;
        sim->getPose(x, y, psi);

        double ay, r, vy, beta;
        sim->getLateralDynamics(ay, r, vy);
        sim->getSideslip(beta);

        double alphaf, alphar, Fyf, Fyr;
        sim->getSlip(alphaf, alphar);
        sim->getLateralForce(Fyf, Fyr);

        double velocity_cmd, steer_cmd;
        sim->getCommands(velocity_cmd, steer_cmd);

        // Store data to file
        result_file << std::setprecision(16) <<
                    t << ";" << velocity_ref << ";" << steer_ref << ";" << velocity_cmd << ";" << steer_cmd << ";"
                    << x << ";" << y << ";" << psi << ";" << ay << ";" << r << ";" << vy << ";" << beta << ";"
                    << alphaf << ";" << alphar << ";" << Fyf << ";" << Fyr << std::endl;

        // Update time
        time += dT;
    }

    // Delete simulator object
    delete sim;

    // CLose results file
    result_file.close();

    return 0;
}
