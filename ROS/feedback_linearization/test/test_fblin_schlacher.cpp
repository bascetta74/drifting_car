#include "MatlabDataArray.hpp"
#include "MatlabEngine.hpp"
#include <iostream>
#include <functional>
#include <algorithm>

#include "fblin_schlacher.h"

int main() {
    std::string matlabLine;
    using namespace matlab::engine;

    // Start MATLAB engine synchronously
    std::unique_ptr<MATLABEngine> matlabPtr = startMATLAB();

    // Initialize the MATLAB simulation
    matlabPtr->eval(u"clear all; close all");

    // Initialize controller
    fblin_schlacher fblin;

    double  m = 1.9;
    double Cf = 50.13;
    double Cr = 122.05;
    double Iz = 0.029;
    double lf = 0.1368;
    double lr = 0.1232;
    fblin.set_bicycleParam(m, Iz, lf, lr, Cf, Cr);

    // Start the simulation
    matlabLine = "simres = sim('fblin_schlacher_sim');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));

    // Execute the simulation
    std::vector<double> time, measurement, control, reference;

    // Get data from Matlab
    matlab::data::TypedArray<double> v   = matlabPtr->getVariable(u"v");
    matlab::data::ArrayDimensions v_size = v.getDimensions();
    matlab::data::TypedArray<double> beta  = matlabPtr->getVariable(u"beta");
    matlab::data::TypedArray<double> r     = matlabPtr->getVariable(u"r");
    matlab::data::TypedArray<double> w1    = matlabPtr->getVariable(u"w1");
    matlab::data::TypedArray<double> w2    = matlabPtr->getVariable(u"w2");

    std::cout << "Executing the simulation..." << std::endl;

    double Fxr, steer;
    std::vector<double> Fxr_vect, steer_vect;
    for (auto k=0; k<v_size.at(0); k++) {
        // Compute linearizing law
        fblin.set_bicycleState(v[k], beta[k], r[k]);
        fblin.control_transformation(w1[k], w2[k], Fxr, steer);

        // Store simulation data
        Fxr_vect.push_back(Fxr);
        steer_vect.push_back(steer);
    }

    std::cout << "Test completed, plotting results" << std::endl;

    matlab::data::ArrayFactory factory;
    matlab::data::TypedArray<double> m_Fxr = factory.createArray({Fxr_vect.size(),1}, Fxr_vect.begin(), Fxr_vect.end());
    matlabPtr->setVariable(u"Fx_cpp", std::move(m_Fxr));
    matlab::data::TypedArray<double> m_steer = factory.createArray({steer_vect.size(),1}, steer_vect.begin(), steer_vect.end());
    matlabPtr->setVariable(u"delta_cpp", std::move(m_steer));

    matlabPtr->eval(u"figure,plot(t,Fx, t,Fx_cpp,'r--'),grid,xlabel('Time [s]'),ylabel('Longitudinal force [N]')");
    matlabPtr->eval(u"figure,plot(t,delta, t,delta_cpp,'r--'),grid,xlabel('Time [s]'),ylabel('Steering angle [rad]')");

    // Terminate MATLAB session
    matlab::engine::terminateEngineClient();

	return 0;
}