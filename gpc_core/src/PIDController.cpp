
#include "gpc/PIDController.h"
#include <iostream>
#include "gpc/EigenPrinter.hpp"

PIDController::PIDController(std::map<std::string, std::vector<double>> aGainsMap, std::vector<int> aVecOfIndices) : 
    mGainsMap(aGainsMap), mDof(-1)
{
    mIndices = aVecOfIndices;
    mDof = mIndices.size();  

    for(const auto& [axis, gains] : mGainsMap)
    {
        if(gains.size() != mDof)
        {
            throw std::runtime_error("Invalid PID controller gains. Make sure all gains vectors are the same size"); 
        }

        std::cout << axis << ": "; 
        for(const auto& gain : gains)
        {
            std::cout << gain << ", "; 
        }
        std::cout << std::endl; 
    }

    mErrorIntegral.resize(mDof); 
    mPrevError.resize(mDof); 
}

PIDController::~PIDController()
{
    
}

bool PIDController::init(std::shared_ptr<IDynamicSystem> aDynamics)
{
    // TODO: something here? 
    return true; 
}

Eigen::VectorXd PIDController::compute(const Eigen::VectorXd& aGoal, const Eigen::VectorXd& aState, const double& aDeltaTime_s)
{
    if((aGoal.size() != aState.size()) && (aGoal.size() != mDof || aState.size() != mDof))
    {
        // handle somehow 
        return Eigen::VectorXd(mDof); 
    }

    Eigen::VectorXd error = aGoal - aState;
    EigenPrinter single(EigenPrinter::Style::SingleLine, 4, "Error: ");   
    single.print(error);  

    Eigen::VectorXd command; 
    command.resize(mDof); 

    for(int i = 0; i < mDof; i++)
    {
        // TODO: update to use the configure indicies
        int stateIndex = i; 

        // compute error deriv and error integral values for this axis 
        double errorDeriv = (error[stateIndex] - mPrevError[stateIndex]) / aDeltaTime_s; 
        mErrorIntegral[i] += (error[stateIndex] * aDeltaTime_s); 

        // compute control inputs 
        double P = mGainsMap["Kp"][i] * error[stateIndex];
        double I = mGainsMap["Ki"][i] * mErrorIntegral[stateIndex]; 
        double D = mGainsMap["Kd"][i] * errorDeriv;  
        
        std::cout << "P: " << P << " I: " << I << " D: " << D << "\n"; 

        command(i) = P + I + D; 
    }

    EigenPrinter single2(EigenPrinter::Style::SingleLine, 4, "PID Control Input: ");   
    single2.print(command);   

    mPrevError = error; 
    return command; 
}