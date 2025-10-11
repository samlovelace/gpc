
#include "gpc/PIDController.h"
#include <iostream>

PIDController::PIDController(std::map<std::string, std::vector<double>> aGainsMap) : mGainsMap(aGainsMap), mDof(-1)
{
    mDof = mGainsMap["Kp"].size(); 

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

    mPrevState.resize(mDof); 
    mErrorIntegral.resize(mDof); 

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
    }

    Eigen::VectorXd error = aGoal - aState;    
    std::cout << "Error: " << error << std::endl; 

    Eigen::VectorXd command; 
    command.resize(mDof); 

    for(int i = 0; i < mDof; i++)
    {
        // compute error deriv and error integral values for this axis 
        double errorDeriv = (aState[i] - mPrevState[i]) / aDeltaTime_s; 
        mErrorIntegral[i] += error[i] * aDeltaTime_s; 

        // compute control inputs 
        double P = mGainsMap["Kp"][i] * error[i];
        double I = mGainsMap["Ki"][i] * mErrorIntegral[i]; 
        double D = mGainsMap["Kd"][i] * errorDeriv;  
        
        command(i) = P + I + D; 
    }

    std::cout << "PID Control Input: " << command << std::endl; 

    mPrevState = aState;
    return command; 
}