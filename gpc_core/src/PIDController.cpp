
#include "gpc/PIDController.h"
#include <iostream>

PIDController::PIDController(std::map<std::string, std::vector<double>> aGainsMap) : mGainsMap(aGainsMap), mDof(-1)
{
    mDof = mGainsMap["Kp"].size(); 

    for(const auto& [axis, gains] : mGainsMap)
    {
        if(gains.size() != mDof)
        {
            throw std::runtime_error("INvalid PID controller gains. Make sure all gains are same size"); 
        }

        std::cout << axis << ": "; 
        for(const auto& gain : gains)
        {
            std::cout << gain << ", "; 
        }
        std::cout << std::endl; 
    }

}

PIDController::~PIDController()
{
    
}

Eigen::VectorXd PIDController::compute(const Eigen::VectorXd& aGoal, const Eigen::VectorXd& aState, const double& aDeltaTime_s)
{
    if((aGoal.size() != aState.size()) && (aGoal.size() != mDof || aState.size() != mDof))
    {
        // handle somehow 
    }

    Eigen::VectorXd error = aGoal - aState; 
    Eigen::VectorXd command(mDof); 

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

    mPrevState = aState;

    return command; 
}