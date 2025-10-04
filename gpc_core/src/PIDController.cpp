
#include "gpc/PIDController.h"
#include <iostream>

PIDController::PIDController(std::map<std::string, std::vector<double>> aGainsMap) : mGainsMap(aGainsMap)
{
    for(const auto& [axis, gains] : mGainsMap)
    {
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

Eigen::VectorXd PIDController::compute(const Eigen::VectorXd& aGoal, const Eigen::VectorXd& aState)
{
    std::cout << "[PID] doing control..." << std::endl; 
    return Eigen::VectorXd(3); 
}