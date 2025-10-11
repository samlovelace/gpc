
#include "gpc/LQRController.h"
#include <iostream>

LQRController::LQRController()
{
    mIsModelBased = true; 
}

LQRController::~LQRController()
{

}

bool LQRController::init(std::shared_ptr<IDynamicSystem> aDynamics)
{
    return true; 
}

Eigen::VectorXd LQRController::compute(const Eigen::VectorXd& aGoal, 
                                       const Eigen::VectorXd& aState, 
                                       const double& aDeltaTime_s)
{
    std::cout << "[LQR] compute" << std::endl; 
    // Eigen::VectorXd U = - mGainMatrix * (aState - aGoal);
    
    // // Saturation
    // for (int i = 0; i < M; ++i) {
    //     if (U(i) > u_max_(i)) U(i) = u_max_(i);
    //     if (U(i) < u_min_(i)) U(i) = u_min_(i);
    // }
    return Eigen::VectorXd(3); 
}