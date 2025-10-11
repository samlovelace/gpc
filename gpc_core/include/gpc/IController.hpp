#ifndef ICONTROLLER_HPP
#define ICONTROLLER_HPP
 
#include <eigen3/Eigen/Dense>
#include <memory>
#include "IDynamicSystem.hpp"


class IController 
{ 
public:
    IController() : mIsModelBased(false), mDynamicModel(nullptr) {}
    virtual ~IController() = default; 

    virtual bool init(std::shared_ptr<IDynamicSystem> aDynamics) = 0; 
    
    virtual Eigen::VectorXd compute(const Eigen::VectorXd& aGoal, 
                                    const Eigen::VectorXd& aState, 
                                    const double& aDeltaTime_s) = 0; 

    bool isModelBased() {return mIsModelBased;}

protected:
   bool mIsModelBased;
   std::shared_ptr<IDynamicSystem> mDynamicModel; 
};
#endif //ICONTROLLER_HPP