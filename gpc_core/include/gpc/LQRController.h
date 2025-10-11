#ifndef LQRCONTROLLER_H
#define LQRCONTROLLER_H
 
#include "IController.hpp" 

class LQRController : public IController
{ 
public:
    LQRController();
    ~LQRController() override; 

    bool init(std::shared_ptr<IDynamicSystem> aDynamics) override; 

    Eigen::VectorXd compute(const Eigen::VectorXd& aGoal, 
                            const Eigen::VectorXd& aState, 
                            const double& aDeltaTime_s) override;

private:
   
};
#endif //LQRCONTROLLER_H