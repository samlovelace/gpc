#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H
 
#include "gpc/IController.hpp"
#include <string> 
#include <vector> 
#include <map> 

class PIDController : public IController
{ 
public:
    PIDController(std::map<std::string, std::vector<double>> aGainsMap);
    ~PIDController() override; 

    bool init(std::shared_ptr<IDynamicSystem> aDynamics) override; 

    Eigen::VectorXd compute(const Eigen::VectorXd& aGoal, 
                            const Eigen::VectorXd& aState, 
                            const double& aDeltaTime_s) override; 

private:
    std::map<std::string, std::vector<double>> mGainsMap; 
    int mDof;
    
    Eigen::VectorXd mPrevState; 
    Eigen::VectorXd mErrorIntegral; 

};
#endif //PIDCONTROLLER_H