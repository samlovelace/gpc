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

    Eigen::VectorXd compute(const Eigen::VectorXd& aGoal, const Eigen::VectorXd& aState) override; 

private:
    std::map<std::string, std::vector<double>> mGainsMap; 

};
#endif //PIDCONTROLLER_H