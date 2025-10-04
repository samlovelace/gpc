#ifndef ICONTROLLER_HPP
#define ICONTROLLER_HPP
 
#include <eigen3/Eigen/Dense>

class IController 
{ 
public:
    virtual ~IController() = default; 

    virtual Eigen::VectorXd compute(const Eigen::VectorXd& aGoal, 
                                    const Eigen::VectorXd& aState, 
                                    const double& aDeltaTime_s) = 0; 

private:
   
};
#endif //ICONTROLLER_HPP