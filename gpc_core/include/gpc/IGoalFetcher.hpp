#ifndef IGOALFETCHER_HPP
#define IGOALFETCHER_HPP
 
#include <eigen3/Eigen/Dense> 

class IGoalFetcher 
{ 
public:
    virtual ~IGoalFetcher() = default; 
    virtual Eigen::VectorXd fetchGoal() = 0;
    virtual void setGoal(Eigen::VectorXd aGoal) {mGoal = aGoal; }  

protected:
    Eigen::VectorXd mGoal; 
   
};
#endif //IGOALFETCHER_HPP