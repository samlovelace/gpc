#ifndef ISAFETYFILTER_HPP
#define ISAFETYFILTER_HPP
 
#include <memory>
#include <eigen3/Eigen/Dense>
#include "gpc/IConstraint.hpp"
 
class ISafetyFilter 
{ 
public:
    virtual ~ISafetyFilter() = default; 

    virtual void configure(int zDim) = 0;
    virtual void setConstraints(const std::vector<std::shared_ptr<IConstraint>>& constraints) = 0;
    virtual bool compute(const SafetyContext& aSafetyContext, Eigen::VectorXd& aSafeControlOut) = 0; 

private:
   
};
#endif //ISAFETYFILTER_HPP  