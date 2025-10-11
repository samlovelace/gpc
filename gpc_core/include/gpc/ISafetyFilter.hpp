#ifndef ISAFETYFILTER_HPP
#define ISAFETYFILTER_HPP
 
#include <eigen3/Eigen/Dense>
 
class ISafetyFilter 
{ 
public:
    virtual ~ISafetyFilter() = default; 
    virtual bool compute(Eigen::VectorXd& aSafeControlOut) = 0; 

private:
   
};
#endif //ISAFETYFILTER_HPP  