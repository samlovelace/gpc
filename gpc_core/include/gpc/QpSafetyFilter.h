#ifndef QPSAFETYFILTER_H
#define QPSAFETYFILTER_H
 
#include "gpc/ISafetyFilter.hpp"
#include "gpc/QpSolver.h"
 
class QpSafetyFilter : public ISafetyFilter
{ 
public:
    QpSafetyFilter();
    ~QpSafetyFilter() override; 

    bool compute(Eigen::VectorXd& aSafeControlOut) override; 

private:

    QpSolver mSolver; 
   
};
#endif //QPSAFETYFILTER_H   