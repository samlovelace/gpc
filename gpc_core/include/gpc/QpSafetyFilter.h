#ifndef QPSAFETYFILTER_H
#define QPSAFETYFILTER_H
 
#include "gpc/ISafetyFilter.hpp"
#include "gpc/IConstraint.hpp"
#include "gpc/QpSolver.h"
 
class QpSafetyFilter : public ISafetyFilter
{ 
public:
    QpSafetyFilter(Eigen::VectorXd& aVecOfWeights);
    ~QpSafetyFilter() override; 

    void configure(int aDim) override;
    void setConstraints(const std::vector<std::shared_ptr<IConstraint>>& aConstraintsVec) override; 
    bool compute(const SafetyContext& aSafetyContext, Eigen::VectorXd& aSafeControlOut) override; 

private:

    QpSolver mSolver; 
    std::vector<std::shared_ptr<IConstraint>> mConstraints; 

    SparseMat mH; 
    Vec mf; 

    Eigen::VectorXd mWeights; 
    int mZdim; 

private: 

    bool buildH();
    void buildProblem(QpProblem& aProblem, const SafetyContext& aSafetyContext);
   
};
#endif //QPSAFETYFILTER_H   