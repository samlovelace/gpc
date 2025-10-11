#ifndef INPUTBOXCONSTRAINT_H
#define INPUTBOXCONSTRAINT_H

#include "ISafetyFilter.hpp"
#include "IConstraint.hpp" 
#include <eigen3/Eigen/Dense>

class InputBoxConstraint : public IConstraint
{ 
public:
    InputBoxConstraint(Eigen::VectorXd u_min, Eigen::VectorXd u_max);
    ~InputBoxConstraint() override; 

    int appendRows(const SafetyContext& ctx,
                   std::vector<Eigen::Triplet<double>>& A_triplets,
                   Eigen::VectorXd& l_accum,
                   Eigen::VectorXd& u_accum,
                   int rowOffset,
                   int zDim) override;

private:
    Eigen::VectorXd mUmin; 
    Eigen::VectorXd mUmax; 
   
};
#endif //INPUTBOXCONSTRAINT_H