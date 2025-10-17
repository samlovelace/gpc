#ifndef ECBFDISTANCECONSTRAINT_H
#define ECBFDISTANCECONSTRAINT_H
 
#include "gpc/IConstraint.hpp"
 
class EcbfDistanceConstraint : public IConstraint
{ 
public:
    EcbfDistanceConstraint();
    ~EcbfDistanceConstraint() override; 

    int appendRows(const SafetyContext& ctx,
                   std::vector<Eigen::Triplet<double>>& A_triplets,
                   Eigen::VectorXd& l_accum,
                   Eigen::VectorXd& u_accum,
                   int rowOffset,
                   int zDim); 

private:
   
};
#endif //ECBFDISTANCECONSTRAINT_H   