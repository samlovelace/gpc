#ifndef ICONSTRAINT_HPP
#define ICONSTRAINT_HPP
 
#include "gpc/SafetyContext.hpp"
#include <eigen3/Eigen/Sparse>
 
class IConstraint 
{ 
public:
    virtual ~IConstraint() = default; 

    virtual int appendRows(const SafetyContext& ctx,
                           std::vector<Eigen::Triplet<double>>& A_triplets,
                           Eigen::VectorXd& l_accum,
                           Eigen::VectorXd& u_accum,
                           int rowOffset,
                           int zDim) = 0; 

private:
   
};
#endif //ICONSTRAINT_HPP