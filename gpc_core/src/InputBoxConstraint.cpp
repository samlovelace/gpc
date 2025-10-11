
#include "gpc/InputBoxConstraint.h"
#include <osqp.h> // OSQP_INFTY 
#include <iostream> 

InputBoxConstraint::InputBoxConstraint(Eigen::VectorXd u_min, Eigen::VectorXd u_max) : mUmin(u_min), mUmax(u_max)
{
    // IOFormat: (precision, flags, coeffSeparator, rowSeparator, rowPrefix, rowSuffix, matPrefix, matSuffix)
    Eigen::IOFormat singleLine(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", " ", "", "", "[", "]");
    std::cout << "[InputBoxConstraint] min = " << mUmin.format(singleLine) << std::endl;
    std::cout << "[InputBoxConstraint] max = " << mUmax.format(singleLine) << std::endl;
}

InputBoxConstraint::~InputBoxConstraint()
{

}

int InputBoxConstraint::appendRows(const SafetyContext& ctx,
                                   std::vector<Eigen::Triplet<double>>& A_triplets,
                                   Eigen::VectorXd& l_accum,
                                   Eigen::VectorXd& u_accum,
                                   int rowOffset,
                                   int zDim)
{
    const int nu = zDim;
    const int rows = nu;

    // grow bounds
    const int oldSize = static_cast<int>(l_accum.size());
    l_accum.conservativeResize(oldSize + rows);
    u_accum.conservativeResize(oldSize + rows);

    // one row per scalar u(i):  l_i <= [0..1..0]·u <= u_i
    for (int i = 0; i < nu; ++i) 
    {
        A_triplets.emplace_back(rowOffset + i, i, 1.0);
        l_accum(oldSize + i) = mUmin.size() ? mUmin(i) : -OSQP_INFTY;
        u_accum(oldSize + i) = mUmax.size() ? mUmax(i) :  OSQP_INFTY;
    }

    return rows;
}