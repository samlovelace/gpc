
#include "gpc/QpSafetyFilter.h"
#include "gpc/EigenPrinter.hpp"

QpSafetyFilter::QpSafetyFilter(Eigen::VectorXd& aVecOfWeights) : mWeights(aVecOfWeights)
{
    // IOFormat: (precision, flags, coeffSeparator, rowSeparator, rowPrefix, rowSuffix, matPrefix, matSuffix)
    Eigen::IOFormat singleLine(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", " ", "", "", "[", "]");
    std::cout << "[QpSafetyFilter] weights = " << mWeights.format(singleLine) << std::endl; 
}

QpSafetyFilter::~QpSafetyFilter()
{

}

void QpSafetyFilter::configure(int zDim)
{
    mZdim = zDim;
    
    //mUlast = Eigen::VectorXd::Zero(mZdim);
    //mBias  = Eigen::VectorXd::Zero(mZdim);

    buildH(); 
}

bool QpSafetyFilter::buildH()
{
    // Cost: 1/2 (u - u_nom - bias)^T (2w I) (u - u_nom - bias)
    // => H = 2w I,  f = -2w (u_nom + bias)
    std::vector<Eigen::Triplet<double>> Ht;
    Ht.reserve(mZdim);
    mH.resize(mZdim, mZdim);

    for (int i = 0; i < mZdim; ++i) 
    {
        Ht.emplace_back(i, i, 2.0 * mWeights(i));
    }

    mH.setFromTriplets(Ht.begin(), Ht.end());
}

void QpSafetyFilter::setConstraints(const std::vector<std::shared_ptr<IConstraint>>& aConstraintsVec)
{
    mConstraints = aConstraintsVec; 
}

void QpSafetyFilter::buildProblem(QpProblem& aProblem, const SafetyContext& aSafetyContext)
{
    // rebuild linear part 
    mf = -2 * mH * aSafetyContext.u_nom; 

    // ---- Constraints: accumulate triplets and bounds ----
    std::vector<Eigen::Triplet<double>> Atriplets;
    Atriplets.reserve(64);
    Eigen::VectorXd l(0), u(0);
    int rowOff = 0;
 
    for (const auto& c : mConstraints) 
    {
       rowOff += c->appendRows(aSafetyContext, Atriplets, l, u, rowOff, mZdim); 
    }

    // TODO: might be needed 
    // If no constraints, add a dummy 0 <= 0 <= 0 row to please some solvers (optional)
    // if (Atriplets.empty()) {
    //     Atrips.emplace_back(0, 0, 0.0);
    //     l.conservativeResize(1); u.conservativeResize(1);
    //     l(0) = 0.0; u(0) = 0.0;
    // }

    const int m = static_cast<int>(l.size());
    aProblem.H = mH;
    aProblem.f = mf;  
    aProblem.A.resize(m, mZdim);
    aProblem.A.setFromTriplets(Atriplets.begin(), Atriplets.end());
    aProblem.lb = l;
    aProblem.ub = u;
}

bool QpSafetyFilter::compute(const SafetyContext& aSafetyContext, Eigen::VectorXd& aSafeControlOut)
{
    std::cout << "[QpSafetyFilter] Optimzing safe control input..." << std::endl;

    // setup Qp problem 
    //   * recompute linear part of Qp problem
    //   * recompute constraints 
    QpProblem problem;  
    buildProblem(problem, aSafetyContext); 

    // std::cout << "H: " << problem.H; 
    // std::cout << "f: " << problem.f; 
    // std::cout << "A: " << problem.A; 
    // std::cout << "lb: " << problem.lb; 
    // std::cout << "ub: " << problem.ub; 
    
    mSolver.setProblemData(problem.H, problem.f, problem.A, problem.lb, problem.ub); 

    if(!mSolver.solve(aSafeControlOut))
    {
        std::cout << "Failed to compute safe control input" << std::endl; 
        return false; 
    } 

    return true; 
}