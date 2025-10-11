
#include "gpc/QpSolver.h"

QpSolver::QpSolver() : mSolver()
{

}

QpSolver::~QpSolver()
{

}

// Canonical OSQP form:  min 0.5 x' P x + q' x   s.t.  l <= A x <= u
bool QpSolver::setProblemData(const SparseMat& P,
                              const Vec&       q,
                              const SparseMat& A,
                              const Vec&       l,
                              const Vec&       u,
                              const Settings&  s)
{
    // --- dimension checks ---
    const int n = static_cast<int>(P.rows());
    if (P.cols() != n)               return fail("P must be square");
    if (q.size() != n)               return fail("q size must equal n");
    if (A.cols() != n)               return fail("A cols must equal n");
    const int m = static_cast<int>(A.rows());
    if (l.size() != m || u.size()!=m) return fail("l/u size must equal A rows");

    n_ = n; m_ = m;

    // OSQP uses upper-triangular part of P; ensure it’s symmetric in that view.
    SparseMat P_up = makeUpperSymmetric(P);

    // Store (optional, useful for updates)
    P_ = P_up; A_ = A; q_ = q; l_ = l; u_ = u;

    // --- load into OSQP-Eigen ---
    mSolver.data()->setNumberOfVariables(n_);
    mSolver.data()->setNumberOfConstraints(m_);
    if (!mSolver.data()->setHessianMatrix(P_))           return fail("setHessianMatrix failed");
    if (!mSolver.data()->setGradient(q_))                return fail("setGradient failed");
    if (!mSolver.data()->setLinearConstraintsMatrix(A_)) return fail("setA failed");
    if (!mSolver.data()->setLowerBound(l_))              return fail("setLowerBound failed");
    if (!mSolver.data()->setUpperBound(u_))              return fail("setUpperBound failed");

    // settings
    mSolver.settings()->setVerbosity(s.verbose);
    mSolver.settings()->setWarmStart(s.warmStart);
    mSolver.settings()->setPolish(s.polish);
    mSolver.settings()->setMaxIteration(s.maxIter);
    mSolver.settings()->setAbsoluteTolerance(s.epsAbs);
    mSolver.settings()->setRelativeTolerance(s.epsRel);
    mSolver.settings()->setRho(s.rho);
    mSolver.settings()->setAlpha(s.alpha);
    if (s.timeLimit_s > 0.0) mSolver.settings()->setTimeLimit(s.timeLimit_s);

    if (!mSolver.initSolver()) return fail("initSolver failed");
    return true;
}

bool QpSolver::solve(Vec& x_opt)
{
    if (n_ == 0) return fail("problem not set");

    auto rc = mSolver.solveProblem();
    
    if (rc != OsqpEigen::ErrorExitFlag::NoError)
        return fail(("OSQP solveProblem error code: " + std::to_string(static_cast<int>(rc))).c_str());

    x_opt = mSolver.getSolution();   
    return true;
}
