#ifndef QPSOLVER_H
#define QPSOLVER_H
 
#include <OsqpEigen/Solver.hpp> 

struct Settings {
    double rho = 0.1;          // ADMM rho
    double epsAbs = 1e-4;      // absolute tolerance
    double epsRel = 1e-4;      // relative tolerance
    int    maxIter = 4000;
    double alpha = 1.6;        // relaxation
    bool   warmStart = true;
    bool   polish = true;
    int    verbose = 0;        // 0=silent, 1=prints
    double timeLimit_s = 0.0; // 0.0 => no limit
};

/**
 * Solves the Quadratic Programming problem in the form 
 *                          min ½ xᵀP x + qᵀx 
 *                          s.t. A x ≤ b
 */
class QpSolver 
{ 
public: 
    using SparseMat = Eigen::SparseMatrix<double, Eigen::ColMajor, long>; 
    using Vec = Eigen::VectorXd; 

    QpSolver();
    ~QpSolver();

    bool setProblemData(const SparseMat& P,
                        const Vec&       q,
                        const SparseMat& A,
                        const Vec&       l,
                        const Vec&       u,
                        const Settings&  s = Settings()); 

    bool solve(Vec& x_opt); 

    // -------- Helpers (dense → sparse) --------
    static SparseMat toSparse(const Eigen::MatrixXd& M) { return M.sparseView(); }
    static SparseMat makeUpperSymmetric(const SparseMat& P) 
    {
        // Use only upper-triangular numeric values; if lower has entries, mirror them up.
        SparseMat U = P.triangularView<Eigen::Upper>();
        SparseMat UT = SparseMat(P.transpose()).triangularView<Eigen::Upper>();
        // average to be safe if both had values (keeps PSD if original was symmetric PSD)
        return 0.5*(U + UT);
    }

    const std::string& lastError() const { return mLastErr; }
    int nVars() const { return n_; }
    int nCons() const { return m_; }

private:
    bool fail(const char* msg) { mLastErr = msg; return false; }

    OsqpEigen::Solver mSolver;
    int n_{0}, m_{0};
    SparseMat P_, A_;
    Vec q_, l_, u_;
    std::string mLastErr;
   
};
#endif //QPSOLVER_H