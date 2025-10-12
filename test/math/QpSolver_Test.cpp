#include "gtest/gtest.h"
#include "gpc/QpSolver.h"
#include <osqp/osqp.h> // OSQP_INFTY

class QpSolver_Test : public ::testing::Test
{
protected:
    QpSolver_Test() = default; 
    ~QpSolver_Test() = default; 

    QpSolver solver; 

    void SetUp(){}
    void TearDown(){}

private:

};

TEST_F(QpSolver_Test, Simple_Unconstrained)
{
    Eigen::Matrix3d P; 
    P << 2, 0, 0,
         0, 4, 0, 
         0, 0, 6; 

    Eigen::Vector3d q; 
    q << -2, -8, 12;

    // A = I, lb=-inf, ub=+inf  -> unconstrained solution
    Eigen::Matrix3d A = Eigen::Matrix3d::Identity();

    Eigen::Vector3d ub, lb;   
    ub << OSQP_INFTY, OSQP_INFTY, OSQP_INFTY;
    lb = -ub; 

    SparseMat Ps = QpSolver::toSparse(P); 
    Ps = QpSolver::makeUpperSymmetric(Ps); 
    SparseMat As = QpSolver::toSparse(A); 

    // setup problem 
    ASSERT_TRUE(solver.setProblemData(Ps, q, As, lb, ub));

    // solve problem 
    Eigen::VectorXd solution;
    ASSERT_TRUE(solver.solve(solution)); 
    
    // check against expected 
    Eigen::Vector3d expected; 
    expected << 1, 2, -2; 
    
    ASSERT_NEAR(expected(0), solution(0), 1e-4);
    ASSERT_NEAR(expected(1), solution(1), 1e-4);
    ASSERT_NEAR(expected(2), solution(2), 1e-4); 
}