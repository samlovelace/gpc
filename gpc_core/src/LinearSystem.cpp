
#include "gpc/LinearSystem.h"
#include <iostream>

LinearSystem::LinearSystem(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C) : A(A), B(B), C(C)
{
    
}

LinearSystem::~LinearSystem()
{

}

bool LinearSystem::propagate()
{

}