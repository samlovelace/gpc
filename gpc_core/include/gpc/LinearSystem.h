#ifndef LINEARSYSTEM_H
#define LINEARSYSTEM_H 

#include "IDynamicSystem.hpp"
#include <eigen3/Eigen/Dense>

class LinearSystem : public IDynamicSystem
{ 
public:
    LinearSystem(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C);
    ~LinearSystem() override; 

    bool propagate() override; 

private:
    Eigen::MatrixXd A; 
    Eigen::MatrixXd B; 
    Eigen::MatrixXd C;


};
#endif //LINEARSYSTEM_H