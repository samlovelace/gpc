#ifndef SAFETYCONTEXT_HPP
#define SAFETYCONTEXT_HPP

#include <eigen3/Eigen/Dense>

struct SafetyContext {
    Eigen::VectorXd x;          // current state
    Eigen::VectorXd xdot;       // optional (empty if N/A)
    Eigen::VectorXd u_nom;      // nominal control from your controller
    double          dt{0.0};    // control period
};

#endif 