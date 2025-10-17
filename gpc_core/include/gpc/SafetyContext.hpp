#ifndef SAFETYCONTEXT_HPP
#define SAFETYCONTEXT_HPP

#include <eigen3/Eigen/Dense>

struct Obstacle {
    Eigen::Vector3d pos;
    Eigen::Vector3d vel; 
    Eigen::Vector3d acc; 
    double radius;
}; 

struct SafetyContext {
    Eigen::VectorXd x;          // current state
    Eigen::VectorXd xdot;       // optional (empty if N/A)
    Eigen::VectorXd u_nom;      // nominal control from your controller
    double          dt{0.0};    // control period
    
    double radius; 
    std::vector<Obstacle> obstacles; // cartesian location of any obstacles 
};
#endif 