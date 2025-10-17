
#include "gpc/EcbfDistanceConstraint.h"
#include <osqp.h> // OSQP_INFTY
#include <iostream> 

EcbfDistanceConstraint::EcbfDistanceConstraint()
{

}

EcbfDistanceConstraint::~EcbfDistanceConstraint()
{

}

int EcbfDistanceConstraint::appendRows(const SafetyContext& ctx,
                                        std::vector<Eigen::Triplet<double>>& A_triplets,
                                        Eigen::VectorXd& l_accum,
                                        Eigen::VectorXd& u_accum,
                                        int rowOffset,
                                        int zDim)
{
    const Eigen::Vector3d p = ctx.x.segment<3>(0); // robot position 
    const Eigen::Vector3d v = ctx.x.segment<3>(3); // robot velocity 

    const int colX = 0, colY = 1, colZ = 2; 

    // ECBF gains 
    const double a1 = 10; 
    const double a2 = 1; 
    constexpr double eps = 1e-12; 

    const int nObs = ctx.obstacles.size();
    const int oldRows = l_accum.size(); 
    const int newRows = oldRows + nObs; 
    
    // resize the bounds vectors 
    l_accum.conservativeResize(newRows);
    u_accum.conservativeResize(newRows); 
    
    int added = 0; 
    
    for(int k = 0; k < nObs; k++)
    {
        const auto& obs = ctx.obstacles[k]; 
        const double d = ctx.radius + obs.radius; 

        // Relative position/velocity 
        Eigen::Vector3d r    = p - obs.pos; 
        Eigen::Vector3d rdot = v - obs.vel;
        const double r2      = r.squaredNorm(); 
        const double rdot2   = rdot.squaredNorm(); 
        
        // ECBF bounds 
        const double h = r2 - d * d;
        const double r_dot_rd = r.dot(rdot); 
        const double r_dot_ao = r.dot(obs.acc); 
        
        // upper bound
        const double u_row = 2.0 * rdot2
                            - 2.0 * r_dot_ao
                            + 2.0 * a1 * r_dot_rd
                            + a2 * h;

        const int row = rowOffset + added;   

        A_triplets.emplace_back(row, colX, -2.0 * r.x()); 
        A_triplets.emplace_back(row, colY, -2.0 * r.y());
        A_triplets.emplace_back(row, colZ, -2.0 * r.z());
        
        // Bounds
        l_accum(row) = -OSQP_INFTY;
        u_accum(row) = u_row;

        added++;
    }

    return added; 
}