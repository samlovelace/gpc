#ifndef ISTATEFETCHER_HPP
#define ISTATEFETCHER_HPP
 
#include <eigen3/Eigen/Dense>
 
class IStateFetcher 
{ 
public:
    virtual ~IStateFetcher() = default; 
    virtual Eigen::VectorXd fetchState() = 0; 

private:
   
};
#endif //ISTATEFETCHER_HPP