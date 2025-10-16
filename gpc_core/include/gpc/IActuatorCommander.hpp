#ifndef IACTUATORCOMMANDER_HPP
#define IACTUATORCOMMANDER_HPP
 
#include <eigen3/Eigen/Dense> 
 
class IActuatorCommander 
{ 
public:
    virtual ~IActuatorCommander() = default; 

    virtual void send(Eigen::VectorXd aControlInput) = 0; 

private:
   
};
#endif //IACTUATORCOMMANDER_HPP