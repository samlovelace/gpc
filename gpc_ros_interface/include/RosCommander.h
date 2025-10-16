#ifndef ROSCOMMANDER_H
#define ROSCOMMANDER_H
 
#include "gpc/IActuatorCommander.hpp" 

class RosCommander : public IActuatorCommander
{ 
public:
    RosCommander();
    ~RosCommander() override; 

    void send(Eigen::VectorXd aControlInput) override; 

private:
   
};
#endif //ROSCOMMANDER_H