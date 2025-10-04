#ifndef ROSSTATEFETCHER_H
#define ROSSTATEFETCHER_H
 
#include "gpc/IStateFetcher.hpp"
 
class RosStateFetcher : public IStateFetcher
{ 
public:
    RosStateFetcher();
    ~RosStateFetcher() override; 

    Eigen::VectorXd fetchState() override; 


private:
   
};
#endif //ROSSTATEFETCHER_H