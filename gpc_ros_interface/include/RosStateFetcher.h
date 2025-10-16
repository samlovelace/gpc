#ifndef ROSSTATEFETCHER_H
#define ROSSTATEFETCHER_H
 
#include "gpc/IStateFetcher.hpp"
#include "robot_idl/msg/abv_state.hpp"
#include <mutex> 

class RosStateFetcher : public IStateFetcher
{ 
public:
    RosStateFetcher();
    ~RosStateFetcher() override; 

    Eigen::VectorXd fetchState() override; 

private: 
    void stateCallback(robot_idl::msg::AbvState::SharedPtr aMsg); 
    void setLatestState(Eigen::VectorXd aState); 
        
private:
    Eigen::VectorXd mLatestState; 
    std::mutex mStateMutex; 
   
};
#endif //ROSSTATEFETCHER_H