#ifndef LINEARSYSTEM_H
#define LINEARSYSTEM_H 

#include "IDynamicSystem.hpp"
#include <eigen3/Eigen/Dense>

class LinearSystem : public IDynamicSystem
{ 
public:
    LinearSystem();
    ~LinearSystem() override; 

    bool propagate() override; 

private:


};
#endif //LINEARSYSTEM_H