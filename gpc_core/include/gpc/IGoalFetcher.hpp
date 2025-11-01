#ifndef IGOALFETCHER_HPP
#define IGOALFETCHER_HPP
 
#include <memory> 
#include <eigen3/Eigen/Dense> 
#include "gpc/GuidanceSystem.h"

class IGoalFetcher 
{ 
public:
    virtual ~IGoalFetcher() = default; 
    virtual bool startListening() = 0;  

    virtual void bindGuidance(std::shared_ptr<GuidanceSystem> aGuidancePtr) {mGuidance = aGuidancePtr; }

protected: 
    std::shared_ptr<GuidanceSystem> mGuidance; 
   
};
#endif //IGOALFETCHER_HPP