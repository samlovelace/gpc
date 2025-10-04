
#include "gpc.h"`
#include "RosStateFetcher.h"

int main()
{
    auto stateFetcher = std::make_shared<RosStateFetcher>(); 

    ControlSystem cs(stateFetcher);
    cs.init("../../configuration/", "config.yaml");  
    cs.run(); 
}