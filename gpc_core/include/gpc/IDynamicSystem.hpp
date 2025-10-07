#ifndef IDYNAMICSYSTEM_HPP
#define IDYNAMICSYSTEM_HPP
 
 
class IDynamicSystem 
{ 
public:
    virtual ~IDynamicSystem() = default; 
    virtual bool propagate() = 0; 

private:
   
};
#endif //IDYNAMICSYSTEM_HPP