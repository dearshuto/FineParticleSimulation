//
//  additional_procedure.hpp
//  CubicFineParticleSimulation
//
//  Created by Shuto Shikama on 2016/10/02.
//
//

#ifndef additional_procedure_hpp
#define additional_procedure_hpp

namespace fj {
    class AdditionalProcedure;
}

class fj::AdditionalProcedure
{
protected:
    enum class Priority : unsigned int
    {
        kI_dont_care,
        kAbsolutelyLast,
    };
public:
    enum  class Target
    {
        kPOVRayOutput,
    };
public:
    
    AdditionalProcedure(const Priority priority)
    : m_priority(priority)
    {
        
    }
    
    virtual void startSimulationProfile() = 0;
    
    virtual void endSimulationProfile() = 0;
    
    virtual void terminate() = 0;
    
    unsigned int getPriorityAdUInt()const
    {
        return static_cast<unsigned int>(m_priority);
    }

private:
    
    const Priority m_priority;

};

#endif /* additional_procedure_hpp */
