//
//  simulation_profile.hpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/09/25.
//
//

#ifndef simulation_profile_hpp
#define simulation_profile_hpp

namespace fj {
    class SimulationProfile;
}

class fj::SimulationProfile
{
protected:
    enum class Priority : unsigned int
    {
        kAbsolutelyLast,
    };
public:
    SimulationProfile() = delete;
    virtual~SimulationProfile() = default;
    
    SimulationProfile(const Priority priority)
    : m_priority(priority)
    {
        
    }
    
    
    
protected:
    unsigned int getPriorityAdUInt()const
    {
        return static_cast<unsigned int>(m_priority);
    }
    
private:
    const Priority m_priority;
};


#endif /* simulation_profile_hpp */
