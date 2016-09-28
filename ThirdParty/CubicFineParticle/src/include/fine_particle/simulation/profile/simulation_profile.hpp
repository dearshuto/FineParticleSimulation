//
//  simulation_profile.hpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/09/25.
//
//

#ifndef simulation_profile_hpp
#define simulation_profile_hpp

#include <memory>

namespace fj {
    class FineParticleWorld;
    class SimulationProfile;
}

/** Worldで行われるシミュレーションの情報を取得&整理する */
class fj::SimulationProfile
{
protected:
    enum class Priority : unsigned int
    {
        kI_dont_care,
        kAbsolutelyLast,
    };
public:
    SimulationProfile() = delete;
    virtual~SimulationProfile() = default;
    
    SimulationProfile(const Priority priority)
    : m_priority(priority)
    {
        
    }

    virtual void startSimulationProfile() = 0;
    
    virtual void endSimulationProfile() = 0;
    
    virtual void terminate() = 0;
    
    void registerWorld(std::weak_ptr<fj::FineParticleWorld>& world)
    {
        m_world = world;
    }
    
    unsigned int getPriorityAdUInt()const
    {
        return static_cast<unsigned int>(m_priority);
    }
protected:
    
    const std::weak_ptr<fj::FineParticleWorld>& getWorld()const
    {
        return m_world;
    }
    
private:
    const Priority m_priority;
    
    std::weak_ptr<fj::FineParticleWorld> m_world;
};


#endif /* simulation_profile_hpp */
