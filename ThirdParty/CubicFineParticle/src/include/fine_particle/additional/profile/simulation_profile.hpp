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
#include <string>
#include "fine_particle/additional/additional_procedure.hpp"

namespace fj {
    class FineParticleWorld;
    class SimulationProfile;
}

/** Worldで行われるシミュレーションの情報を取得&整理する */
class fj::SimulationProfile : public fj::AdditionalProcedure
{
    typedef fj::AdditionalProcedure Super;
public:
    SimulationProfile() = delete;
    virtual~SimulationProfile() = default;
    
    SimulationProfile(const fj::FineParticleWorld& world, const Priority priority)
    : Super(priority)
    , m_world(world)
    , m_priority(priority)
    , m_outputDirectory("./")
    {
        
    }

    virtual void startSimulationProfile() = 0;
    
    virtual void endSimulationProfile() = 0;
    
    virtual void terminate() = 0;
    
    const std::string& getOutputDirectory()const
    {
        return m_outputDirectory;
    }
    
    void setOutputDirectory(const std::string& filename)
    {
        m_outputDirectory = filename;
    }
    
    unsigned int getPriorityAdUInt()const
    {
        return static_cast<unsigned int>(m_priority);
    }
    
protected:
    
    const fj::FineParticleWorld& getWorld()const
    {
        return m_world;
    }
    
private:
    const fj::FineParticleWorld& m_world;

    const Priority m_priority;
    
    std::string m_outputDirectory;
};


#endif /* simulation_profile_hpp */
