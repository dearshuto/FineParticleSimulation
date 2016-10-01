//
//  every_frame_simulation_time_profile.hpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/09/26.
//
//

#ifndef every_frame_simulation_time_profile_hpp
#define every_frame_simulation_time_profile_hpp

#include "fine_particle/simulation/profile/simulation_time_profile.hpp"

namespace fj {
    class EveryFrameSimulationTimeProfile;
}

class fj::EveryFrameSimulationTimeProfile : public fj::SimulationTimeProfile
{
    typedef fj::SimulationTimeProfile Super;
public:
    EveryFrameSimulationTimeProfile() = delete;
    ~EveryFrameSimulationTimeProfile() = default;
    
    EveryFrameSimulationTimeProfile(const fj::FineParticleWorld& world)
    : Super(world)
    , m_frame(0)
    {
        
    }

    void endSimulationProfile()override;
    
    void terminate()override;

    unsigned int getFrameCount()const
    {
        return m_frame;
    }
    
    void incrementFrameCount()
    {
        ++m_frame;
    }
private:
    unsigned int m_frame;
};

#endif /* every_frame_simulation_time_profile_hpp */
