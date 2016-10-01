//
//  mohr_stress_circle_profile.hpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/09/27.
//
//

#ifndef mohr_stress_circle_profile_hpp
#define mohr_stress_circle_profile_hpp

#include "simulation_profile.hpp"


namespace fj {
    class MohrStressCircleProfile;
}

class fj::MohrStressCircleProfile : public fj::SimulationProfile
{
    typedef fj::SimulationProfile Super;
public:
    MohrStressCircleProfile() = delete;
    ~MohrStressCircleProfile() = default;

    MohrStressCircleProfile(const fj::FineParticleWorld& world)
    : Super(world, Priority::kI_dont_care)
    {
        
    }
    
    void startSimulationProfile()override;
    
    void endSimulationProfile()override;
    
    void terminate()override;
    
    void setFilter(const std::function<bool(const int index)>& function)
    {
        m_filterFunction = function;
    }
    
private:
    std::function<bool(const int index)> m_filterFunction;
};

#endif /* mohr_stress_circle_profile_hpp */
