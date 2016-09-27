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
    MohrStressCircleProfile()
    : Super(Priority::kI_dont_care)
    {
        
    }
    ~MohrStressCircleProfile() = default;
    
    void startSimulationProfile()override;
    
    void endSimulationProfile()override;
    
    void terminate()override;
};

#endif /* mohr_stress_circle_profile_hpp */
