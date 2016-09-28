//
//  mohr_stress_circle_distribution.hpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/09/28.
//
//

#ifndef mohr_stress_circle_distribution_hpp
#define mohr_stress_circle_distribution_hpp

#include "simulation_profile.hpp"

namespace fj {
    class MohrStressCircleDistribution;
}

class fj::MohrStressCircleDistribution : public fj::SimulationProfile
{
    typedef fj::SimulationProfile Super;
public:
    MohrStressCircleDistribution()
    : Super(Priority::kI_dont_care)
    {
        
    }
    ~MohrStressCircleDistribution() = default;
    
    void startSimulationProfile()override;
    
    void endSimulationProfile()override;
    
    void terminate()override;
    
    void setGraph(const float min, const float max, const float duration);
    
private:
    float m_min;
    float m_max;
    float m_duration;
};

#endif /* mohr_stress_circle_distribution_hpp */
