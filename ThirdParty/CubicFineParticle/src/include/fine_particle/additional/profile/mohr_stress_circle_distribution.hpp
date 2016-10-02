//
//  mohr_stress_circle_distribution.hpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/09/28.
//
//

#ifndef mohr_stress_circle_distribution_hpp
#define mohr_stress_circle_distribution_hpp

#include "fine_particle/additional/additional_procedure.hpp"

namespace fj {
    class MohrStressCircleDistribution;
}

class fj::MohrStressCircleDistribution : public fj::AdditionalProcedure
{
    typedef fj::AdditionalProcedure Super;
public:
    MohrStressCircleDistribution() = delete;
    ~MohrStressCircleDistribution() = default;

    MohrStressCircleDistribution(const fj::FineParticleWorld& world)
    : Super(Priority::kI_dont_care, world)
    {
        
    }

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
