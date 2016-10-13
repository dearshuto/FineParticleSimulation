//
//  mohr_stress_circle_profile.hpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/09/27.
//
//

#ifndef mohr_stress_circle_profile_hpp
#define mohr_stress_circle_profile_hpp

#include "fine_particle/additional/additional_procedure.hpp"

namespace fj {
    class MohrStressCircleProfile;
}

/** n番目に生成された粒子の崩壊曲線を可視化する. gnuplotで描画できる形式で書き出す.
 * 本来なら複数の粒子を追跡できるようにしたいが, とりあえず1つの粒子だけ*/
class fj::MohrStressCircleProfile : public fj::AdditionalProcedure
{
    typedef fj::AdditionalProcedure Super;
public:
    MohrStressCircleProfile() = delete;
    ~MohrStressCircleProfile() = default;

    MohrStressCircleProfile(const fj::FineParticleWorld& world)
    : Super(Priority::kI_dont_care, world)
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
