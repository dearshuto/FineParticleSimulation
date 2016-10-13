//
//  simulation_time_profile.hpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/09/25.
//
//

#ifndef simulation_time_profile_hpp
#define simulation_time_profile_hpp

#include <chrono>
#include <limits>
#include <string>
#include "fine_particle/additional/additional_procedure.hpp"

namespace fj {
    class SimulationTimeProfile;
}

/** シミュレーションの開始から終了までの間で, 1フレームのシミュレーション時間を計測する */
class fj::SimulationTimeProfile : public fj::AdditionalProcedure
{
    typedef fj::AdditionalProcedure Super;
    typedef std::chrono::system_clock::time_point time_point;
    typedef long long MilliSecTime;
public:
    SimulationTimeProfile() = delete;
    virtual~SimulationTimeProfile() = default;
    
    SimulationTimeProfile(const fj::FineParticleWorld& world)
    : Super(Priority::kAbsolutelyLast, world)
    , m_average(0)
    , m_max(0)
    , m_min(std::numeric_limits<MilliSecTime>::infinity())
    {
        
    }

    virtual void startSimulationProfile()override;
    
    virtual void endSimulationProfile()override;
    
    virtual void terminate()override;
    
protected:
    
    void updateSimulationTimeMinMax(const MilliSecTime& currentStep);
    
    void updateSimulationTimeAverage(const MilliSecTime& currentStep);
    
public:
    
    const time_point& getStartTime()const
    {
        return m_start;
    }
    
    const MilliSecTime& getCurrentAverage()const
    {
        return m_average;
    }
    
    const MilliSecTime& getMax()const
    {
        return m_max;
    }
    
    const MilliSecTime& getMin()const
    {
        return m_min;
    }
    
private:
    
    time_point m_start;
    
    MilliSecTime m_average;
    
    MilliSecTime m_max;
    
    MilliSecTime m_min;
};

#endif /* simulation_time_profile_hpp */
