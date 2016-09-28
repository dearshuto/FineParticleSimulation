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
#include "simulation_profile.hpp"

namespace fj {
    class SimulationTimeProfile;
}

class fj::SimulationTimeProfile : public fj::SimulationProfile
{
    typedef fj::SimulationProfile Super;
    typedef std::chrono::system_clock::time_point time_point;
    typedef long long MilliSecTime;
public:
    SimulationTimeProfile()
    : Super(Priority::kAbsolutelyLast)
    , m_average(0)
    , m_max(0)
    , m_min(std::numeric_limits<MilliSecTime>::infinity())
    {
        
    }
    
    virtual~SimulationTimeProfile() = default;
    
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
