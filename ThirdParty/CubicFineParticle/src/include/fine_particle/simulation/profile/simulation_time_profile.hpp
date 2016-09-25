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
    typedef long long MiliSecTime;
public:
    SimulationTimeProfile(const std::string& outputDirectory = ".")
    : Super(Priority::kAbsolutelyLast)
    , m_outputDirectory(outputDirectory)
    , m_average(0)
    , m_max(0)
    , m_min(std::numeric_limits<MiliSecTime>::infinity())
    {
        
    }
    
    ~SimulationTimeProfile() = default;
    
    void startSimulationProfile()override;
    
    void endSimulationProfile()override;
    
    void terminate()override;
    
private:
    std::string m_outputDirectory;
    
    time_point m_start;
    
    MiliSecTime m_average;
    
    MiliSecTime m_max;
    
    MiliSecTime m_min;
};

#endif /* simulation_time_profile_hpp */
