//
//  every_frame_simulation_time_profile.cpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/09/26.
//
//

#include <fstream>
#include "fine_particle/additional/profile/every_frame_simulation_time_profile.hpp"

void fj::EveryFrameSimulationTimeProfile::endSimulationProfile()
{
    Super::endSimulationProfile();
    
    std::ofstream output(getOutputDirectory() + "/" + std::to_string(getFrameCount()) + ".log");
    
    output << "Max: " << getMax() / 1000.0 << " sec" << std::endl;
    output << "Min: " << getMin() / 1000.0 << " sec" << std::endl;
    output << "Avg: " << getCurrentAverage() / 1000.0 << " sec" << std::endl;
    
    incrementFrameCount();
}

void fj::EveryFrameSimulationTimeProfile::terminate()
{
    // 毎フレーム出力してるので, 最後に何かする必要はない
}
