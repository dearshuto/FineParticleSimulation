//
//  simulation_time_profile.cpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/09/25.
//
//

#include <fstream>
#include <ostream>
#include "fine_particle/simulation/profile/simulation_time_profile.hpp"

void fj::SimulationTimeProfile::startSimulationProfile()
{
    m_start = std::chrono::system_clock::now();
}

void fj::SimulationTimeProfile::endSimulationProfile()
{
    auto currentStep = std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::system_clock::now() - m_start ).count();

    if (currentStep > m_max)
    {
        m_max = currentStep;
    }
    
    if (currentStep < m_min)
    {
        m_min = currentStep;
    }

    m_average = (m_average == 0) ? currentStep : (currentStep + m_average) / 2.0;
}

void fj::SimulationTimeProfile::terminate()
{
    std::ofstream output("simulation_time.log");
    
    output << "Max: " << m_max / 1000.0 << " sec" << std::endl;
    output << "Min: " << m_min / 1000.0 << " sec" << std::endl;
    output << "Avg: " << m_average / 1000.0 << " sec" << std::endl;
}
