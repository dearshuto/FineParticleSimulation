//
//  mohr_stress_circle_distribution.cpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/09/28.
//
//

#include <fstream>
#include <vector>
#include "fine_particle/simulation/fine_particle_world.hpp"
#include "fine_particle/additional/profile/mohr_stress_circle_distribution.hpp"

void fj::MohrStressCircleDistribution::startSimulationProfile()
{
    
}

void fj::MohrStressCircleDistribution::endSimulationProfile()
{

    const auto& frame = fj::FineParticleWorld::GetSimulationStep();
    const auto& world = getFineParticleWorld();
    
    std::vector<unsigned int> data;
    
    data.resize(( (m_max - m_min)/m_duration ) + 1);
    std::fill(data.begin(), data.end(), 0);
    
    for (int i = 0; i < world.getParticles().size(); i++)
    {
        const auto& particle = world.getParticles()[i];
        const float kRadius = particle->getMohrStressCircle().getRadius();
        const auto kClamp = std::max(m_min, std::min(kRadius, m_max));
        const int kCell = static_cast<int>(kClamp / m_duration);
        
        ++(data[kCell]);
    }
    
    std::ofstream output("distribution_" + std::to_string(frame) + ".data");
    std::ofstream command("distribution_" + std::to_string(frame) + ".gnuplot");
    for (int i = 0; i < data.size(); i++)
    {
        output << m_duration * i << " " << data[i] << std::endl;
    }
    
    command << "reset" << std::endl;
    command << "set terminal png" << std::endl;
    command << "set output \"distribution_" << frame << ".png\"" << std::endl;
    command << "plot \"distribution_" << frame << ".data\" using 1:2 with lines" << std::endl;
}

void fj::MohrStressCircleDistribution::terminate()
{
    
}

void fj::MohrStressCircleDistribution::setGraph(const float min, const float max, const float duration)
{
    m_min = min;
    m_max = max;
    m_duration = duration;
}
