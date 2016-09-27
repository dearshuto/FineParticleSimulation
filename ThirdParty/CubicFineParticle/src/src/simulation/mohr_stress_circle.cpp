//
//  mohr_stress_circle.cpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/09/11.
//
//

#include <numeric>
#include "fine_particle/simulation/mohr_stress_circle.hpp"

void fj::MohrStressCircle::addNormalStress(const double normalStress)
{
    m_normalStress.push_back(normalStress);
}

void fj::MohrStressCircle::rebuildMohrCircle()
{
    if (m_normalStress.empty())
    {
        Center = {0.0, 0.0};
        Radius = 0.0;
        return;
    }
    
    const auto& kMinMax = std::minmax(std::begin(m_normalStress), std::end(m_normalStress));
    const auto kMin = *kMinMax.first;
    const auto kMax = *kMinMax.second;
    
    Center = {static_cast<btScalar>(( kMin + kMax) / 2.0), 0};
    Radius = (kMax - kMin) / btScalar(2.);
}

bool fj::MohrStressCircle::hasIntersectionPoint(const fj::WarrenSpringParameter& warrenSpringParameter)const
{
    
    
    return true;
}
