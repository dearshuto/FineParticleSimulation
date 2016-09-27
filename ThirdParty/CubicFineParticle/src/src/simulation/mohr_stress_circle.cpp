//
//  mohr_stress_circle.cpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/09/11.
//
//

#include <numeric>
#include "fine_particle/simulation/particle/discritized_particle_shape.hpp"
#include "fine_particle/simulation/mohr_stress_circle.hpp"

void fj::MohrStressCircle::addContactForce(const btVector3& normalStress)
{
    m_contactForce.push_back(normalStress);
}

void fj::MohrStressCircle::rebuildMohrCircle(const btMatrix3x3& rotateMatrix)
{
    if (m_contactForce.empty())
    {
        m_center = {{0.0, 0.0}};
        m_radius = 0.0;
    }
    else
    {
        rebuildCircleCenterAndRadius(rotateMatrix);
    }
}

void fj::MohrStressCircle::clearContactForce()
{
    m_contactForce.clear();
}

void fj::MohrStressCircle::rebuildCircleCenterAndRadius(const btMatrix3x3& rotateMatrix)
{
    // 各面にかかっている垂直応力の中の最大値と最小値を見つける.
    
    NormalStressContainer stressContainer = computeNormalStress(rotateMatrix);
    const auto& kMinMax = std::minmax(std::begin(stressContainer), std::end(stressContainer));
    const auto kMin = *kMinMax.first;
    const auto kMax = *kMinMax.second;
    
    m_center = {{static_cast<btScalar>(( kMin + kMax) / 2.0), 0}};
    m_radius = (kMax - kMin) / btScalar(2.);
}

fj::MohrStressCircle::NormalStressContainer fj::MohrStressCircle::computeNormalStress(const btMatrix3x3 &rotateMatrix)const
{
    // 各面に対してかかっている垂直応力を算出する
    
    // 離散化形状
    auto discretizedShape = fj::DiscritizedParticleShape::GetDiscretizedParticleShapeNormal(getDiscretizedShapeType());
    
    // 離散化形状の麺の数だけコンテナを用意して, 0で初期化しておく
    NormalStressContainer stressContainer;
    stressContainer.resize(discretizedShape->size());
    std::fill(stressContainer.begin(), stressContainer.end(), 0);
    
    // 離散化形状の各面の法線に回転成分を適用する. そのあとすべての接触力を評価する
    for (int i = 0; i < discretizedShape->size(); i++)
    {
        const auto kNormal = rotateMatrix * discretizedShape->get(i);
        
        for (const auto& stress: m_contactForce)
        {
            stressContainer[i] += std::max( static_cast<btScalar>(0.0), stress.dot(kNormal));
        }
    }
    
    return stressContainer;
}

bool fj::MohrStressCircle::hasIntersectionPoint(const fj::WarrenSpringParameter& warrenSpringParameter)const
{
    
    
    return true;
}
