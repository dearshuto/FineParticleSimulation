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

void fj::MohrStressCircle::rebuildMohrCircle(const btQuaternion& rotateMatrix)
{
    if (m_contactForce.empty())
    {
        Center = {0.0, 0.0};
        Radius = 0.0;
        return;
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

void fj::MohrStressCircle::rebuildCircleCenterAndRadius(const btQuaternion& rotateMatrix)
{
    // 各面にかかっている垂直応力の中の最大値と最小値を見つける.
    
    NormalStressContainer stressContainer = computeNormalStress(rotateMatrix);
    const auto& kMinMax = std::minmax_element(std::begin(stressContainer), std::end(stressContainer));
    const auto kMin = *std::get<0>(kMinMax);
    const auto kMax = *std::get<1>(kMinMax);

    Center = {static_cast<btScalar>(( kMin + kMax) / 2.0), 0};
    Radius = (kMax - kMin) / btScalar(2.);
}

fj::MohrStressCircle::NormalStressContainer fj::MohrStressCircle::computeNormalStress(const btQuaternion &rotateMatrix)const
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
        auto kNormal = discretizedShape->get(i);//rotateMatrix * discretizedShape->get(i);
//        kNormal *= rotateMatrix.inverse();
        
        for (const auto& stress: m_contactForce)
        {
            stressContainer[i] += std::max( static_cast<btScalar>(0.0), stress.dot({kNormal.x(), kNormal.y(), kNormal.z()}));
        }
    }
    
    return stressContainer;
}
