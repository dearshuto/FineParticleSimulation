//
//  collapse_detector.cpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/09/11.
//
//

#include <btBulletDynamicsCommon.h>
#include "fine_particle/simulation/mohr_stress_circle.hpp"
#include "fine_particle/simulation/particle/collapse_detector.hpp"

bool fj::Particle::CollapseDetector::shouldCallapse(const fj::Particle &particle)const
{
    const auto kMohrStressCircle = generateMohrStressCircle(particle);
    
    return kMohrStressCircle.hasIntersectionPoint( particle.getWarrenSpringParameter() );
}

fj::MohrStressCircle fj::Particle::CollapseDetector::generateMohrStressCircle(const fj::Particle &particle)const
{
    // 粒子の回転情報を取得
    btTransform transform;
    particle.getMotionState()->getWorldTransform(transform);
    const btMatrix3x3& kRotationMatrix = transform.getBasis();
    
    // 粒子の形状を取得。法線があれば垂直抗力を算出できるね。
    auto faceNormals = fj::DiscritizedParticleShape::GetDiscritizedParticleShapeNormal(particle.getDiscretizedShapeType());
    
    // 法線を回転させる
    for (auto& normal :  faceNormals)
    {
        normal = kRotationMatrix * normal;
    }

    // 各法線方向にかかる垂直抗力を算出
    fj::MohrStressCircle mohrStressCircle;
    
    for (const auto& kNormal : faceNormals)
    {
        for (const btVector3& kNormalStress : particle.getContactForceContainer())
        {
            mohrStressCircle.addNormalStress(
                                             std::max( static_cast<btScalar>(0), kNormalStress.dot(-kNormal))
                                             );
        }
    }
    
    mohrStressCircle.rebuildMohrCircle();
    
    return mohrStressCircle;
}