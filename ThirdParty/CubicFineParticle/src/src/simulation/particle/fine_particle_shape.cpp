//
//  fine_particle_shape.cpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/09/15.
//
//

#include "fine_particle/simulation/particle/fine_particle_shape.hpp"

void fj::FineParticleShape::getAabb(const btTransform &t, btVector3 &aabbMin, btVector3 &aabbMax)const
{
    // 親クラスのSphereShapeでは, 球に外接するAABBが設定される.
    // AABBを2倍に拡張しておくと, 粒子が少し離れた状態でブロードフェーズの判定が通る.
    //この離れる距離が影響範囲に対応する
    // TODO: 2倍でいいのかな？動的に変形させるべきでは？
    
    Super::getAabb(t, aabbMin, aabbMax);
    
    aabbMin *= btScalar(2.0);
    aabbMax *= btScalar(2.0);
}

btScalar fj::FineParticleShape::getRigidRadius()const
{
    return m_rigidRadius;
}

btScalar fj::FineParticleShape::getEffectRangeRadius()const
{
    return Super::getRadius();
}