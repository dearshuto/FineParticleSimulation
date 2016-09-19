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
    
    const btVector3& center = t.getOrigin();
    btVector3 extent(getMargin(),getMargin(),getMargin());
    aabbMin = center - extent;
    aabbMax = center + extent;
}

btScalar fj::FineParticleShape::getRigidRadius()const
{
    return Super::getRadius();
}

btScalar fj::FineParticleShape::getEffectRangeRadius()const
{
    return m_effectRangeRadius;
}