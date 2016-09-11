//
//  particle.cpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/08/25.
//
//

#include <numeric>
#include <iostream>
#include "fine_particle/simulation/fine_particle_world.hpp"
#include "fine_particle/simulation/particle/particle.hpp"
#include "fine_particle/simulation/mohr_stress_circle.hpp"

std::unique_ptr<btSphereShape> fj::Particle::SphereShape( new btSphereShape(0.5) );
std::unique_ptr<btSphereShape> fj::Particle::OverlapShape( new btSphereShape(10.) );
std::unique_ptr<btBoxShape> fj::Particle::BoxShape( new btBoxShape(btVector3(1, 1, 1)) );

std::unique_ptr<fj::Particle> fj::Particle::generateParticle(const fj::DiscritizedParticleShape::ShapeType type, const btVector3& position)
{
    constexpr btScalar mass(0.1);
    const btVector3 localInertia(0,0,0);
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(position);
    std::unique_ptr<btDefaultMotionState> myMotionState(new btDefaultMotionState(transform));
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState.get(), fj::Particle::BoxShape.get(),localInertia);
    std::unique_ptr<fj::Particle> particle(new fj::Particle(type, rbInfo, std::move(myMotionState)));
    particle->setRollingFriction(1);
    particle->setFriction(1);

    return particle;
}

void fj::Particle::init()
{
    // アップキャストするために自分の情報をもたせておく
    m_internalType = btCollisionObject::CO_RIGID_BODY | btCollisionObject::CO_USER_TYPE;
    m_overlap.setCollisionShape(SphereShape.get());
    m_effectRange.setCollisionShape(OverlapShape.get());
}

void fj::Particle::updateCollisionShapePosition(btScalar timestep)
{
    btTransform trans;
    getMotionState()->getWorldTransform(trans);
    m_overlap.setWorldTransform(trans);
}

void fj::Particle::setOverlapInWorld(fj::FineParticleWorld* world)
{
    m_overlap.setCollisionFlags( getCollisionFlags() |  btCollisionObject::CF_NO_CONTACT_RESPONSE );
    world->addCollisionObject(&m_overlap, fj::CollisionGroup::kOverlap, fj::CollisionFiltering::kOverlap );
    
    m_effectRange.setCollisionFlags( getCollisionFlags() |  btCollisionObject::CF_NO_CONTACT_RESPONSE );
    world->addCollisionObject(&m_effectRange, fj::CollisionGroup::kEffectRange,  fj::CollisionFiltering::kEffectRange );
}

bool fj::Particle::isCollapse()const
{
    // 粒子の回転情報を取得
    btTransform transform;
    Super::getMotionState()->getWorldTransform(transform);
    const btMatrix3x3& kRotationMatrix = transform.getBasis();
    
    // 粒子の形状を取得。法線があれば垂直抗力を算出できるね。
    auto faceNormals = fj::DiscritizedParticleShape::GetDiscritizedParticleShapeNormal(getDiscretizedShapeType());
    
    // 法線を回転させる
    for (auto& normal :  faceNormals)
    {
        normal = kRotationMatrix * normal;
    }
    
    // 各法線方向にかかる垂直抗力を算出
    fj::MohrStressCircle mohrStressCircle;
    
    for (const auto& kNormal : faceNormals)
    {
        for (const btVector3& kNormalStress : m_contactForceContainer)
        {
            mohrStressCircle.addNormalStress(
                                   std::max( static_cast<btScalar>(0), kNormalStress.dot(-kNormal))
                                   );
        }
    }
    
    mohrStressCircle.rebuildMohrCircle();
    return mohrStressCircle.hasIntersectionPoint( m_warrenSpringParameter );
}

void fj::Particle::addContactForce(const btVector3& contactForce)
{
    m_contactForceContainer.push_back(contactForce);
}

void fj::Particle::applyContactForce()
{
    const btVector3 kContactForceSum = std::accumulate(std::begin(m_contactForceContainer), std::end(m_contactForceContainer), btVector3(0, 0, 0)/*初期値*/);
    applyCentralForce(kContactForceSum);
}

void fj::Particle::clearContactForce()
{
    m_contactForceContainer.clear();
}

int fj::Particle::overlappingSize()const
{
    return m_effectRange.getOverlappingPairs().size();
    return m_overlap.getOverlappingPairs().size();
}

const btCollisionObject* fj::Particle::getEffectObject(const int index)const
{
    return m_effectRange.getOverlappingObject(index);
}