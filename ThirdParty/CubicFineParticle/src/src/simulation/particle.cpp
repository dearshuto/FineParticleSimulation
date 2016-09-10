//
//  particle.cpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/08/25.
//
//

#include <iostream>
#include "fine_particle/simulation/fine_particle_world.hpp"
#include "fine_particle/simulation/particle.hpp"

std::unique_ptr<btSphereShape> fj::Particle::SphereShape( new btSphereShape(0.5) );
std::unique_ptr<btSphereShape> fj::Particle::OverlapShape( new btSphereShape(10.) );
std::unique_ptr<btBoxShape> fj::Particle::BoxShape( new btBoxShape(btVector3(1, 1, 1)) );

std::unique_ptr<fj::Particle> fj::Particle::generateParticle(const double x, const double y, const double z)
{
    constexpr btScalar mass(0.1);
    const btVector3 localInertia(0,0,0);
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(btVector3(x, y, z));
    std::unique_ptr<btDefaultMotionState> myMotionState(new btDefaultMotionState(transform));
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState.get(), fj::Particle::BoxShape.get(),localInertia);
    std::unique_ptr<fj::Particle> particle(new fj::Particle(rbInfo, std::move(myMotionState)));
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
    
    return true;
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