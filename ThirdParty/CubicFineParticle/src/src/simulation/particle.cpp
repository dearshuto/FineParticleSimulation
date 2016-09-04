//
//  particle.cpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/08/25.
//
//

#include <iostream>
#include "simulation/fine_particle_world.hpp"
#include "simulation/particle.hpp"

std::unique_ptr<btSphereShape> fj::Particle::SphereShape( new btSphereShape(0.5) );
std::unique_ptr<btSphereShape> fj::Particle::OverlapShape( new btSphereShape(10.0) );
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
    m_overlap.setCollisionShape(OverlapShape.get());
    m_effectRange.setCollisionShape(OverlapShape.get());
}

void fj::Particle::update(btScalar timestep)
{
    btTransform trans;
    getMotionState()->getWorldTransform(trans);
    m_overlap.setWorldTransform(trans);
}

void fj::Particle::setOverlapInWorld(fj::FineParticleWorld* world)
{
    m_overlap.setCollisionFlags( getCollisionFlags() |  btCollisionObject::CF_NO_CONTACT_RESPONSE );
    world->addCollisionObject(&m_overlap, fj::CollisionGroup::kOverlap, fj::CollisionFiltering::kOverlap );
    world->addCollisionObject(&m_effectRange, fj::CollisionGroup::kEffectRange,  fj::CollisionFiltering::kEffectRange );
}

bool fj::Particle::isCollapse()const
{

    return false;
}

int fj::Particle::overlappingSize()const
{
    return m_overlap.getOverlappingPairs().size();
}

const fj::Particle& fj::Particle::getOverlappingParticle(const int index)const
{
    // Overlapingはフィルタリングをかけてあるはず
    const fj::Particle* particle = static_cast<const fj::Particle*>( m_overlap.getOverlappingObject(index) );
    return std::cref(*particle);
}