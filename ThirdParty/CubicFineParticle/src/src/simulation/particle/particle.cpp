//
//  particle.cpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/08/25.
//
//

#include <functional>
#include <iostream>
#include <numeric>

#include "fine_particle/simulation/fine_particle_world.hpp"
#include "fine_particle/simulation/mohr_stress_circle.hpp"
#include "fine_particle/simulation/particle/fine_particle_shape.hpp"
#include "fine_particle/simulation/particle/particle.hpp"

fj::FineParticleShape fj::Particle::CollisionShape(0.5);

std::unique_ptr<fj::Particle> fj::Particle::generateParticle(const fj::DiscritizedParticleShape::ShapeType type, const btVector3& position)
{
    constexpr btScalar mass(0.1);
    const btVector3 localInertia(0,0,0);
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(position);
    std::unique_ptr<btDefaultMotionState> myMotionState(new btDefaultMotionState(transform));
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState.get(), &fj::Particle::CollisionShape,localInertia);
    std::unique_ptr<fj::Particle> particle(new fj::Particle(type, rbInfo, std::move(myMotionState)));
    particle->setRollingFriction(1);
    particle->setFriction(1);
    return particle;
}

void fj::Particle::updateCollapseStatus()
{
    getMohrStressCirclePtr()->rebuildMohrCircle( Super::getWorldTransform().getRotation() );
}

void fj::Particle::addContactForce(const btVector3& contactForce)
{
    getMohrStressCirclePtr()->addContactForce(contactForce);
}

void fj::Particle::collapse()
{
    const btVector3 kContactForceSum = std::accumulate(std::begin(getContactForceContainer()), std::end(getContactForceContainer()), btVector3(0, 0, 0)/*初期値*/);
    applyCentralForce(kContactForceSum);
    getRheorogyModelParameterPtr()->DashpodEnvelope = 0.0;
}

void fj::Particle::lockWithFriction()
{
    // 速度を奪うことで摩擦力による拘束を表現する.
    // しかし, これだと接触力がいっさいない場合でも粒子の動きが止まってしまう
    // これは不自然なので, 接触力の有無を考慮する
    
//    clearForces();
    const btVector3 kContactForceSum = std::accumulate(std::begin(getContactForceContainer()), std::end(getContactForceContainer()), btVector3(0, 0, 0)/*初期値*/);
    applyCentralForce(kContactForceSum);
    
//    if ( !getMohrStressCircle().getContactForceContainer().empty() )
//    {
//        setLinearVelocity(btVector3(0, 0, 0));
//    }
    getRheorogyModelParameterPtr()->DashpodEnvelope = 50.0;
}

void fj::Particle::clearContactForce()
{
    getMohrStressCirclePtr()->clearContactForce();
}

btScalar fj::Particle::getRadius()const
{
    return static_cast<const fj::FineParticleShape*>(getCollisionShape())->getRigidRadius();
}

btVector3 fj::Particle::getPosition()const
{
    btTransform trans;
    m_motionState->getWorldTransform(trans);
    return trans.getOrigin();
}
