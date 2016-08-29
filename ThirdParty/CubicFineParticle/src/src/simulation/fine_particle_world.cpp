//
//  fine_particle_world.cpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/08/25.
//
//

#include <BulletCollision/CollisionDispatch/btSimulationIslandManager.h>
#include "simulation/particle.hpp"
#include "simulation/fine_particle_world.hpp"

int fj::FineParticleWorld::stepSimulation(btScalar timestep)
{
    for (const auto& particle : m_particles)
    {
        particle->update(timestep);
    }
    
    return m_world->stepSimulation(timestep);
}

void fj::FineParticleWorld::applyJointForce()
{
    
}

void fj::FineParticleWorld::stepDEM(btScalar timestep)
{

}

void fj::FineParticleWorld::addCollisionObject(btCollisionObject *body, short group, short mask)
{
    m_world->addCollisionObject(body, group, mask);
}

void fj::FineParticleWorld::addRigidBody(std::unique_ptr<btRigidBody> body, short group, short mask)
{
    m_world->addRigidBody(body.get(), group, mask);
    m_rigidBody.push_back( std::move(body) );
}

void fj::FineParticleWorld::addParticle(std::unique_ptr<fj::Particle> body, short group, short mask)
{
    body->setOverlapInWorld(this);
    m_world->addRigidBody(body.get(), group, mask);
    m_particles.push_back( std::move(body) );
}

void fj::FineParticleWorld::setGravity(const btVector3 &gravity)
{
    m_world->setGravity(gravity);
}