//
//  fine_particle_world.cpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/08/25.
//
//

#include <cmath>
#include <BulletCollision/CollisionDispatch/btSimulationIslandManager.h>
#include "simulation/particle.hpp"
#include "simulation/fine_particle_world.hpp"

void fj::FineParticleWorld::accumlateParticleForce(btScalar timestep)
{
    for (const auto& particle : m_particles)
    {
        particle->update(timestep);
        
        for (int i = 0; i < particle->overlappingSize(); i++)
        {
            const fj::Particle& kOverlap = particle->getOverlappingParticle(i);
            btTransform targetPosition, neighborPosition;
            particle->getMotionState()->getWorldTransform(targetPosition);
            kOverlap.getMotionState()->getWorldTransform(neighborPosition);
            
            const btVector3 kComeUpVector = targetPosition.getOrigin() - neighborPosition.getOrigin();
            const btScalar kNorm = kComeUpVector.norm();
            const btScalar kDistance = (particle->getRadius() + kOverlap.getRadius()) - kNorm;
            
            if ( !std::isfinite(kDistance) || (kNorm == btScalar(0.0)) )
            {
                continue;
            }
            
            if (kDistance > 0)
                particle->applyCentralImpulse(0.1 * kDistance * kComeUpVector.normalized());
        }
    }
    
}

int fj::FineParticleWorld::stepSimulation(btScalar timestep)
{
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