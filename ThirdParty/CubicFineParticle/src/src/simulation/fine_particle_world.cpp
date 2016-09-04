//
//  fine_particle_world.cpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/08/25.
//
//

#include <cmath>
#include <iostream>
#include <BulletCollision/CollisionDispatch/btSimulationIslandManager.h>
#include "fine_particle/simulation/particle.hpp"
#include "fine_particle/simulation/fine_particle_world.hpp"

int fj::FineParticleWorld::stepSimulation(btScalar timestep)
{
    updateParticleCollisionShapePosition(timestep);
    
    accumulateCollisionForce(timestep);
    
    accumulateVandeerWaalsForce(timestep);
    
    return m_world->stepSimulation(timestep);
}


void fj::FineParticleWorld::updateParticleCollisionShapePosition(const btScalar timestep)
{
    for (auto& particle : m_particles)
    {
        particle->updateCollisionShapePosition(timestep);
    }
}

void fj::FineParticleWorld::accumulateCollisionForce(const btScalar timestep)
{
    // 粒子同士が衝突することによって発生する力を計算する
    
    typedef fj::Particle::ParticlesOverlapDetector ParticlesOverlapDetector;
    
    for (int i = 0; i < m_dispatcher->getNumManifolds(); i++)
    {
        const btPersistentManifold* manifold = m_dispatcher->getManifoldByIndexInternal(i);
        
        const ParticlesOverlapDetector* particle1 = ParticlesOverlapDetector::upcast(manifold->getBody0());
        const ParticlesOverlapDetector* particle2 = ParticlesOverlapDetector::upcast(manifold->getBody1());
        
        if (particle1 && particle2)
        {
            const btTransform& kTransform1 = particle1->getWorldTransform();
            const btTransform& kTransform2 = particle2->getWorldTransform();
            
            const btVector3 kComeUpVector = kTransform1.getOrigin() - kTransform2.getOrigin();
            const btScalar kNorm = kComeUpVector.norm();
            const btScalar kDistance = /*(particle->getRadius() + kOverlap.getRadius())*/0.5*2.0 - kNorm;
            
            if ( !std::isfinite(kDistance) || (kNorm == btScalar(0.0)) )
            {
                continue;
            }
            
            if (kDistance > 0)
            {
                particle1->Parent->applyCentralImpulse(kSpringK * kDistance * kComeUpVector.normalized());
                particle2->Parent->applyCentralImpulse(kSpringK * kDistance * kComeUpVector.normalized());
            }
            
        }
        else
        {
            
        }
    }
    
}

void fj::FineParticleWorld::accumulateVandeerWaalsForce(btScalar timestep)
{
    for (const auto& particle : m_particles)
    {
        for (int i = 0; i < particle->overlappingSize(); i++)
        {
            const fj::Particle* kOverlapParticle = fj::Particle::upcast( particle->getEffectObject(i) );

            if (kOverlapParticle)
            {
                // coming soon...
            }
            else
            {
                continue;
            }
            
        }
    }
    
}

void fj::FineParticleWorld::applyJointForce()
{
    
}

void fj::FineParticleWorld::stepDEM(btScalar timestep)
{

}

void fj::FineParticleWorld::addCollisionObject(btCollisionObject *body, fj::CollisionGroup group, fj::CollisionFiltering mask)
{
    m_world->addCollisionObject(body, static_cast<uint16_t>(group), static_cast<uint16_t>(mask) );
}

void fj::FineParticleWorld::addRigidBody(std::unique_ptr<btRigidBody> body, fj::CollisionGroup group, fj::CollisionFiltering mask)
{
    m_world->addRigidBody(body.get(), static_cast<uint16_t>(group), static_cast<uint16_t>(mask) );
    m_rigidBody.push_back( std::move(body) );
}

void fj::FineParticleWorld::addParticle(std::unique_ptr<fj::Particle> body, fj::CollisionGroup group, fj::CollisionFiltering mask)
{
    body->setOverlapInWorld(this);
    m_world->addRigidBody(body.get(), static_cast<uint16_t>(group), static_cast<uint16_t>(mask) );
    m_particles.push_back( std::move(body) );
}

void fj::FineParticleWorld::setGravity(const btVector3 &gravity)
{
    m_world->setGravity(gravity);
}