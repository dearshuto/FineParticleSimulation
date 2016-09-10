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
    // TODO: 最適化できるよ
    
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
            
            const btVector3 kDirection12 = kTransform2.getOrigin() - kTransform1.getOrigin();
            const btVector3 kDirection21 = -kDirection12;
            const btScalar kNorm = kDirection12.norm();
            const btScalar kDistance = (particle1->getRadius() + particle1->getRadius()) - kNorm;
            
            if ( !std::isfinite(kDistance) || (kNorm == btScalar(0.0)) )
            {
                continue;
            }
            
            if (kDistance > 0)
            {
                // ばね
                particle1->Parent->addContactForce(kSpringK * kDistance * kDirection21.normalized() );
                particle2->Parent->addContactForce(kSpringK * kDistance * kDirection12.normalized() );

            }
            
            const btVector3 kRelativeVelocity12 = particle2->Parent->getLinearVelocity() - particle1->Parent->getLinearVelocity();
            const btVector3 kRelativeVelocity21 = -kRelativeVelocity12;
            
            if (kRelativeVelocity12.isZero())
            {
                continue;
            }
            else
            {
                // ダッシュポッド
                particle1->Parent->addContactForce(-kSpringK / 2.0 * kRelativeVelocity21.normalized());
                particle1->Parent->addContactForce(-kSpringK / 2.0 * kRelativeVelocity12.normalized());
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
        btTransform transform;
        particle->getMotionState()->getWorldTransform(transform);
        const btVector3& kPosition = transform.getOrigin();
        
        for (int i = 0; i < particle->overlappingSize(); i++)
        {
            const fj::Particle* kOverlapParticle = fj::Particle::upcast( particle->getEffectObject(i) );
            
            if (kOverlapParticle)
            {
                const btVector3& kOverlapPosition = kOverlapParticle->getWorldTransform().getOrigin();
                
                // 近傍粒子から受けるファンデルワールス力の方向
                const btVector3& kDirection = kPosition - kOverlapPosition;
                
                // 粒子の中心間距離
                const btScalar kDistance = (kPosition - kOverlapPosition).norm();
                
                // 粒子の表面間距離
                const btScalar kH = std::max(btScalar(0.000004), kDistance - (particle->getRadius() + kOverlapParticle->getRadius()));
                
                // 換算粒径
                const btScalar kD = (particle->getRadius() * kOverlapParticle->getRadius()) / (particle->getRadius() + kOverlapParticle->getRadius());
                
                const btVector3 kF = (HamakerConstant * kD / (24 * std::pow(kH, 2.0)) ) * kDirection;
                
                particle->applyCentralForce(kF);
            }
            else
            {
                continue;
            }
            
        }
    }
    
}

void fj::FineParticleWorld::BulletWorldWrapper::synchronizeMotionStates()
{
    // 位置更新する段階ではすべての力の計算は終わっている
    // 粒子にかかっている力をもとに崩壊条件を判定する
    
    for (auto& particle : kParentWorld.m_particles)
    {
        // 崩壊条件を満たしてない場合, 力をなくしてしまえば動かない
        if ( particle->isCollapse() )
        {
            particle->applyContactForce();
        }
        else
        {
            particle->clearForces();
        }
        particle->clearContactForce();
    }

    btDiscreteDynamicsWorld::synchronizeMotionStates();
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