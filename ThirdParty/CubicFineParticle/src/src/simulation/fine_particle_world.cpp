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
#include "fine_particle/simulation/particle/particle.hpp"
#include "fine_particle/simulation/fine_particle_world.hpp"

void fj::FineParticleWorld::stepSimulation(btScalar timestep)
{
    updateParticleCollisionShapePosition(timestep);
    
    accumulateContactForce(timestep);
    
//    accumulateVandeerWaalsForce(timestep);
    
    updateParticleCollapse(timestep);

    updateAllObjectTransform(timestep);
}


void fj::FineParticleWorld::updateParticleCollisionShapePosition(const btScalar timestep)
{
    for (auto& particle : m_particles)
    {
        particle->updateCollisionShapePosition(timestep);
    }
}

void fj::FineParticleWorld::accumulateContactForce(const btScalar timestep)
{
    // Bullet Physicsは衝突を検出した全てのペアを保持している
    // これらのペアの中からParticleのオーバーラップを検出するオブジェクト同士の衝突に対して処理を施す

    typedef fj::Particle::ParticlesOverlapDetector ParticlesOverlapDetector;
    
    for (int i = 0; i < m_dispatcher->getNumManifolds(); i++)
    {
        const btPersistentManifold* manifold = m_dispatcher->getManifoldByIndexInternal(i);
        const ParticlesOverlapDetector* kParticleOverlap1 = ParticlesOverlapDetector::upcast(manifold->getBody0());
        const ParticlesOverlapDetector* kParticleOverlap2 = ParticlesOverlapDetector::upcast(manifold->getBody1());

        if (kParticleOverlap1 && kParticleOverlap2)
        {
            fj::Particle* particle1 = kParticleOverlap1->Parent;
            fj::Particle* particle2 = kParticleOverlap2->Parent;
            
            applyNormalComponentContactForce(particle1, particle2);
            applyTangentialComponentContactForce(particle1, particle2);
        }
    }


}

void fj::FineParticleWorld::applyNormalComponentContactForce(fj::Particle*const particle1, fj::Particle*const particle2)const
{
    constexpr double kPI = 3.141592653589793238462643383279502884;
    const btTransform& kTransform1 = particle1->getWorldTransform();
    const btTransform& kTransform2 = particle2->getWorldTransform();
    
    const btVector3 kDirection12 = kTransform2.getOrigin() - kTransform1.getOrigin();
    const btVector3 kDirection21 = -kDirection12;
    const btScalar kNorm = kDirection12.norm();
    const btScalar kDistance = (particle1->getRadius() + particle1->getRadius()) - kNorm;
    
    if ( !std::isfinite(kDistance) || (kNorm == btScalar(0.0)) )
    {
        return;
    }
    
    if (kDistance > 0)
    {
        // ばね
        particle1->addContactForce(SpringK * kDistance * kDirection21.normalized() );
        particle2->addContactForce(SpringK * kDistance * kDirection12.normalized() );
        
    }

    
    // ダッシュポッド
    const btVector3 kRelativeVelocity12 = particle2->getLinearVelocity() - particle1->getLinearVelocity();
    const btVector3 kRelativeVelocity21 = -kRelativeVelocity12;
    const auto kReducedMass = computeReducedMass(std::cref(*particle1), std::cref(*particle2));
    
    const auto kEta = -2.0 * std::log(E) * std::sqrt(
                                                   (kReducedMass * SpringK)
                                                     / (std::pow(kPI, 2.0) * std::pow(std::log(E), 2.0))
                                                   );
    
    particle1->addContactForce(-kEta * kRelativeVelocity21);
    particle2->addContactForce(-kEta * kRelativeVelocity12);

}

void fj::FineParticleWorld::applyTangentialComponentContactForce(fj::Particle*const particle1, fj::Particle*const particle2)const
{

}

btScalar fj::FineParticleWorld::computeReducedMass(const fj::Particle &particle1, const fj::Particle &particle2)const
{
    const auto kMass1 = particle1.getMass();
    const auto kMass2 = particle2.getMass();
    
    return (kMass1 * kMass2) / (kMass1 + kMass2);
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

void fj::FineParticleWorld::updateParticleCollapse(const btScalar timestep)
{
    for (auto& particle : m_particles)
    {
        if ( /*particle->isCollapse()*/true )
        {
            particle->applyContactForce();
        }
        else
        {
            // 崩壊条件を満たしてない場合, 速度を奪ってしまえば位置更新されない
            particle->clearForces();
            particle->setLinearVelocity(btVector3(0, 0, 0));
        }
        particle->clearContactForce();
    }
}

void fj::FineParticleWorld::updateAllObjectTransform(const btScalar timestep)
{
    m_world->stepSimulation(timestep, 1/*max substeps*/, timestep);
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