//
//  fine_particle_world.cpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/08/25.
//
//

#include <cassert>
#include <cmath>
#include <iostream>
#include <BulletCollision/CollisionDispatch/btSimulationIslandManager.h>
#include "fine_particle/simulation/particle/particle.hpp"
#include "fine_particle/simulation/fine_particle_world.hpp"

void fj::FineParticleWorld::terminate()
{
    terminateProfiles();
}

void fj::FineParticleWorld::stepSimulation(btScalar timestep)
{
    startProfiling();
    
    accumulateFineParticleForce(timestep);

    updateParticleCollapse(timestep);

    updateAllObjectTransform(timestep);
    
    endProfiling();
}

void fj::FineParticleWorld::accumulateFineParticleForce(const btScalar timestep)
{
    btBroadphasePairArray*const broadPhasePairArray = &m_pairCache->getOverlappingPairCache()->getOverlappingPairArray();
    
    for (unsigned int i = 0; i < broadPhasePairArray->size(); i++)
    {
        auto& broadPhasePair = broadPhasePairArray->at(i);
        
        btCollisionObject* body0 = static_cast<btCollisionObject*>(broadPhasePair.m_pProxy0->m_clientObject);
        btCollisionObject* body1 = static_cast<btCollisionObject*>(broadPhasePair.m_pProxy1->m_clientObject);
        
        fj::Particle*const particle1 = fj::Particle::upcast(body0);
        fj::Particle*const particle2 = fj::Particle::upcast(body1);
        
        if (particle1 && particle2)
        {
            FineParticlesContactInfo contactInfo{particle1, particle2};
            
            applyContactForce(contactInfo);
            applyVandeerWaalsForce(contactInfo);
        }
    }

}

void fj::FineParticleWorld::applyContactForce(const FineParticlesContactInfo& contactInfo)
{
    const auto particle1 = contactInfo.Particle1;
    const auto particle2 = contactInfo.Particle2;
    const auto kRadiusSum = particle1->getRadius() + particle2->getRadius();
    const bool kIsOverlap = contactInfo.kDistance < kRadiusSum;
    
    if ( kIsOverlap )
    {
        const auto kOverlap = kRadiusSum - contactInfo.kDistance;

        applyNormalComponentContactForce(contactInfo, kOverlap);
        applyTangentialComponentContactForce(contactInfo);
    }
}

void fj::FineParticleWorld::applyNormalComponentContactForce(const FineParticlesContactInfo& contactInfo,const btScalar overlap)const
{
    constexpr double kPI = 3.141592653589793238462643383279502884;
    fj::Particle*const particle1 = contactInfo.Particle1;
    fj::Particle*const particle2 = contactInfo.Particle2;
    
    const btVector3 kDirection12 = contactInfo.kNormalizedDirection12;
    const btVector3 kDirection21 = -kDirection12;
    const auto kDistance = contactInfo.kDistance;
    
    // ばね
    particle1->addContactForce(SpringK * kDistance * kDirection21 );
    particle2->addContactForce(SpringK * kDistance * kDirection12 );
    
    // ダッシュポッド
    const btVector3 kRelativeVelocity12 = particle2->getLinearVelocity() - particle1->getLinearVelocity();
    const btVector3 kRelativeVelocity21 = -kRelativeVelocity12;
    const auto kReducedMass = computeReducedMass(std::cref(*particle1), std::cref(*particle2));
    
    const auto kEta = -2.0 * DashpodEnvelop * std::log(E) * std::sqrt(
                                                   (kReducedMass * SpringK)
                                                     / (std::pow(kPI, 2.0) * std::pow(std::log(E), 2.0))
                                                   );
    
    particle1->addContactForce(kEta * kRelativeVelocity21);
    particle2->addContactForce(kEta * kRelativeVelocity12);

}

void fj::FineParticleWorld::applyTangentialComponentContactForce(const FineParticlesContactInfo& contactInfo)const
{

}

btScalar fj::FineParticleWorld::computeReducedMass(const fj::Particle &particle1, const fj::Particle &particle2)const
{
    const auto kMass1 = particle1.getMass();
    const auto kMass2 = particle2.getMass();
    
    return (kMass1 * kMass2) / (kMass1 + kMass2);
}

void fj::FineParticleWorld::applyVandeerWaalsForce(const FineParticlesContactInfo& contactInfo)const
{
    fj::Particle*const particle1 = contactInfo.Particle1;
    fj::Particle*const particle2 = contactInfo.Particle2;
    const btScalar kRadius1 = particle1->getRadius();
    const btScalar kRadius2 = particle2->getRadius();
    
    // 粒子の中心間距離
    const btScalar kDistance = contactInfo.kDistance;

    // 近傍粒子から受けるファンデルワールス力の方向
    const btVector3 kNormalizedDirection21 = -contactInfo.kNormalizedDirection12;
    const btVector3 kNormalizedDirection12 = contactInfo.kNormalizedDirection12;
    
    // 粒子の表面間距離
    const btScalar kH = std::max(
                                 btScalar(0.000004) //*発散防止のクランプ*/
                                 , kDistance - (kRadius1 + kRadius2)
                                 );
    
    // 換算粒径
    const btScalar kD = (kRadius1 * kRadius2) / (kRadius1 + kRadius2);
    
    const btScalar kF = HamakerConstant * kD / (24 * std::pow(kH, 2.0));
    
    particle1->applyCentralForce(kF * kNormalizedDirection21);
    particle2->applyCentralForce(kF * kNormalizedDirection12);
}

void fj::FineParticleWorld::updateParticleCollapse(const btScalar timestep)
{
    for (auto& particle : m_particles)
    {
        if ( shouldCollapse(*particle) )
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

bool fj::FineParticleWorld::shouldCollapse(const fj::Particle &particle)const
{
    // とりあえず常に崩壊
    return true;
    
    const auto& kMohrStressCircle = particle.getMohrStressCircle();
    const auto& kWarrennSpringParameter = particle.getWarrenSpringParameter();
    
    // Use any algorithm...
}

void fj::FineParticleWorld::updateAllObjectTransform(const btScalar timestep)
{
    m_world->stepSimulation(timestep, 1/*max substeps*/, timestep);
}

void fj::FineParticleWorld::startProfiling()
{
    for (auto& profile : m_profiles)
    {
        profile->startSimulationProfile();
    }
}

void fj::FineParticleWorld::endProfiling()
{
    for (auto& profile : m_profiles)
    {
        profile->endSimulationProfile();
    }
}

void fj::FineParticleWorld::terminateProfiles()
{
    for (auto& profile : m_profiles)
    {
        profile->terminate();
    }
}

void fj::FineParticleWorld::addCollisionObject(btCollisionObject *body)
{
    m_world->addCollisionObject(body);
}

void fj::FineParticleWorld::addRigidBody(std::unique_ptr<btRigidBody> body)
{
    assert( !fj::Particle::upcast(body.get()) && "Use addParticle function to register a instance of fj::Particle.");
    
    m_world->addRigidBody(body.get());
    m_rigidBody.push_back( std::move(body) );
}

void fj::FineParticleWorld::addParticle(std::unique_ptr<fj::Particle> body)
{
    m_world->addRigidBody(body.get() );
    m_particles.push_back( std::move(body) );
}

void fj::FineParticleWorld::removeParticle(fj::Particle *const particle)
{
    // Bullet Physicsからの削除
    m_world->removeCollisionObject(particle);
    
    // m_particlesの中に同一のアドレスをもつインスタンスが存在しないことを前提とする
    auto inContainer = std::find_if(m_particles.begin(), m_particles.end()
                                    , [particle](std::unique_ptr<fj::Particle>& containerComponent){
                                        return containerComponent.get() == particle;
                                    });
    assert(inContainer != m_particles.end());
    m_particles.erase(inContainer);
}

void fj::FineParticleWorld::setGravity(const btVector3 &gravity)
{
    m_world->setGravity(gravity);
}

void fj::FineParticleWorld::addProfileSystem(std::unique_ptr<fj::SimulationProfile> profile)
{
    m_profiles.push_back(std::move(profile));
}
