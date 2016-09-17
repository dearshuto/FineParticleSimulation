//
//  fine_particle_world.hpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/08/25.
//
//

#ifndef fine_particle_world_hpp
#define fine_particle_world_hpp

#include <memory>
#include <vector>
#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>
#include "fine_particle/simulation/particle/particle.hpp"
#include "fine_particle/simulation/bullet_algorithm/fine_particle_simulation_collision_configuration.hpp"

namespace fj {
    class Particle;
    class FineParticleWorld;
}

class fj::FineParticleWorld
{
    using TimeStep = btScalar;
    
    struct FineParticlesContactInfo
    {
        FineParticlesContactInfo(fj::Particle*const particle1, fj::Particle*const particle2)
        : Particle1(particle1)
        , Particle2(particle2)
        , kDirection12(particle2->getPosition() - particle1->getPosition())
        , kDistance(kDirection12.norm())
        , kNormalizedDirection12(kDirection12 / kDistance)
        {
            
        }
        
        fj::Particle*const Particle1;
        fj::Particle*const Particle2;
        const btVector3 kDirection12;
        const btScalar kDistance;
        const btVector3 kNormalizedDirection12;
    };
public:
    FineParticleWorld()
    : SpringK(1)
    , E(10.0)
    , HamakerConstant(0)
    , m_collisionConfiguration( new fj::FineParticleSimulationCollisionConfiguration() )
    , m_dispatcher( new btCollisionDispatcher( m_collisionConfiguration.get() ) )
    , m_pairCache( new btDbvtBroadphase() )
    , m_constraintSolver( new btSequentialImpulseConstraintSolver() )
    , m_world( new btDiscreteDynamicsWorld( m_dispatcher.get()
              , m_pairCache.get()
              , m_constraintSolver.get()
              , m_collisionConfiguration.get())
              )
    
    {
        m_world->getBroadphase()->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());
    }
    
    ~FineParticleWorld() = default;
    
    void stepSimulation(btScalar timestep);
    
    /**
     * この関数を使って登録した剛体は、プログラム側で解放されます
     */
    void addRigidBody(std::unique_ptr<btRigidBody> body
                      , fj::CollisionGroup group = fj::CollisionGroup::kNone
                      , fj::CollisionFiltering mask = fj::CollisionFiltering::kNone
                      );
    
    /**
     * この関数を使って登録した剛体はユーザが責任を持ってメモリを解放してください
     */
    void addCollisionObject(btCollisionObject* body
                            , fj::CollisionGroup group = fj::CollisionGroup::kNone
                            , fj::CollisionFiltering mask = fj::CollisionFiltering::kNone
                            );
    
    void addParticle(std::unique_ptr<fj::Particle> body
                     , fj::CollisionGroup group = fj::CollisionGroup::kNone
                     , fj::CollisionFiltering mask = fj::CollisionFiltering::kNone
                     );
    
    void setGravity(const btVector3& gravity);
    
private:
    
    void accumulateFineParticleForce(const btScalar timestep);
    
    void applyContactForce(const FineParticlesContactInfo& contactInfo);
    
    void applyNormalComponentContactForce(const FineParticlesContactInfo& contactInfo, const btScalar overlap)const;
    
    void applyTangentialComponentContactForce(const FineParticlesContactInfo& contactInfo)const;
    
    /** 換算質量を求める */
    btScalar computeReducedMass(const fj::Particle& particle1, const fj::Particle& particle2)const;
        
    void applyVandeerWaalsForce(const FineParticlesContactInfo& contactInfo)const;
    
    void updateParticleCollapse(const btScalar timestep);
    
    void updateAllObjectTransform(const btScalar timestep);
    
public:
    const std::vector<std::unique_ptr<fj::Particle>>& getParticles()const
    {
        return std::cref(m_particles);
    }
    
    /** レオロジーモデルで使用するばね係数 */
    double SpringK;
    
    /** 粒子間の反発力 */
    double E;
    
    double HamakerConstant;
private:
    std::vector<std::unique_ptr<fj::Particle>> m_particles;
    
    
    //--Bullet Physicsのフレームワークを利用するためのインスタンス--//
    
    /** Bullet Physicsは生ポインタで全ての処理をするので, メモリの管理はユーザ側でしなくてはならない
     * Bullet Physicsの中でシミュレーション対象となる剛体のメモリ管理用のコンテナ */
    std::vector<std::unique_ptr<btRigidBody>> m_rigidBody;
    
    /** Bullet Physicsを利用するために最低限必要なインスタンス */
    std::unique_ptr<btCollisionConfiguration> m_collisionConfiguration;
    std::unique_ptr<btDispatcher> m_dispatcher;
    std::unique_ptr<btBroadphaseInterface> m_pairCache;
    std::unique_ptr<btConstraintSolver> m_constraintSolver;
    std::unique_ptr<btDiscreteDynamicsWorld> m_world;
};

#endif /* fine_particle_world_hpp */
