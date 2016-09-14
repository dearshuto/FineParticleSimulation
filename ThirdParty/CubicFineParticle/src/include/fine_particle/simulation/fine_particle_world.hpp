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

namespace fj {
    class Particle;
    class FineParticleWorld;
}

class fj::FineParticleWorld
{
    class BulletWorldWrapper : public btDiscreteDynamicsWorld
    {
        typedef btDiscreteDynamicsWorld Super;
    public:
        BulletWorldWrapper(btDispatcher* dispatcher
                           ,btBroadphaseInterface* pairCache
                           ,btConstraintSolver* constraintSolver
                           ,btCollisionConfiguration* collisionConfiguration
                           , const FineParticleWorld& parent)
        : btDiscreteDynamicsWorld(dispatcher, pairCache, constraintSolver, collisionConfiguration)
        , kParentWorld(parent)
        {
            
        }

        void	synchronizeMotionStates() override;
        
        void	internalSingleStepSimulation( btScalar timeStep)override;
        
        void	applyGravity()override;
    private:
        const FineParticleWorld& kParentWorld;
    };
public:
    FineParticleWorld()
    : kSpringK(1)
    , HamakerConstant(0)
    , m_collisionConfiguration( new btDefaultCollisionConfiguration() )
    , m_dispatcher( new btCollisionDispatcher( m_collisionConfiguration.get() ) )
    , m_pairCache( new btDbvtBroadphase() )
    , m_constraintSolver( new btSequentialImpulseConstraintSolver() )
    , m_world( new BulletWorldWrapper( m_dispatcher.get()
              , m_pairCache.get()
              , m_constraintSolver.get()
              , m_collisionConfiguration.get()
              ,*this)
              )
    
    {
        m_world->getBroadphase()->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());
    }
    
    ~FineParticleWorld() = default;
    
    int stepSimulation(btScalar timestep);
    
    /**
     * この関数を使って登録した剛体は、プログラム側で解放されます
     */
    void addRigidBody(std::unique_ptr<btRigidBody> body, fj::CollisionGroup group, fj::CollisionFiltering mask);
    
    /**
     * この関数を使って登録した剛体はユーザが責任を持ってメモリを解放してください
     */
    void addCollisionObject(btCollisionObject* body, fj::CollisionGroup group, fj::CollisionFiltering mask);
    
    void addParticle(std::unique_ptr<fj::Particle> body, fj::CollisionGroup group, fj::CollisionFiltering mask);
    
    void setGravity(const btVector3& gravity);
    
private:
    
    void updateParticleCollisionShapePosition(const btScalar timestep);
    
    void accumulateCollisionForce(const btScalar timestep);
    
    void accumulateVandeerWaalsForce(const btScalar timestep);
    
public:
    const std::vector<std::unique_ptr<fj::Particle>>& getParticles()const
    {
        return std::cref(m_particles);
    }
    
    double kSpringK;
    
    double HamakerConstant;
private:
    std::vector<std::unique_ptr<fj::Particle>> m_particles;
    std::vector<std::unique_ptr<btRigidBody>> m_rigidBody;
    
    std::unique_ptr<btCollisionConfiguration> m_collisionConfiguration;
    std::unique_ptr<btDispatcher> m_dispatcher;
    std::unique_ptr<btBroadphaseInterface> m_pairCache;
    std::unique_ptr<btConstraintSolver> m_constraintSolver;
    std::unique_ptr<BulletWorldWrapper> m_world;
};

#endif /* fine_particle_world_hpp */
