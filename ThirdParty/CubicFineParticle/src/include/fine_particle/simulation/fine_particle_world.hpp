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

namespace fj {
    class Particle;
    class FineParticleWorld;
}

class fj::FineParticleWorld
{
    typedef btDiscreteDynamicsWorld Super;
public:
    FineParticleWorld()
    : m_collisionConfiguration( new btDefaultCollisionConfiguration() )
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
    
    int stepSimulation(btScalar timestep);
    
    void accumlateParticleForce(btScalar timestep);
    
    /**
     * この関数を使って登録した剛体は、プログラム側で解放されます
     */
    void addRigidBody(std::unique_ptr<btRigidBody> body, short group, short mask);
    
    /**
     * この関数を使って登録した剛体はユーザが責任を持ってメモリを解放してください
     */
    void addCollisionObject(btCollisionObject* body, short group, short mask);
    
    void addParticle(std::unique_ptr<fj::Particle> body, short group, short mask);
    
    void setGravity(const btVector3& gravity);
    
private:
    
    void applyJointForce();
    
    void stepDEM(btScalar timestep);
    
public:
    const std::vector<std::unique_ptr<fj::Particle>>& getParticles()const
    {
        return std::cref(m_particles);
    }
private:
    std::vector<std::unique_ptr<fj::Particle>> m_particles;
    std::vector<std::unique_ptr<btRigidBody>> m_rigidBody;
    
    std::unique_ptr<btCollisionConfiguration> m_collisionConfiguration;
    std::unique_ptr<btDispatcher> m_dispatcher;
    std::unique_ptr<btBroadphaseInterface> m_pairCache;
    std::unique_ptr<btConstraintSolver> m_constraintSolver;
    std::unique_ptr<btDynamicsWorld> m_world;
};

#endif /* fine_particle_world_hpp */
