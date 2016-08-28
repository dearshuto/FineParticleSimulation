//
//  particle.hpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/08/25.
//
//

#ifndef particle_hpp
#define particle_hpp

#include <memory>
#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>


namespace fj {
    
    enum class CollisionFiltering : uint8_t
    {
        kRigid = 1 << 0,
        kOverlap = 1 << 1,
        kParticle = 1 << 2,
    };
    
    class Particle;
}

class fj::Particle : public btRigidBody
{
public:
    Particle() = delete;
    ~Particle() = default;
    
    Particle(const btRigidBodyConstructionInfo& info, std::unique_ptr<btMotionState> motionState)
    : btRigidBody(info)
    , m_motionState( std::move(motionState) )
    {
        init();
    }
    
    static std::unique_ptr<fj::Particle> generateParticle(const double x, const double y, const double z);
    
    bool isCollapse()const;
    
    /**
     * btCollisionObjectをfj::Particleにアップキャストする.
     * キャストに失敗するとnullptrを返す.
     */
    static fj::Particle* upcast(btCollisionObject* colObj)
    {
        if (colObj->getInternalType()&btCollisionObject::CO_USER_TYPE)
            return (fj::Particle*)colObj;
        return nullptr;
    }

    static const fj::Particle* upcast(const btCollisionObject* colObj)
    {
        if (colObj->getInternalType()&btCollisionObject::CO_USER_TYPE)
            return (const fj::Particle*)colObj;
        return nullptr;
    }

private:
    void init();
public:
    static std::unique_ptr<btSphereShape> SphereShape;
    
    static std::unique_ptr<btSphereShape> OverlapShape;
    
    static std::unique_ptr<btBoxShape> BoxShape;

    btGhostObject Overlap;
private:

    
    std::unique_ptr<btMotionState> m_motionState;
};

#endif /* particle_hpp */
