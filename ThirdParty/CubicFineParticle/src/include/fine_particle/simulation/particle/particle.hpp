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
#include "fine_particle/simulation/mohr_stress_circle.hpp"
#include "discritized_particle_shape.hpp"


namespace fj {
    
    enum class CollisionGroup : uint16_t
    {
        kNone = 1,
        
        /**
         * 普通の物体はこれ
         */
        kRigid = 2,
        
        /**
         * 粒子の剛体部分. kRigidと衝突する.
         */
        kRigidParticle = 4,
    };
    
    /**
     * fj::CollisionFilteringをもとにした衝突の組合せ
     */
    enum class CollisionFiltering : uint16_t
    {
        kNone = 1,
        
        /** 
         * 剛体は剛体同士の衝突と粒子との衝突が起きる. さらに剛体表面からのファンデルワールス力を検知するためにEffectRangeも検出する
         */
        kRigid = (static_cast<uint16_t>(CollisionGroup::kRigid)
                  | static_cast<uint16_t>(CollisionGroup::kRigidParticle)),
        
        /**
         * 粒子間に働く力は独自計算をするので, 粒子間の衝突だけは検知させない
         */
        kRigidParticle = (static_cast<uint16_t>(CollisionGroup::kRigid)),
    };
    
    class FineParticleShape;
    class Particle;
}

class fj::Particle : public btRigidBody
{
public:
    typedef btRigidBody Super;
    typedef std::vector<btVector3> ContactForceContainer;
    class CollapseDetector;
public:
    Particle() = delete;
    ~Particle() = default;
    
    Particle(const fj::DiscritizedParticleShape::ShapeType shapeType, const btRigidBodyConstructionInfo& info, std::unique_ptr<btMotionState> motionState)
    : btRigidBody(info)
    , m_motionState( std::move(motionState) )
    , m_discretizedShapeType(shapeType)
    , m_mass(info.m_mass)
    {
        init();
    }
    
    static std::unique_ptr<fj::Particle> generateParticle(const fj::DiscritizedParticleShape::ShapeType type, const btVector3& position);
        
    bool isCollapse()const;
    
    void addContactForce(const btVector3& constctForce);
    
    void applyContactForce();
    
    void clearContactForce();
    
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

    btScalar getRadius()const;
    
    fj::DiscritizedParticleShape::ShapeType getDiscretizedShapeType()const
    {
        return m_discretizedShapeType;
    }
    
    const ContactForceContainer& getContactForceContainer()const
    {
        return m_contactForceContainer;
    }
    
    const fj::WarrenSpringParameter& getWarrenSpringParameter()const
    {
        return m_warrenSpringParameter;
    }
    
    void setCollapseDetector(const std::weak_ptr<CollapseDetector>& collapseDetector)
    {
        m_collapseDetector = collapseDetector;
    }
    
    btScalar getMass()const
    {
        return m_mass;
    }
    
    btVector3 getPosition()const;
private:
    void init();
public:
    static fj::FineParticleShape CollisionShape;
private:
    
    std::unique_ptr<btMotionState> m_motionState;
    
    fj::DiscritizedParticleShape::ShapeType m_discretizedShapeType;
    
    ContactForceContainer m_contactForceContainer;
    
    fj::WarrenSpringParameter m_warrenSpringParameter;
    
    std::weak_ptr<CollapseDetector> m_collapseDetector;
    
    btScalar m_mass;
};

#endif /* particle_hpp */
