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
#include "discritized_particle_shape.hpp"
#include "mohr_stress_circle.hpp"

namespace fj {
    
    enum class CollisionGroup : uint16_t
    {
        /**
         * 普通の物体はこれ
         */
        kRigid = 1,
        
        /**
         * 粒子と粒子のオーバーラップを検出するための物体
         */
        kOverlap = 2,
        
        /**
         * 粒子の剛体部分. kRigidと衝突する.
         */
        kRigidParticle = 4,
        
        /**
         * ファンデルワールス力が働く範囲を決定するダミー物体
         */
        kEffectRange = 8,
    };
    
    /**
     * fj::CollisionFilteringをもとにした衝突の組合せ
     */
    enum class CollisionFiltering : uint16_t
    {
        /** 
         * 剛体は剛体同士の衝突と粒子との衝突が起きる. さらに剛体表面からのファンデルワールス力を検知するためにEffectRangeも検出する
         */
        kRigid = (static_cast<uint16_t>(CollisionGroup::kRigid)
                  | static_cast<uint16_t>(CollisionGroup::kRigidParticle)
                  | static_cast<uint16_t>(CollisionGroup::kEffectRange)),
        
        /**
         * 粒子間に働く力は独自計算をするので, 粒子間の衝突だけは検知させない
         */
        kRigidParticle = (static_cast<uint16_t>(CollisionGroup::kRigid)
                          | static_cast<uint16_t>(CollisionGroup::kOverlap)
                          | static_cast<uint16_t>(CollisionGroup::kEffectRange)
//                          | static_cast<uint16_t>(CollisionGroup::kRigidParticle)
                          ),
        
        /**
         * 粒子のオーバーラップを検出するだけなので, kOverlapだけでOK
         */
        kOverlap = static_cast<uint16_t>(CollisionGroup::kOverlap),
        
        /**
         * ファンデルワールス力が働く物体を検知する
         */
        kEffectRange = (static_cast<uint16_t>(CollisionGroup::kRigid)
                        | static_cast<uint16_t>(CollisionGroup::kRigidParticle)),
        
    };
    
    class FineParticleWorld;
    class Particle;
}

class fj::Particle : public btRigidBody
{
public:
    typedef btRigidBody Super;
    typedef std::vector<btVector3> ContactForceContainer;
    
    class ParticlesOverlapDetector : public btGhostObject
    {
        static constexpr int kPartitcleOverlapDetectorCollisionType = 128;
    public:
        ParticlesOverlapDetector() = delete;
        
        ParticlesOverlapDetector(fj::Particle*const parent)
        : Parent(parent)
        {
            // btCollisionObject::CollisionObjectTypesが64までしかないので128を勝手に使っちゃう
            m_internalType = btCollisionObject::CO_GHOST_OBJECT | kPartitcleOverlapDetectorCollisionType;
        }
        ~ParticlesOverlapDetector() = default;
        
        static fj::Particle::ParticlesOverlapDetector* upcast(btCollisionObject* colObj)
        {
            if (colObj->getInternalType() & kPartitcleOverlapDetectorCollisionType)
                return (fj::Particle::ParticlesOverlapDetector*)colObj;
            return nullptr;
        }
        
        static const fj::Particle::ParticlesOverlapDetector* upcast(const btCollisionObject* colObj)
        {
            if (colObj->getInternalType() & kPartitcleOverlapDetectorCollisionType)
                return (const fj::Particle::ParticlesOverlapDetector*)colObj;
            return nullptr;
        }

        btScalar getRadius()const
        {
            return static_cast<const btSphereShape*>(getCollisionShape())->getRadius();
        }
        
        fj::Particle*const Parent;
    };
    
public:
    Particle() = delete;
    ~Particle() = default;
    
    Particle(const fj::DiscritizedParticleShape::ShapeType shapeType, const btRigidBodyConstructionInfo& info, std::unique_ptr<btMotionState> motionState)
    : btRigidBody(info)
    , m_overlap( this )
    , m_motionState( std::move(motionState) )
    , m_discretizedShapeType(shapeType)
    {
        init();
    }
    
    static std::unique_ptr<fj::Particle> generateParticle(const fj::DiscritizedParticleShape::ShapeType type, const btVector3& position);
        
    void setOverlapInWorld( fj::FineParticleWorld* world);
    
    bool isCollapse()const;
    
    void addContactForce(const btVector3& constctForce);
    
    void applyContactForce();
    
    void clearContactForce();
    
    /**
     * 毎フレーム更新が必要な処理
     */
    void updateCollisionShapePosition(btScalar timestep);
    
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

    btScalar getRadius()const
    {
        return static_cast<const btSphereShape*>(getCollisionShape())->getRadius();
    }
    
    int overlappingSize()const;
    
    const btCollisionObject* getEffectObject(const int index)const;
    
    fj::DiscritizedParticleShape::ShapeType getDiscretizedShapeType()const
    {
        return m_discretizedShapeType;
    }
    
private:
    void init();
public:
    static std::unique_ptr<btSphereShape> SphereShape;
    
    static std::unique_ptr<btSphereShape> OverlapShape;
    
    static std::unique_ptr<btBoxShape> BoxShape;

private:
    ParticlesOverlapDetector m_overlap;
    
    /**
     * 接触してなくても影響が及ぶ粒子を検出するためのダミーオブジェクト
     */
    btGhostObject m_effectRange;
    
    std::unique_ptr<btMotionState> m_motionState;
    
    fj::DiscritizedParticleShape::ShapeType m_discretizedShapeType;
    
    ContactForceContainer m_contactForceContainer;
    
    fj::WarrenSpringParameter m_warrenSpringParameter;
};

#endif /* particle_hpp */
