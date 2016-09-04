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
                          | static_cast<uint16_t>(CollisionGroup::kEffectRange)),
        
        /**
         * 粒子のオーバーラップを検出するだけなので, kRigidParticleだけでOK
         */
        kOverlap = static_cast<uint16_t>(CollisionGroup::kRigidParticle),
        
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
    Particle() = delete;
    ~Particle() = default;
    
    Particle(const btRigidBodyConstructionInfo& info, std::unique_ptr<btMotionState> motionState)
    : btRigidBody(info)
    , m_motionState( std::move(motionState) )
    {
        init();
    }
    
    static std::unique_ptr<fj::Particle> generateParticle(const double x, const double y, const double z);
        
    void setOverlapInWorld( fj::FineParticleWorld* world);
    
    bool isCollapse()const;
    
    /**
     * 毎フレーム更新が必要な処理
     */
    void update(btScalar timestep);
    
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
    
    const fj::Particle& getOverlappingParticle(const int index)const;
    
private:
    void init();
public:
    static std::unique_ptr<btSphereShape> SphereShape;
    
    static std::unique_ptr<btSphereShape> OverlapShape;
    
    static std::unique_ptr<btBoxShape> BoxShape;

private:
    btPairCachingGhostObject m_overlap;
    
    /**
     * 接触してなくても影響が及ぶ粒子を検出するためのダミーオブジェクト
     */
    btGhostObject m_effectRange;
    
    std::unique_ptr<btMotionState> m_motionState;
};

#endif /* particle_hpp */
