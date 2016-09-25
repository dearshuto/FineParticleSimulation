//
//  fine_particle_world.hpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/08/25.
//
//

/**
 * @mainpage 粉体シミュレーション
 * 粉の挙動を再現するためのプログラムです.
 *
 * @section main_sec 粉体を再現するために必要なこと
 * 1. 力学計算
 * 2. 衝突判定
 *
 * これらを実現するためにBullet Physicsを導入します.
 * Bullet Physics の特徴
 * 1. 力学計算に必要なインタフェースが一通り揃っている
 * 2. 衝突判定が高速
 * 3. 拡張しやすい
 *
 * @section main_sec 力学計算
 * 粉体のシミュレーションは独自のアルゴリズムを導入する.
 * そのためには, 以下の3つの衝突を区別して検出しなくてはならない.
 * 1. 粒子同士のオーバーラップ
 * 2. 近傍粒子の検出
 * 3. 粒子と他の物体との衝突
 *
 * @subsection main_subsec Bullet Physicsの衝突判定アルゴリズム
 * Bullet PhysicsではAABB木を利用した衝突判定を行っている.
 * ブロードフェーズでは衝突形状のAABBで判定を行い, ナローフェーズで実際の形状を利用した衝突判定を行う.
 *
 * @par 粉体シミュレーションへの拡張
 * 粉体粒子同士のブロードフェーズにおける衝突はBullet Physicsに任せ, ナローフェーズを独自に実装するという手段をとる.
 * AABBは通常, 物体に接するように定義されるが, あえて大きめのAABBにすることで近傍粒子との衝突も同時に検出する. ←これ工夫ポイント.
 * すなわち, Bullet Physicsを以下のようにカスタマイズする.
 * 1. ナローフェーズにおける粒子同士の衝突は何もしない
 * 2. 粒子のAABBを大きくする
 */

#ifndef fine_particle_world_hpp
#define fine_particle_world_hpp

#include <memory>
#include <vector>
#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>
#include "fine_particle/simulation/particle/particle.hpp"
#include "fine_particle/simulation/profile/simulation_profile.hpp"
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
    , DashpodEnvelop(1.0)
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

    void terminate();
    
    void stepSimulation(btScalar timestep);
    
    /**
     * この関数を使って登録した剛体は、プログラム側で解放されます
     */
    void addRigidBody(std::unique_ptr<btRigidBody> body);

    /**
     * この関数を使って登録した剛体はユーザが責任を持ってメモリを解放してください
     */
    void addCollisionObject(btCollisionObject* body);

    void addParticle(std::unique_ptr<fj::Particle> body);

    void removeParticle(fj::Particle*const particle);
    
    void setGravity(const btVector3& gravity);

    
    //------------------ Profiling -----------------------------//
    void addProfileSystem(std::unique_ptr<fj::SimulationProfile> profile);
    
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

    //----------------- Profiling ------------------------------//
    void startProfiling();
    
    void endProfiling();
    
    void terminateProfiles();
    
public:
    const std::vector<std::unique_ptr<fj::Particle>>& getParticles()const
    {
        return std::cref(m_particles);
    }

    /** レオロジーモデルで使用するばね係数 */
    double SpringK;

    /** ダッシュポッドの影響をブーストする係数 */
    double DashpodEnvelop;
    
    /** 粒子間の反発力 */
    double E;

    double HamakerConstant;
private:
    std::vector<std::unique_ptr<fj::Particle>> m_particles;

    std::vector<std::unique_ptr<fj::SimulationProfile>> m_profiles;

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
