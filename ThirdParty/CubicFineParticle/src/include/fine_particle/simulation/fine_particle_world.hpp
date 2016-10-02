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
#include "fine_particle/additional/additional_procedure.hpp"
#include "fine_particle/additional/profile/mohr_stress_circle_profile.hpp"
#include "fine_particle/additional/povray/povray_output.hpp"
#include "fine_particle/simulation/particle/particle.hpp"
#include "fine_particle/simulation/bullet_algorithm/fine_particle_simulation_collision_configuration.hpp"


namespace fj {
    class Particle;
    class FineParticleWorld;
}

/** シミュレーションが進行する空間 */
class fj::FineParticleWorld
{
    using TimeStep = btScalar;
    
    /** 粉体粒子間の情報を計算すると同時に保持する */
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
        // btGhostObjectがオーバラップしたオブジェクトを検知できるようにコールバックを登録しておく
        m_world->getBroadphase()->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());
    }

    ~FineParticleWorld() = default;

    /** シミュレーションの終了後に呼び出してください. 今のところプロファイリングの終了処理に使ってます.
     * 呼び出さなくてもシミュレーションにおけるメモリリークは起きません */
    void terminate();
    
    /** タイムステップ分だけシミュレーションを進める 
     * @param timestep シミュレーションの間隔. 0.0001くらい短い方が安定する. */
    void stepSimulation(btScalar timestep);
    
    /** シミュレーション対象を追加する.
     * この関数を使って登録した剛体は、プログラム側で解放されます
     * @param body fj::Particle以外の剛体 */
    void addRigidBody(std::unique_ptr<btRigidBody> body);

    /** シミュレーション対象を追加する.
     * この関数を使って登録した剛体はユーザが責任を持ってメモリを解放してください 
     * @param body fj::Particle以外の剛体 */
    void addCollisionObject(btCollisionObject* body);

    void addParticle(std::unique_ptr<fj::Particle> body);

    /** 指定された粒子を削除する 
     * @param particle 削除対象. 存在しなかった場合の挙動は未定義.
     * @pre particleはシミュレーションに組み込まれているインスタンスである
     * @pre particleはnullptrでない. */
    void removeParticle(fj::Particle*const particle);
    
    /** シミュレーションする空間における重力の方向を指定する.
     * @param gravity 重力の方向. すべての要素が有限値であればどんなベクトルでもいい. */
    void setGravity(const btVector3& gravity);

    
    //------------------ Profiling -----------------------------//
    /** 追加処理を追加する. テンプレート引数が fj::AdditionalProcedure を継承していないとコンパイルエラーになる.
     * @pre プロファイルが fj::SimulationProfile::Priority 順に並んでいる.
     * @post プロファイルが fj::SimulationProfile::Priority 順に並んでいる.*/
    template<class T>
    T*const addProfileSystem()
    {
        std::unique_ptr<T> additionalProcedure(new T(*this));
        T*const ptr = additionalProcedure.get();
        
            // 挿入箇所を探索
            const auto at = std::find_if(m_profiles.begin(), m_profiles.end()
                                         , [&](std::unique_ptr<fj::AdditionalProcedure>& containedProfile){
                                             return additionalProcedure->getPriorityAdUInt() <= containedProfile->getPriorityAdUInt();
                                         });
        
        m_profiles.insert(at, std::move(additionalProcedure));
        return ptr;
    }
    
private:

    //--------------------------- 粉体の力学計算 ------------------------------------------------------------------//
    /** 粉体粒子にかかる接触力と静電気力を計算する */
    void accumulateFineParticleForce(const btScalar timestep);

    /** 各粒子の接触力を算出する
     * @pre m_particlesに格納されているすべての粒子において, 各粒子が保持する接触力が0である
     */
    void applyContactForce(const FineParticlesContactInfo& contactInfo);

    /** 粒子間の垂直方向の力を算出&適用する */
    void applyNormalComponentContactForce(const FineParticlesContactInfo& contactInfo, const btScalar overlap)const;

    void applyTangentialComponentContactForce(const FineParticlesContactInfo& contactInfo)const;

    /** とりあえず2つの粒子のダッシュポッド強度の平均をとる */
    btScalar computeDashpodEnvelope(const fj::Particle& particle1, const fj::Particle& particle2)const;
    
    /** 換算質量を求める 
     * @pre particle1, particle2の両方の質量が生の値をもつ
     */
    btScalar computeReducedMass(const fj::Particle& particle1, const fj::Particle& particle2)const;

    /** 各粒子にかかるファンデルワールス力を算出&適用する */
    void applyVandeerWaalsForce(const FineParticlesContactInfo& contactInfo)const;

    /** 粉体崩壊曲線にもとづいて各粒子の動きを制限する */
    void updateParticleCollapse(const btScalar timestep);

    bool shouldCollapse(const fj::Particle& particle)const;
    
    /** このワールドがシミュレーション対象としている物体すべてを動かす */
    void updateAllObjectTransform(const btScalar timestep);

    
    
    //------------------------------ Profiling -----------------------------------------------------------------------//
    /** 
     @pre fj::FineParticleWorld::stepSImulation関数で, どの力学計算関数よりも先に呼ばれる */
    void startProfiling();

    /**
     @pre fj::FineParticleWorld::stepSImulation関数で, どの力学計算関数よりも後に呼ばれる */
    void endProfiling();
    
    void terminateProfiles();
    
    
    static void IncrementSimulationStep()
    {
        ++s_simulationStep;
    }

public:
    static unsigned int GetSimulationStep()
    {
        return s_simulationStep;
    }
    
    const std::vector<std::unique_ptr<fj::Particle>>& getParticles()const
    {
        return std::cref(m_particles);
    }

    
    
    
    //------------------------------- 粉体シミュレーションにおけるパラメータ --------------------------------------------//
    /** レオロジーモデルで使用するばね係数 */
    double SpringK;
    
    /** 粒子間の反発力 */
    double E;

    /** ファンデルワールス力による付着度 */
    double HamakerConstant;
    
private:
    
    std::vector<std::unique_ptr<fj::Particle>> m_particles;

    std::vector<std::unique_ptr<fj::AdditionalProcedure>> m_profiles;

    static unsigned int s_simulationStep;
    
    
    //---------------- Bullet Physicsのフレームワークを利用するためのインスタンス ----------------------//
    /** Bullet Physicsは生ポインタで全ての処理をするので, メモリの管理はユーザ側でしなくてはならない
     * Bullet Physicsの中でシミュレーション対象となる剛体のメモリ管理用のコンテナ */
    std::vector<std::unique_ptr<btRigidBody>> m_rigidBody;

    // Bullet Physicsを利用するために最低限必要なインスタンス
    std::unique_ptr<btCollisionConfiguration> m_collisionConfiguration;
    std::unique_ptr<btDispatcher> m_dispatcher;
    std::unique_ptr<btBroadphaseInterface> m_pairCache;
    std::unique_ptr<btConstraintSolver> m_constraintSolver;
    std::unique_ptr<btDiscreteDynamicsWorld> m_world;
};

#endif /* fine_particle_world_hpp */
