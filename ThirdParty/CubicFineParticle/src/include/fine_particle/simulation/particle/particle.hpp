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
#include "fine_particle/simulation/mohr_stress_circle.hpp"
#include "fine_particle/shape_2d/warren_spring_curve.hpp"
#include "discritized_particle_shape.hpp"


namespace fj {
    class FineParticleShape;
    class Particle;
}

class fj::Particle : public btRigidBody
{
private:
    typedef btRigidBody Super;
    typedef std::vector<btVector3> ContactForceContainer;

    struct FineParticleCollapseFactor
    {
        // ShapeTypeを必ず指定させるためのコンストラクタ
        FineParticleCollapseFactor(const fj::DiscritizedParticleShape::ShapeType shapeType)
        : MohrStressCircle(shapeType){}
        
        /** 粉体崩壊曲線を定義するのに必要なパラメータ */
        fj::WarrenSpringCurve WarrenSpringCurve;
        
        fj::MohrStressCircle MohrStressCircle;
    };

    struct RheorogyModelParameter
    {
        RheorogyModelParameter()
        : DashpodEnvelope(1.0)
        {}
        
        /** ダッシュポッドの影響をブーストする係数 */
        btScalar DashpodEnvelope;
    };
public:
    class CollapseDetector;
public:
    Particle() = delete;
    ~Particle() = default;
    
    Particle(const fj::DiscritizedParticleShape::ShapeType shapeType, const btRigidBodyConstructionInfo& info, std::unique_ptr<btMotionState> motionState)
    : btRigidBody(info)
    , m_motionState( std::move(motionState) )
    , m_mass(info.m_mass)
    , m_collapseFactor(shapeType)
    {
        // アップキャストするために自分の情報をもたせておく
        m_internalType = btCollisionObject::CO_RIGID_BODY | btCollisionObject::CO_USER_TYPE;
    }
    
    
    
    
    //---------- Static Functions --------------------------------------------
    
    static std::unique_ptr<fj::Particle> generateParticle(const fj::DiscritizedParticleShape::ShapeType type, const btVector3& position);
    
    /** btCollisionObjectをfj::Particleにアップキャストする.
     * キャストに失敗するとnullptrを返す. */
    static fj::Particle* upcast(btCollisionObject* colObj)
    {
        if (colObj->getInternalType()&btCollisionObject::CO_USER_TYPE)
            return (fj::Particle*)colObj;
        return nullptr;
    }
    
    /** btCollisionObjectをfj::Particleにアップキャストするconst版.
     * キャストに失敗するとnullptrを返す. */
    static const fj::Particle* upcast(const btCollisionObject* colObj)
    {
        if (colObj->getInternalType()&btCollisionObject::CO_USER_TYPE)
            return (const fj::Particle*)colObj;
        return nullptr;
    }
    
    
    
    
    //---------- Public Member Funcsions ---------------------------------------
    void updateCollapseStatus();
    
    void addContactForce(const btVector3& constctForce);
    
    /// 「崩壊=粒子が移動する」なので, 接触力を粒子に与える.
    void collapse();
    
    /// 摩擦力によって動きが止まる現象を表現する
    /// @pre 接触力が計算済みでモール応力円が構成済みである
    void lockWithFriction();
    
    void clearContactForce();
    
    
    
    
    //---------- Public Getters -------------------------------------------------
    
    const FineParticleCollapseFactor& getFineParticleCollapseFactor()const
    {
        return m_collapseFactor;
    }
    
    const ContactForceContainer& getContactForceContainer()const
    {
        return getMohrStressCircle().getContactForceContainer();
    }

    fj::DiscritizedParticleShape::ShapeType getDiscretizedShapeType()const
    {
        return getFineParticleCollapseFactor().MohrStressCircle.getDiscretizedShapeType();
    }
    
    const RheorogyModelParameter& getRheorogyModelParameter()const
    {
        return m_rheorogyModelParameter;
    }

    RheorogyModelParameter* getRheorogyModelParameterPtr()
    {
        return &m_rheorogyModelParameter;
    }

    const fj::WarrenSpringCurve& getWarrenSpringCurve()const
    {
        return getFineParticleCollapseFactor().WarrenSpringCurve;
    }

    fj::WarrenSpringCurve* getWarrenSpringCurvePtr()
    {
        return &(getFineParticleCollapseFactorPtr()->WarrenSpringCurve);
    }

    const fj::MohrStressCircle& getMohrStressCircle()const
    {
        return getFineParticleCollapseFactor().MohrStressCircle;
    }
    
    btScalar getMass()const
    {
        return m_mass;
    }
    
    btVector3 getPosition()const;
    
    btScalar getRadius()const;
    
    
    
    
    //---------- Public Setters -----------------------------------------------
    void setCollapseDetector(const std::weak_ptr<CollapseDetector>& collapseDetector)
    {
        m_collapseDetector = collapseDetector;
    }
    
    
    
    
    //---------- Private Getters ----------------------------------------------
private:
    FineParticleCollapseFactor* getFineParticleCollapseFactorPtr()
    {
        return &m_collapseFactor;
    }
    
    fj::MohrStressCircle* getMohrStressCirclePtr()
    {
        return &(getFineParticleCollapseFactorPtr()->MohrStressCircle);
    }
public:
    static fj::FineParticleShape CollisionShape;
private:

    /** Bulelt Physicsで必要なインスタンスのメモリ管理 */
    std::unique_ptr<btMotionState> m_motionState;
    
    btScalar m_mass;
    
    FineParticleCollapseFactor m_collapseFactor;
    
    RheorogyModelParameter m_rheorogyModelParameter;
    
    /** 崩壊判定のアルゴリズム */
    std::weak_ptr<CollapseDetector> m_collapseDetector;
};

#endif /* particle_hpp */
