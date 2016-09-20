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
#include "discritized_particle_shape.hpp"


namespace fj {
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
