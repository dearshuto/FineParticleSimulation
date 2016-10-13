//
//  mohr_stress_circle.hpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/09/11.
//
//

#ifndef mohr_stress_circle_hpp
#define mohr_stress_circle_hpp

#include <array>
#include <tuple>
#include <vector>
#include <btBulletDynamicsCommon.h>
#include "fine_particle/simulation/particle/discritized_particle_shape.hpp"
#include "fine_particle/shape_2d/circle.hpp"
#include "fine_particle/shape_2d/warren_spring_curve.hpp"

namespace fj {
    class MohrStressCircle;
}

/** モール応力円 */
class fj::MohrStressCircle : public fj::Circle
{
    typedef fj::Circle Super;
    typedef std::vector<btVector3> ContactForceContainer;
    typedef std::vector<btScalar> NormalStressContainer;
public:
    MohrStressCircle() = delete;
    ~MohrStressCircle() = default;
    
    MohrStressCircle(const fj::DiscritizedParticleShape::ShapeType shapeType)
    : m_discretizedShapeType(shapeType)
    {

    }
    
    /** @param normalStress 有限な値である力*/
    void addContactForce(const btVector3& normalStress);
    
    /** モール応力円の中心と半径を再計算する.*/
    void rebuildMohrCircle(const btQuaternion& rotateMatrix);
    
    void clearContactForce();
    
    const Position2D& getCenter()const
    {
        return Super::Center;
    }

    const ContactForceContainer& getContactForceContainer()const
    {
        return m_contactForce;
    }
    
    const fj::DiscritizedParticleShape::ShapeType getDiscretizedShapeType()const
    {
        return m_discretizedShapeType;
    }
    
    const btScalar getRadius()const
    {
        return Super::Radius;
    }
    
private:
    void rebuildCircleCenterAndRadius(const btQuaternion& rotateMatrix);
    
    NormalStressContainer computeNormalStress(const btQuaternion& rotateMatrix)const;
    
private:
    /** 接触している粒子から受けてる力. 1つの接触につき1つの力が保持される. */
    ContactForceContainer m_contactForce;
    
    fj::DiscritizedParticleShape::ShapeType m_discretizedShapeType;
};

#endif /* mohr_stress_circle_hpp */
