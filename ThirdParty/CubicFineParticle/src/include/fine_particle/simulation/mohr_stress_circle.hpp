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

namespace fj {
    
    // ワーレンス・スプリング曲線を定義するためのパラメータ
    struct WarrenSpringParameter
    {
        WarrenSpringParameter()
        : SheerIndex(1)
        , Adhesion(1)
        , Collapsibility(1)
        {
            
        }
        
        double SheerIndex; //剪断指数→粉体崩壊曲線の曲率に対応する
        double Adhesion; // 粘着力→大きいほど崩壊しにくくなる. 粉体崩壊曲線のτ切片に対応する.
        double Collapsibility; //垂直応力を大きくしたときの崩壊のしやすさ。粉体崩壊曲線の傾きに対応する.
    };
    
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
    void rebuildMohrCircle(const btMatrix3x3& rotateMatrix);
    
    void clearContactForce();
    
    bool hasIntersectionPoint(const fj::WarrenSpringParameter& warrenSpringParameter)const;
    
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
    void rebuildCircleCenterAndRadius(const btMatrix3x3& rotateMatrix);
    
    NormalStressContainer computeNormalStress(const btMatrix3x3& rotateMatrix)const;
    
private:
    /** 接触している粒子から受けてる力. 1つの接触につき1つの力が保持される. */
    ContactForceContainer m_contactForce;
    
    fj::DiscritizedParticleShape::ShapeType m_discretizedShapeType;
};

#endif /* mohr_stress_circle_hpp */
