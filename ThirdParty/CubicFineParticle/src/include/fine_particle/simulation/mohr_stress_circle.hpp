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
class fj::MohrStressCircle
{
    typedef std::array<btScalar, 2> Position2D;
    
public:
    MohrStressCircle() = default;
    ~MohrStressCircle() = default;
    
    MohrStressCircle(const fj::DiscritizedParticleShape::ShapeType shapeType)
    : m_discretizedShapeType(shapeType)
    {

    }
    
    /** @param normalStress 有限な値である力*/
    void addNormalStress(const double normalStress);
    
    /** モール応力円の中心と半径を再計算する.*/
    void rebuildMohrCircle();
    
    bool hasIntersectionPoint(const fj::WarrenSpringParameter& warrenSpringParameter)const;
    
    const Position2D& getCenter()const
    {
        return m_center;
    }

    const btScalar getRadius()const
    {
        return m_radius;
    }
    
private:
    Position2D m_center;
    
    btScalar m_radius;
    
    std::vector<double> m_normalStress;
    
    fj::DiscritizedParticleShape::ShapeType m_discretizedShapeType;
};

#endif /* mohr_stress_circle_hpp */
