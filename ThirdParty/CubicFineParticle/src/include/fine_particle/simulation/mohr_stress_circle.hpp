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
public:
    MohrStressCircle() = default;
    ~MohrStressCircle() = default;
    
    /** @param size 評価する垂直応力の個数. 指定した方がメモリ効率が良くなる. */
    MohrStressCircle(const size_t size)
    : m_normalStress(size)
    {
        
    }
    
    /** @param normalStress 有限な値である力*/
    void addNormalStress(const double normalStress);
    
    /** モール応力円の中心と半径を再計算する.*/
    void rebuildMohrCircle();
    
    bool hasIntersectionPoint(const fj::WarrenSpringParameter& warrenSpringParameter)const;
    
    const Position2D& getCenter()const
    {
        return Super::Center;
    }

    const btScalar getRadius()const
    {
        return Super::Radius;
    }
    
private:
    std::vector<double> m_normalStress;
};

#endif /* mohr_stress_circle_hpp */
