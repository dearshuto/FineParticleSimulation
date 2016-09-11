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

namespace fj {
    struct WarrenSpringParameter
    {
        double SheerIndex; //剪断指数→粉体崩壊曲線の曲率に対応する
        double Adhesion; // 粘着力→大きいほど崩壊しにくくなる. 粉体崩壊曲線のτ切片に対応する.
        double Collapsibility; //垂直応力を大きくしたときの崩壊のしやすさ。粉体崩壊曲線の傾きに対応する.
    };
    
    class MohrStressCircle;
}

class fj::MohrStressCircle
{
public:
    MohrStressCircle() = default;
    ~MohrStressCircle() = default;
    
    MohrStressCircle(const size_t size)
    : m_normalStress(size)
    {
        
    }
    
    void addNormalStress(const double normalStress);
    
    void rebuildMohrCircle();
    
    bool hasIntersectionPoint(const fj::WarrenSpringParameter warrenSpringParameter);
    
private:
    std::array<btScalar, 2> m_center;
    
    btScalar m_radius;
    
    std::vector<double> m_normalStress;
};

#endif /* mohr_stress_circle_hpp */
