//
//  warren_spring_curve.hpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/09/27.
//
//

#ifndef warren_spring_curve_hpp
#define warren_spring_curve_hpp

#include "math_function_c1.hpp"

namespace fj {
    class WarrenSpringCurve;
}

class fj::WarrenSpringCurve : public fj::MathFunctionC1
{
public:
    
    // ワーレンス・スプリング曲線を定義するためのパラメータ
    struct Parameter
    {
        Parameter()
        : SheerIndex(1)
        , Adhesion(1)
        , Collapsibility(1)
        {
            
        }
        
        double SheerIndex; //剪断指数→粉体崩壊曲線の曲率に対応する
        double Adhesion; // 粘着力→大きいほど崩壊しにくくなる. 粉体崩壊曲線のτ切片に対応する.
        double Collapsibility; //垂直応力を大きくしたときの崩壊のしやすさ。粉体崩壊曲線の傾きに対応する.
    };
    
private:
    WarrenSpringCurve() = default;
public:
    ~WarrenSpringCurve() = default;
    
    virtual double compute(const double x)const = 0;
    
    virtual double computeGradient(const double x)const = 0;
};

#endif /* warren_spring_curve_hpp */
