//
//  newton_method.hpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/09/27.
//
//

#ifndef newton_method_hpp
#define newton_method_hpp

#include "circle_explicit_function_closest_point_solver.hpp"

namespace fj {
    class MathFunctionC2;
    class NewtonMethod;
}

/** ニュートン法を利用した最短距離を求めるソルバ. とりあえず粉体崩壊曲線に特化した実装をする. 需要があればちゃんと分岐を考えるのもあり. */
class fj::NewtonMethod
{
private:
    NewtonMethod() = default;
public:
    ~NewtonMethod() = default;
    
    static const fj::NewtonMethod& GetInstance()
    {
        static fj::NewtonMethod instance;
        return instance;
    }
    
    /**
     * @param func 判定条件引数には距離の2乗が渡されます*/
    fj::Circle::Position2D computeClosestPoint(const fj::MathFunctionC2& explicitFunction, const fj::Circle& circle, const std::function<bool(const double distance)>* func = nullptr)const;
};

#endif /* newton_method_hpp */
