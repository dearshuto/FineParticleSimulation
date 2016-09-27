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
    class NewtonMethod;
}

/** ニュートン法を利用した最短距離を求めるソルバ. とりあえず粉体崩壊曲線に特化した実装をする. 需要があればちゃんと分岐を考えるのもあり. */
class fj::NewtonMethod : public fj::CircleExplicitFunctionClosestPointSolver
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
    
    fj::Circle::Position2D computeClosestPoint(const std::function<double(double)>& explicitFunction, const fj::Circle& circle) const override;
};

#endif /* newton_method_hpp */
