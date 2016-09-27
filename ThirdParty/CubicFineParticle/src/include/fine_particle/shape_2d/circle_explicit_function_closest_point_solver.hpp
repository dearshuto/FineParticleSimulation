//
//  circle_explosit_function_explicit_solver.hpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/09/27.
//
//

#ifndef circle_explicit_function_closest_point_solver_hpp
#define circle_explicit_function_closest_point_solver_hpp

#include <functional>
#include "fine_particle/shape_2d/circle.hpp"

namespace fj {
    class CircleExplicitFunctionClosestPointSolver;
}

/** 陽関数と円との最短距離を求める */
class fj::CircleExplicitFunctionClosestPointSolver
{
protected:
    CircleExplicitFunctionClosestPointSolver() = default;
public:
    virtual~CircleExplicitFunctionClosestPointSolver() = default;
    
    virtual fj::Circle::Position2D computeClosestPoint(const std::function<double(double)>& explicitFunction, const fj::Circle& circle)const = 0;
};

#endif /* circle_explosit_function_interaction_solver_hpp */
