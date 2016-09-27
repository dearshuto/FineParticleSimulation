//
//  newton_method.cpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/09/27.
//
//

#include <numeric>
#include "fine_particle/shape_2d/newton_method.hpp"

fj::Circle::Position2D fj::NewtonMethod::computeClosestPoint(const std::function<double (double)> &explicitFunction, const fj::Circle &circle)const
{
    return {std::numeric_limits<double>::infinity()};
}
