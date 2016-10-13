//
//  newton_method.cpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/09/27.
//
//

#include <cmath>
#include <numeric>
#include "fine_particle/shape_2d/math_function_c2.hpp"
#include "fine_particle/shape_2d/newton_method.hpp"

fj::Circle::Position2D fj::NewtonMethod::computeClosestPoint(const fj::MathFunctionC2 &explicitFunction, const fj::Circle &circle, const std::function<bool(double)>*func )const
{
    const auto& kCenterX = circle.Center.X;
    const auto& kCenterY = circle.Center.Y;
    
    // Circleの中心からの距離の2乗を計算する式
    const auto kSquaredDistanceFunction = [&](const double x){
        return std::pow(kCenterX - x, 2.0) + std::pow(kCenterY - explicitFunction.compute(x), 2.0);
    };
    
    // Circleの中心からの距離の変化. これが最小となるときが曲線と中心との距離が最も近いとき.
    const auto kSquaredDistanceFunctionGradient = [&](const double x){
        return 2.0*(kCenterX - x) + 2.0*explicitFunction.compute(x)*explicitFunction.computeGradient(x);
    };
    
    const auto kSquaredDistanceLaplacian = [&](const double x){
        return 2.0*( std::pow(explicitFunction.computeGradient(x), 2.0) + explicitFunction.compute(x) * explicitFunction.computeLaplacian(x) + 1 );
    };
    
    auto initialValue = circle.Center.X;
    
    for (int i = 0; i < 100; i++)
    {
        const double kX = initialValue;
        const double kY = kSquaredDistanceFunctionGradient(kX);
        
        const bool kShouldReturn = func ? (*func)(kSquaredDistanceFunction(kX)) : (kY < 0.000001);
        
        if (kShouldReturn)
        {
            return {kX, explicitFunction.compute(kX)};
        }
        
        const auto kGradient = kSquaredDistanceLaplacian(kX);
        
        // 接線の方程式
        auto kLineEquation = [&](const double x){
            return kGradient*(x - kX) + kY;
        };
        
        // 接戦とx軸との交点を初期値にして再スタート
        initialValue = kX - kY / kLineEquation(kX);
    }
    
    return {std::numeric_limits<double>::infinity()};
}
