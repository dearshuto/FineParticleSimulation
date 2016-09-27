//
//  math_function_c2.hpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/09/27.
//
//

#ifndef math_function_c2_hpp
#define math_function_c2_hpp

namespace fj {
    class MathFunctionC2;
}

/** 1回微分可能な関数 */
class fj::MathFunctionC2
{
public:
    MathFunctionC2() = default;
    virtual~MathFunctionC2() = default;
    
    virtual double compute(const double x)const = 0;
    
    virtual double computeGradient(const double x)const = 0;
    
    virtual double computeLaplacian(const double x)const = 0;
};

#endif /* math_function_c1_hpp */
