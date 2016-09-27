//
//  math_function_c1.hpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/09/27.
//
//

#ifndef math_function_c1_hpp
#define math_function_c1_hpp

namespace fj {
    class MathFunctionC1;
}

/** 1回微分可能な関数 */
class fj::MathFunctionC1
{
public:
    MathFunctionC1() = default;
    virtual~MathFunctionC1() = default;
    
    virtual double compute(const double x)const = 0;
    
    virtual double computeGradient(const double x)const = 0;
};

#endif /* math_function_c1_hpp */
