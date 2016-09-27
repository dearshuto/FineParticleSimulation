//
//  warren_spring_curve.cpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/09/27.
//
//

#include <cmath>
#include "fine_particle/shape_2d/warren_spring_curve.hpp"

double fj::WarrenSpringCurve::compute(const double x)const
{
    const auto& kParameter = getParameter();
    const auto& kTau = kParameter.Adhesion;
    const auto& kSigma = kParameter.Collapsibility;
    const auto& kN = kParameter.SheerIndex;
    
    return kTau * std::pow( (x + kSigma) / kSigma
                           , 1.0/kN);
}

double fj::WarrenSpringCurve::computeGradient(const double x)const
{
    const auto& kParameter = getParameter();
    const auto& kTau = kParameter.Adhesion;
    const auto& kSigma = kParameter.Collapsibility;
    const auto& kN = kParameter.SheerIndex;

    return (kTau / (kN * kSigma)) * std::pow( (x + kSigma) / kSigma
                                             , (1.0 - kN) / kN);
}
