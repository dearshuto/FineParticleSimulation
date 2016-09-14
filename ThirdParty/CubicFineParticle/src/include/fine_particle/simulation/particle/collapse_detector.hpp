//
//  collapse_detector.hpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/09/11.
//
//

#ifndef collapse_detector_hpp
#define collapse_detector_hpp

#include "particle.hpp"

namespace fj {
    class Particle;
    class MohrStressCircle;
}

class fj::Particle::CollapseDetector
{
public:
    CollapseDetector() = default;
    virtual~CollapseDetector() = default;
    
    virtual bool shouldCallapse(const fj::Particle& particle)const;
    
protected:
    fj::MohrStressCircle generateMohrStressCircle(const fj::Particle& particle)const;
};

#endif /* collapse_detector_hpp */
