//
//  joint.hpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/08/25.
//
//

#ifndef joint_hpp
#define joint_hpp

namespace fj {
    class Particle;
    class Joint;
}

class fj::Joint
{
public:
    Joint() = delete;
    ~Joint() = default;
    
    Joint(const fj::Particle* particle1, const fj::Particle* particle2)
    : m_particle1(particle1)
    , m_particle2(particle2)
    {
        
    }
    
    /**
     * 接続している粒子に、距離に応じた力を追加する
     */
    void applyForceToConnectedParticle();
    
private:
    const fj::Particle* m_particle1;
    const fj::Particle* m_particle2;
};

#endif /* joint_hpp */
