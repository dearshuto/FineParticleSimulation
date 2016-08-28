//
//  povray_output.hpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/08/25.
//
//

#ifndef povray_output_hpp
#define povray_output_hpp

#include <memory>
#include <vector>

namespace fj {
    class FineParticleWorld;
    class Particle;
    class POVrayOutput;
}

class fj::POVrayOutput
{
public:
    POVrayOutput() = delete;
    ~POVrayOutput() = default;
    
    POVrayOutput( std::weak_ptr<fj::FineParticleWorld> world)
    : m_world(world)
    {
        
    }
    
    bool saveToFile(const std::string& filename)const;
    
private:
    const std::weak_ptr<fj::FineParticleWorld> m_world;
};

#endif /* povray_output_hpp */
