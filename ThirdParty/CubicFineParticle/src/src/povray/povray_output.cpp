//
//  povray_output.cpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/08/25.
//
//

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>
#include "fine_particle/simulation/fine_particle_world.hpp"
#include "fine_particle/simulation/particle/particle.hpp"
#include "fine_particle/povray/povray_output.hpp"

bool fj::POVrayOutput::saveToFile(const std::string &filename)const
{
    std::ofstream output(filename);
    
    if (output.fail())
    {
        std::cout << "cannot open " << filename << std::endl;
        return false;
    }
    
    const auto kWorld = m_world.lock();
    if ( !kWorld )
    {
        std::cout << "The registerd world instance is invalid" << std::endl;
    }
    
    const auto& kPosition = getCameraInformation().Location;
    const auto& kLookAt = getCameraInformation().LookAt;
    
    std::string POV("");
    POV += "#include\"Colors.inc\"";
    POV += "camera {";
    POV += "   perspective";
    POV +=("    location <"
           + std::to_string(kPosition.X)
           + std::string(",")
           + std::to_string(kPosition.Y)
           + std::string(",")
           + std::to_string(kPosition.Z)
           + std::string(">")
           );
           
    POV += "    angle 60";
    POV += "     up <0,1,0>";
    POV +=("    look_at <"
           + std::to_string(kLookAt.X)
           + std::string(",")
           + std::to_string(kLookAt.Y)
           + std::string(",")
           + std::to_string(kLookAt.Z)
           + std::string(">")
           );
    
    POV += "  }";
    
    
    POV += "light_source{";
    POV +=("<"
           + std::to_string(kPosition.X)
           + std::string(",")
           + std::to_string(kPosition.Y)
           + std::string(",")
           + std::to_string(kPosition.Z)
           + std::string(">")
           );
    POV += "    color 1.0";
    POV += "}";
    
    POV += "sky_sphere{";
    POV += "    pigment{";
    POV += "        gradient y";
    POV += "        color_map{";
    POV += "            [ 0.0 White * 0.9 ]";
    POV += "            [ 1.0 color rgb<0.3,0.4,1.2>]";
    POV += "        }";
    POV += "    }";
    POV += "}";
    
    
    POV += "plane{y,0";
    POV += "    pigment{";
    POV += "        checker White*1.2, color rgb<0.5,0.9,0.9>*10";
    POV += "        scale 0.2";
    POV += "    }";
    POV += "    finish{phong 1 reflection 0.3}";
    POV += "}";
    
    for (const auto& particle : kWorld->getParticles())
    {
        btTransform trans;
        particle->getMotionState()->getWorldTransform(trans);
        const auto position = trans.getOrigin();
        POV += "sphere{";

        POV += std::string("<") + std::to_string(position.x()) + "," + std::to_string(position.y()) + "," + std::to_string(position.z()) + std::string(">");
        
        POV += "," + std::to_string(particle->getRadius());
        POV += "    texture{ pigment{ color rgb<1.0, 1.0, 1.0> }}";
        POV += "}";
    }
    
    output << POV;
    
    return true;
}