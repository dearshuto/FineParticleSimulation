//
//  povray_output.cpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/08/25.
//
//

#include <iostream>
#include <fstream>
#include "simulation/fine_particle_world.hpp"
#include "simulation/particle.hpp"
#include "povray/povray_output.hpp"

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
    
    std::string POV;
    POV += "#include\"Colors.inc\"";
    POV += "camera {";
    POV += "   perspective";
    POV += "    location <-20,10,0>";
    POV += "    angle 60";
    POV += "     up <0,1,0>";
    POV += "    look_at <0,0,3>";
    POV += "  }";
    
    
    POV += "light_source{";
    POV += "    <0, 50, 0>";
    POV += "    color 1.5";
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
    POV += "        checker White*1.2, color rgb<0.5,0.9,0.9>*0.5";
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
        
        POV += ",2";//radius
        POV += "    texture{ pigment{ color rgb<1.0, 1.0, 1.0> }}";
        POV += "}";
    }
    
    output << POV;
    
    return true;
}