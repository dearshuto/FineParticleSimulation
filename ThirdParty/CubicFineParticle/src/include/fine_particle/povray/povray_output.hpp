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
private:
    struct Vector3D
    {
        Vector3D()
        : X(0), Y(0), Z(0)
        {
            
        }
        
        double X;
        double Y;
        double Z;
    };
public:
    struct CameraInfomation
    {
        Vector3D Location;
        Vector3D LookAt;
    };
public:
    POVrayOutput() = delete;
    ~POVrayOutput() = default;
    
    POVrayOutput( std::weak_ptr<fj::FineParticleWorld> world)
    : m_world(world)
    {
        
    }
    
    bool saveToFile(const std::string& filename)const;
    
    CameraInfomation* getCameraInformationPtr()
    {
        return &m_cameraInfomation;
    }

    const CameraInfomation& getCameraInformation()const
    {
        return m_cameraInfomation;
    }

private:
    const std::weak_ptr<fj::FineParticleWorld> m_world;
    
    CameraInfomation m_cameraInfomation;
};

#endif /* povray_output_hpp */
