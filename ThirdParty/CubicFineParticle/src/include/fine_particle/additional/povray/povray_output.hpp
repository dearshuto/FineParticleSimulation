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
#include "fine_particle/additional/additional_procedure.hpp"

namespace fj {
    class FineParticleWorld;
    class POVRayOutput;
}

/** シーンをPOV-Ray形式で出力する.
 * x-z平面に乗っている無限平面と粒子をレンダリングするシーンを出力する. */
class fj::POVRayOutput : public fj::AdditionalProcedure
{
private:
    typedef fj::AdditionalProcedure Super;
    
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
    POVRayOutput() = delete;
    ~POVRayOutput() = default;
    
    POVRayOutput(const fj::FineParticleWorld& world)
    : Super(Priority::kI_dont_care, world)
    {
        
    }
    
    void startSimulationProfile() override;
    
    void endSimulationProfile()  override;
    
    void terminate() override;
    
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
    
    CameraInfomation m_cameraInfomation;
};

#endif /* povray_output_hpp */
