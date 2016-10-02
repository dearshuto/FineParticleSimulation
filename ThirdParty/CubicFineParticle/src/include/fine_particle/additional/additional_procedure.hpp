//
//  additional_procedure.hpp
//  CubicFineParticleSimulation
//
//  Created by Shuto Shikama on 2016/10/02.
//
//

#ifndef additional_procedure_hpp
#define additional_procedure_hpp

#include <string>

namespace fj {
    class FineParticleWorld;
    class AdditionalProcedure;
}

class fj::AdditionalProcedure
{
protected:
    enum class Priority : unsigned int
    {
        kI_dont_care,
        kAbsolutelyLast,
    };
public:
    
    AdditionalProcedure(const Priority priority, const fj::FineParticleWorld& world, const std::string& outputDirectory = "./")
    : m_priority(priority)
    , m_world(world)
    , m_outputDirectory(outputDirectory)
    {
        
    }
    
    virtual void startSimulationProfile() = 0;
    
    virtual void endSimulationProfile() = 0;
    
    virtual void terminate() = 0;
    
    unsigned int getPriorityAdUInt()const
    {
        return static_cast<unsigned int>(m_priority);
    }

    const fj::FineParticleWorld& getFineParticleWorld()const
    {
        return m_world;
    }
    
    const std::string& getOutputDirectory()const
    {
        return m_outputDirectory;
    }
    
    void setOutputDirectory(const std::string& outputDirectory)
    {
        m_outputDirectory = outputDirectory;
    }
private:
    
    const Priority m_priority;
    
    const fj::FineParticleWorld& m_world;
    
    std::string m_outputDirectory;
};

#endif /* additional_procedure_hpp */
