//
//  fine_particle_simulation_collision_configuration.hpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/09/15.
//
//

#ifndef fine_particle_simulation_collision_configuration_hpp
#define fine_particle_simulation_collision_configuration_hpp

#include <btBulletDynamicsCommon.h>
#include "fine_fine_collision_algorithm.hpp"

namespace fj {
    class FineParticleSimulationCollisionConfiguration;
}

class fj::FineParticleSimulationCollisionConfiguration : public btDefaultCollisionConfiguration
{
    typedef btDefaultCollisionConfiguration Super;
public:
    ~FineParticleSimulationCollisionConfiguration()
    {
        m_finefine->~btCollisionAlgorithmCreateFunc();
        btAlignedFree(m_finefine);
    }
    
    FineParticleSimulationCollisionConfiguration(const btDefaultCollisionConstructionInfo& constructionInfo = btDefaultCollisionConstructionInfo())
    : Super(constructionInfo)
    {
        void* mem = btAlignedAlloc (sizeof(fj::FineFineCollisionAlgorithm::CreateFunc),16);
        m_finefine = new (mem) fj::FineFineCollisionAlgorithm::CreateFunc;
    }
    
    btCollisionAlgorithmCreateFunc* getCollisionAlgorithmCreateFunc(int proxyType0,int proxyType1)override;
    
private:
    btCollisionAlgorithmCreateFunc* m_finefine;
};

#endif /* fine_particle_simulation_collision_configuration_hpp */
