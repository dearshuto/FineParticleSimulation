//
//  fine_fine_collision_algorithm.hpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/09/15.
//
//

#ifndef fine_fine_collision_algorithm_hpp
#define fine_fine_collision_algorithm_hpp

#include <btBulletDynamicsCommon.h>

namespace fj {
    class FineFineCollisionAlgorithm;
}

class fj::FineFineCollisionAlgorithm : public btActivatingCollisionAlgorithm
{
    typedef btActivatingCollisionAlgorithm Super;
public:
    FineFineCollisionAlgorithm( const btCollisionAlgorithmConstructionInfo& ci
                               ,const btCollisionObjectWrapper* col0Wrap
                               ,const btCollisionObjectWrapper* col1Wrap)
    : Super(ci, col0Wrap, col1Wrap)
    {
        
    }
    
    FineFineCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci)
    : Super(ci)
    {
        
    }
    
    void processCollision (const btCollisionObjectWrapper* body0Wrap,const btCollisionObjectWrapper* body1Wrap,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut)override;
    
    btScalar calculateTimeOfImpact(btCollisionObject* body0,btCollisionObject* body1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut) override
    {
        // not yet
        return btScalar(1.0);
    }
    
    void	getAllContactManifolds(btManifoldArray&	manifoldArray) override;
    
    struct CreateFunc :public 	btCollisionAlgorithmCreateFunc
    {
        virtual	btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci, const btCollisionObjectWrapper* col0Wrap,const btCollisionObjectWrapper* col1Wrap)
        {
            void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(fj::FineFineCollisionAlgorithm));
            return new(mem) fj::FineFineCollisionAlgorithm(ci,col0Wrap,col1Wrap);
        }
    };

};

#endif /* fine_fine_collision_algorithm_hpp */
