//
//  fine_particle_shape.hpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/09/15.
//
//

#ifndef fine_particle_shape_hpp
#define fine_particle_shape_hpp

#include <btBulletCollisionCommon.h>

namespace fj {
    class FineParticleShape;
}

class fj::FineParticleShape : public btSphereShape
{
    typedef btSphereShape Super;
public:
    FineParticleShape() = delete;
    ~FineParticleShape() = default;
    
    FineParticleShape(const btScalar radius)
    : Super(radius)
    , m_rigidRadius(radius / 2.0)
    {
        m_shapeType = CUSTOM_POLYHEDRAL_SHAPE_TYPE;
    }
    
    void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const override;
    
    btScalar getRigidRadius()const;
    
    btScalar getEffectRangeRadius()const;
private:
    btScalar m_rigidRadius;
};

#endif /* fine_particle_shape_hpp */
