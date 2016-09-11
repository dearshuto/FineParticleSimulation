//
//  discritized_particle_shape.hpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/09/10.
//
//

#ifndef discritized_particle_shape_hpp
#define discritized_particle_shape_hpp

#include <vector>

class btVector3;

namespace fj {
    class DiscritizedParticleShape;
}

class fj::DiscritizedParticleShape
{
public:
    enum class ShapeType
    {
        kCube,
    };
private:
    // インスタンス化はしない方針で
    DiscritizedParticleShape() = delete;
    ~DiscritizedParticleShape() = delete;
public:
    static std::vector<btVector3> GetDiscritizedParticleShapeNormal(const ShapeType type);
};

#endif /* discritized_particle_shape_hpp */
