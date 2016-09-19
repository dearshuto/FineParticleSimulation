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

/**
 * 粉体粒子を離散化した形状を提供する
 * 離散化形状は法線の集合で表される.
 */
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
