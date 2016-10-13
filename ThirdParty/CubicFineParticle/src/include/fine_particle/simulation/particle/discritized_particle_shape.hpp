//
//  discritized_particle_shape.hpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/09/10.
//
//

#ifndef discritized_particle_shape_hpp
#define discritized_particle_shape_hpp

#include <memory>

class btVector3;
class btMatrix3x3;

namespace fj {
    class DiscritizedParticleShape;
}

/** 粉体粒子を離散化した形状を提供する
 * 離散化形状は法線の集合で表される. */
class fj::DiscritizedParticleShape
{
public:
    enum class ShapeType
    {
        kCube,
    };
    
    class NormalContainer
    {
    public:
        NormalContainer() = default;
        virtual~NormalContainer() = default;
        
        virtual unsigned int size()const = 0;
        virtual const btVector3& get(unsigned int index)const = 0;
        virtual void rotate(const btMatrix3x3& matrix) = 0;
    };
    
private:
    // インスタンス化はしない方針で
    DiscritizedParticleShape() = delete;
    ~DiscritizedParticleShape() = delete;
public:
    static std::unique_ptr<NormalContainer> GetDiscretizedParticleShapeNormal(const ShapeType type);
};

#endif /* discritized_particle_shape_hpp */
