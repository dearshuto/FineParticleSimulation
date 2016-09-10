//
//  discritized_particle_shape.cpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/09/10.
//
//

#include <btBulletDynamicsCommon.h>
#include <btBulletCollisionCommon.h>

#include "fine_particle/simulation/discritized_particle_shape.hpp"

std::vector<btVector3> fj::DiscritizedParticleShape::GetDiscritizedParticleShapeNormal(const fj::DiscritizedParticleShape::ShapeType type)
{
    switch (static_cast<int>(type))
    {
        case static_cast<int>(ShapeType::kCube):
            return {
                {1, 0, 0},
                {-1, 0, 0},
                {0, 1, 0},
                {0, -1, 0},
                {0, 0, 1},
                {0, 0, -1},
            };
            
        default:
            return {{0, 0, 0}};
    }
    
}