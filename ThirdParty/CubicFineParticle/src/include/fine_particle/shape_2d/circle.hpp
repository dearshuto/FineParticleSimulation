//
//  circle.hpp
//  CubicFineParticleSimulation
//
//  Created by Shuto on 2016/09/27.
//
//

#ifndef circle_hpp
#define circle_hpp

namespace fj {
    struct Circle;
}

struct fj::Circle
{
    struct Position2D
    {
        double X;
        double Y;
    };
public:
    Circle(const Position2D& center = {0, 0}, const float radius = 0.0)
    : Center(center)
    , Radius(radius)
    {
        
    }
    
    Position2D Center;
    float Radius;
};

#endif /* circle_hpp */
